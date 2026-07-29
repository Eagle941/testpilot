//! Validated playback samples, interpolation, and range conversion.

pub use crate::error::PlaybackError;

/// One finite scenario value at an explicit non-negative timestamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Scenario-relative timestamp in seconds.
    pub time_seconds: f64,
    /// Finite value in the signal's configured source unit.
    pub value: f64,
}

impl Sample {
    /// Creates a sample after validating its timestamp and value.
    pub fn new(time_seconds: f64, value: f64) -> Result<Self, PlaybackError> {
        if !time_seconds.is_finite() {
            return Err(PlaybackError::NonFiniteTime { time_seconds });
        }
        if time_seconds < 0.0 {
            return Err(PlaybackError::NegativeTime { time_seconds });
        }
        if !value.is_finite() {
            return Err(PlaybackError::NonFiniteValue { value });
        }

        Ok(Self {
            time_seconds,
            value,
        })
    }
}

/// Two time-ordered samples that bound linear interpolation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearSegment {
    start: Sample,
    end: Sample,
}

impl LinearSegment {
    /// Creates a segment whose end timestamp must be greater than its start.
    pub fn new(start: Sample, end: Sample) -> Result<Self, PlaybackError> {
        if end.time_seconds <= start.time_seconds {
            return Err(PlaybackError::NonIncreasingSegment {
                start_seconds: start.time_seconds,
                end_seconds: end.time_seconds,
            });
        }

        Ok(Self { start, end })
    }

    /// Returns the segment's earlier sample.
    pub const fn start(self) -> Sample {
        self.start
    }

    /// Returns the segment's later sample.
    pub const fn end(self) -> Sample {
        self.end
    }

    /// Interpolates the value at a timestamp within this segment, inclusively.
    ///
    /// Exact endpoint timestamps return the corresponding source value without
    /// performing interpolation arithmetic.
    pub fn value_at(self, time_seconds: f64) -> Result<f64, PlaybackError> {
        if !time_seconds.is_finite() {
            return Err(PlaybackError::NonFiniteTime { time_seconds });
        }
        if time_seconds < self.start.time_seconds || time_seconds > self.end.time_seconds {
            return Err(PlaybackError::TimeOutsideSegment {
                time_seconds,
                start_seconds: self.start.time_seconds,
                end_seconds: self.end.time_seconds,
            });
        }
        if time_seconds == self.start.time_seconds {
            return Ok(self.start.value);
        }
        if time_seconds == self.end.time_seconds {
            return Ok(self.end.value);
        }

        let factor = (time_seconds - self.start.time_seconds)
            / (self.end.time_seconds - self.start.time_seconds);
        let value = self.start.value + factor * (self.end.value - self.start.value);
        if !value.is_finite() {
            return Err(PlaybackError::ArithmeticOverflow);
        }
        Ok(value)
    }
}

/// Affine conversion between strictly increasing source and target ranges.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AffineRange {
    source: [f64; 2],
    target: [f64; 2],
}

impl AffineRange {
    /// Creates a conversion after validating both ranges.
    pub fn new(source: [f64; 2], target: [f64; 2]) -> Result<Self, PlaybackError> {
        validate_range("source", source)?;
        validate_range("target", target)?;
        Ok(Self { source, target })
    }

    /// Returns the inclusive source range.
    pub const fn source(self) -> [f64; 2] {
        self.source
    }

    /// Returns the inclusive target range.
    pub const fn target(self) -> [f64; 2] {
        self.target
    }

    /// Converts a finite source value without clamping it.
    ///
    /// Values outside [`Self::source`] are rejected.
    pub fn convert(self, value: f64) -> Result<f64, PlaybackError> {
        if !value.is_finite() {
            return Err(PlaybackError::NonFiniteValue { value });
        }
        if value < self.source[0] || value > self.source[1] {
            return Err(PlaybackError::ValueOutsideSourceRange {
                value,
                minimum: self.source[0],
                maximum: self.source[1],
            });
        }

        let converted = self.target[0]
            + (value - self.source[0]) * (self.target[1] - self.target[0])
                / (self.source[1] - self.source[0]);
        if !converted.is_finite() {
            return Err(PlaybackError::ArithmeticOverflow);
        }
        Ok(converted)
    }
}

fn validate_range(name: &'static str, range: [f64; 2]) -> Result<(), PlaybackError> {
    if !range.iter().all(|endpoint| endpoint.is_finite()) {
        return Err(PlaybackError::InvalidRange {
            name,
            reason: "endpoints must be finite",
        });
    }
    if range[0] >= range[1] {
        return Err(PlaybackError::InvalidRange {
            name,
            reason: "lower endpoint must be less than upper endpoint",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(time_seconds: f64, value: f64) -> Sample {
        match Sample::new(time_seconds, value) {
            Ok(sample) => sample,
            Err(error) => panic!("valid test sample rejected: {error}"),
        }
    }

    #[test]
    fn interpolates_between_irregular_timestamps() {
        let segment = match LinearSegment::new(sample(0.2, -10.0), sample(0.7, 30.0)) {
            Ok(segment) => segment,
            Err(error) => panic!("valid segment rejected: {error}"),
        };

        assert_eq!(segment.value_at(0.2), Ok(-10.0));
        assert_eq!(segment.value_at(0.7), Ok(30.0));
        assert!((segment.value_at(0.45).unwrap_or(f64::NAN) - 10.0).abs() < 1e-12);
    }

    #[test]
    fn rejects_invalid_samples_and_segments() {
        assert!(matches!(
            Sample::new(f64::NAN, 0.0),
            Err(PlaybackError::NonFiniteTime { .. })
        ));
        assert!(matches!(
            Sample::new(-0.1, 0.0),
            Err(PlaybackError::NegativeTime { .. })
        ));
        assert!(matches!(
            Sample::new(0.0, f64::INFINITY),
            Err(PlaybackError::NonFiniteValue { .. })
        ));
        assert!(matches!(
            LinearSegment::new(sample(1.0, 0.0), sample(1.0, 1.0)),
            Err(PlaybackError::NonIncreasingSegment { .. })
        ));
    }

    #[test]
    fn rejects_frame_times_outside_segment() {
        let segment = LinearSegment::new(sample(1.0, 0.0), sample(2.0, 1.0))
            .unwrap_or_else(|error| panic!("valid segment rejected: {error}"));

        assert!(matches!(
            segment.value_at(0.99),
            Err(PlaybackError::TimeOutsideSegment { .. })
        ));
        assert!(matches!(
            segment.value_at(2.01),
            Err(PlaybackError::TimeOutsideSegment { .. })
        ));
    }

    #[test]
    fn converts_affine_ranges_without_clamping() {
        let conversion = AffineRange::new([-100.0, 100.0], [-1.0, 1.0])
            .unwrap_or_else(|error| panic!("valid conversion rejected: {error}"));

        assert_eq!(conversion.convert(-100.0), Ok(-1.0));
        assert_eq!(conversion.convert(0.0), Ok(0.0));
        assert_eq!(conversion.convert(100.0), Ok(1.0));
        assert!(matches!(
            conversion.convert(100.1),
            Err(PlaybackError::ValueOutsideSourceRange { .. })
        ));
    }

    #[test]
    fn rejects_invalid_affine_ranges() {
        assert!(matches!(
            AffineRange::new([0.0, 0.0], [-1.0, 1.0]),
            Err(PlaybackError::InvalidRange { .. })
        ));
        assert!(matches!(
            AffineRange::new([0.0, f64::INFINITY], [-1.0, 1.0]),
            Err(PlaybackError::InvalidRange { .. })
        ));
    }
}
