//! Validated playback samples, interpolation, and range conversion.

use std::time::Duration;

pub use crate::error::PlaybackError;

/// One finite scenario value at an explicit scenario-relative time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Scenario-relative sample time.
    pub time: Duration,
    /// Finite value in the signal's configured source scale.
    pub value: f64,
}

impl Sample {
    /// Creates a sample after validating its value.
    pub fn new(time: Duration, value: f64) -> Result<Sample, PlaybackError> {
        if !value.is_finite() {
            return Err(PlaybackError::NonFiniteValue { value });
        }

        Ok(Sample { time, value })
    }
}

/// Two time-ordered samples that bound time-based linear interpolation.
///
/// A [`LinearSegment`] converts two scenario samples, `(t0, v0)` and
/// `(t1, v1)`, into a value at a requested time `t` using:
///
/// ```text
/// v0 + (t - t0) * (v1 - v0) / (t1 - t0)
/// ```
///
/// The requested time must be between the two sample timestamps, inclusive.
/// This type interpolates along the time series; it does not convert the
/// resulting value between engineering or simulator ranges.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearSegment {
    /// Earlier sample that bounds interpolation from below.
    start: Sample,
    /// Later sample that bounds interpolation from above.
    end: Sample,
}

impl LinearSegment {
    /// Creates a segment whose end timestamp must be greater than its start.
    pub fn new(start: Sample, end: Sample) -> Result<LinearSegment, PlaybackError> {
        if end.time <= start.time {
            return Err(PlaybackError::NonIncreasingSegment {
                start: start.time,
                end: end.time,
            });
        }

        Ok(LinearSegment { start, end })
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
    pub fn value_at(self, time: Duration) -> Result<f64, PlaybackError> {
        if time < self.start.time || time > self.end.time {
            return Err(PlaybackError::TimeOutsideSegment {
                time,
                start: self.start.time,
                end: self.end.time,
            });
        }
        if time == self.start.time {
            return Ok(self.start.value);
        }
        if time == self.end.time {
            return Ok(self.end.value);
        }

        let factor = (time - self.start.time).as_secs_f64()
            / (self.end.time - self.start.time).as_secs_f64();
        let value = self.start.value + factor * (self.end.value - self.start.value);
        if !value.is_finite() {
            return Err(PlaybackError::ArithmeticOverflow);
        }
        Ok(value)
    }
}

/// Converts values from a configured source range to a simulator range.
///
/// [`AffineRange`] applies an affine, or linear-with-offset, mapping after a
/// time-series value has been interpolated. For source range `[x0, x1]`, target
/// range `[y0, y1]`, and source value `x`, the converted value is:
///
/// ```text
/// y0 + (x - x0) * (y1 - y0) / (x1 - x0)
/// ```
///
/// For example, a source range of `[-25.0, 25.0]` can be mapped to the
/// simulator range `[-16383.0, 16384.0]`. Values outside the source range are
/// rejected rather than clamped. This type converts value scales; it does not
/// interpolate between time-series samples. In the playback pipeline,
/// [`LinearSegment`] runs first and [`AffineRange`] runs second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AffineRange {
    /// Inclusive source range, [minimum, maximum].
    source: [f64; 2],
    /// Inclusive target range, [minimum, maximum].
    target: [f64; 2],
}

impl AffineRange {
    /// Creates a conversion after validating both ranges.
    pub fn new(source: [f64; 2], target: [f64; 2]) -> Result<AffineRange, PlaybackError> {
        Self::validate_range("source", source)?;
        Self::validate_range("target", target)?;
        Ok(AffineRange { source, target })
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
    /// Values outside [`AffineRange::source`] are rejected.
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
}

#[cfg(test)]
mod tests {
    use super::*;

    fn time(seconds: f64) -> Duration {
        Duration::try_from_secs_f64(seconds).unwrap()
    }

    fn sample(seconds: f64, value: f64) -> Sample {
        match Sample::new(time(seconds), value) {
            Ok(sample) => sample,
            Err(error) => panic!("valid test sample rejected: {error}"),
        }
    }

    #[test]
    fn preserves_sample_and_segment_values() {
        let start = sample(0.2, -10.0);
        let end = sample(0.7, 30.0);
        let segment = LinearSegment::new(start, end)
            .unwrap_or_else(|error| panic!("valid segment rejected: {error}"));

        assert_eq!(
            start,
            Sample {
                time: time(0.2),
                value: -10.0
            }
        );
        assert_eq!(segment.start(), start);
        assert_eq!(segment.end(), end);
    }

    #[test]
    fn interpolates_between_irregular_timestamps() {
        let segment = match LinearSegment::new(sample(0.2, -10.0), sample(0.7, 30.0)) {
            Ok(segment) => segment,
            Err(error) => panic!("valid segment rejected: {error}"),
        };

        assert_eq!(segment.value_at(time(0.2)), Ok(-10.0));
        assert_eq!(segment.value_at(time(0.7)), Ok(30.0));
        assert!((segment.value_at(time(0.45)).unwrap_or(f64::NAN) - 10.0).abs() < 1e-12);
    }

    #[test]
    fn rejects_invalid_samples_and_segments() {
        assert!(matches!(
            Sample::new(Duration::ZERO, f64::INFINITY),
            Err(PlaybackError::NonFiniteValue { .. })
        ));
        assert!(matches!(
            LinearSegment::new(sample(1.0, 0.0), sample(1.0, 1.0)),
            Err(PlaybackError::NonIncreasingSegment { .. })
        ));
    }

    #[test]
    fn rejects_interpolation_overflow() {
        let segment = LinearSegment::new(sample(0.0, f64::MAX), sample(2.0, -f64::MAX))
            .unwrap_or_else(|error| panic!("valid segment rejected: {error}"));

        assert!(matches!(
            segment.value_at(time(1.0)),
            Err(PlaybackError::ArithmeticOverflow)
        ));
    }

    #[test]
    fn rejects_frame_times_outside_segment() {
        let segment = LinearSegment::new(sample(1.0, 0.0), sample(2.0, 1.0))
            .unwrap_or_else(|error| panic!("valid segment rejected: {error}"));

        assert!(matches!(
            segment.value_at(time(0.99)),
            Err(PlaybackError::TimeOutsideSegment { .. })
        ));
        assert!(matches!(
            segment.value_at(time(2.01)),
            Err(PlaybackError::TimeOutsideSegment { .. })
        ));
    }

    #[test]
    fn converts_affine_ranges_without_clamping() {
        let conversion = AffineRange::new([-100.0, 100.0], [-1.0, 1.0])
            .unwrap_or_else(|error| panic!("valid conversion rejected: {error}"));

        assert_eq!(conversion.convert(-100.0), Ok(-1.0));
        assert_eq!(conversion.convert(0.0), Ok(0.0));
        assert_eq!(conversion.source(), [-100.0, 100.0]);
        assert_eq!(conversion.target(), [-1.0, 1.0]);
        assert_eq!(conversion.convert(100.0), Ok(1.0));
        assert!(matches!(
            conversion.convert(f64::NAN),
            Err(PlaybackError::NonFiniteValue { .. })
        ));
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
            Err(PlaybackError::InvalidRange { name: "source", .. })
        ));
        assert!(matches!(
            AffineRange::new([0.0, 1.0], [1.0, 1.0]),
            Err(PlaybackError::InvalidRange { name: "target", .. })
        ));
        let conversion = AffineRange::new([0.0, 1.0], [-f64::MAX, f64::MAX])
            .unwrap_or_else(|error| panic!("valid conversion rejected: {error}"));
        assert!(matches!(
            conversion.convert(0.5),
            Err(PlaybackError::ArithmeticOverflow)
        ));
    }
}
