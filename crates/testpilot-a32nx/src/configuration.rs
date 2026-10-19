//! Shared validation and native numeric readback for A32NX initialisation.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

/// Decodes an A32NX discrete ARINC word, accepting only Normal Operation status.
pub(super) fn decode_discrete_word(
    variable: &'static str,
    value: f64,
) -> Result<Option<u32>, InitialisationError> {
    if !value.is_finite()
        || !(0.0..=0x3_ffff_ffff_u64 as f64).contains(&value)
        || value.fract() != 0.0
    {
        return Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "packed ARINC word must have a 32-bit payload and two-bit status",
        });
    }
    let packed = value as u64;
    let payload = f64::from(f32::from_bits(packed as u32));
    if !payload.is_finite()
        || !(0.0..=f64::from(u32::MAX)).contains(&payload)
        || payload.fract() != 0.0
    {
        return Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "discrete ARINC payload must encode a non-negative integer as f32",
        });
    }
    Ok((packed >> 32 == 3).then_some(payload as u32))
}

/// Appends a signed target-minus-current error only outside the inclusive acceptance bounds.
/// Read failures remain diagnostic text so the original timeout and cleanup are preserved.
pub(super) fn describe_deviation(
    output: &mut String,
    field: &str,
    target: f64,
    current: Result<f64, InitialisationError>,
    tolerance: f64,
    unit: &str,
) {
    match current {
        Ok(current) if current < target - tolerance || current > target + tolerance => {
            output.push_str(&format!(
                "\n  {field}: target={target:.4} {unit}, current={current:.4} {unit}, difference(target-current)={:+.4} {unit}, tolerance=+/-{tolerance:.4} {unit}",
                target - current,
            ));
        }
        Ok(_) => {}
        Err(error) => output.push_str(&format!("\n  {field}: readback unavailable: {error}")),
    }
}

/// Moves a control setpoint toward its goal by at most the allowed elapsed-time step.
pub(super) fn ramp_towards(value: f64, target: f64, maximum_change: f64) -> f64 {
    value + (target - value).clamp(-maximum_change, maximum_change)
}

/// Ensures a native LVAR exists and is readable before any setup writes.
pub(super) fn validate_interface(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
) -> Result<(), InitialisationError> {
    if !simulator
        .local_variable_exists(variable)
        .map_err(InitialisationError::Readback)?
    {
        return Err(InitialisationError::MissingInterface { variable });
    }
    simulator
        .validate_read(variable, None)
        .map_err(InitialisationError::Readback)
}

/// Reads a native numeric LVAR without applying an SDK unit conversion.
pub(super) fn read_native(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
) -> Result<f64, InitialisationError> {
    simulator
        .read(variable, None)
        .map_err(InitialisationError::Readback)
}

/// Defends aircraft-specific flight-condition boundaries against invalid targets.
pub(super) fn validate_flight_condition_target(
    field: &'static str,
    value: f64,
) -> Result<(), InitialisationError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(InitialisationError::InvalidFlightConditionTarget { field, value });
    }
    Ok(())
}

/// Validates and reads a finite SDK SimVar with an explicit unit.
pub(super) fn read_simvar(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
    unit: &'static str,
) -> Result<f64, InitialisationError> {
    simulator
        .validate_read(variable, Some(unit))
        .map_err(InitialisationError::Readback)?;
    let value = simulator
        .read(variable, Some(unit))
        .map_err(InitialisationError::Readback)?;
    if !value.is_finite() {
        return Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "must be finite",
        });
    }
    Ok(value)
}
