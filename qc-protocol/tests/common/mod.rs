//! A simulated caller and charger timing shared by the integration tests.

/// The simulated caller measures and polls every this many ms.
pub const SCAN_MS: u64 = 100;
/// D− while the charger shorts it to D+, held at 0.6 V (CHY100 Table 1).
pub const SHORTED_MV: u16 = 600;
/// D− pulled low by the charger.
pub const LOW_MV: u16 = 0;
/// A simulated Quick Charge charger opens the short and pulls D− low this
/// long after D+ reached 0.6 V: inside the 1 to 1.5 s D+ High Glitch Filter
/// Time (CHY100 TGLITCH(BC)DONE).
pub const RELEASE_MS: u64 = 1_250;

/// D− from a Quick Charge charger, `ms` after D+ reached 0.6 V.
pub fn quick_charge_dm_mv(ms: u64) -> u16 {
    if ms < RELEASE_MS { SHORTED_MV } else { LOW_MV }
}
