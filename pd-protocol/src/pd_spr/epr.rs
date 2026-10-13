//! EPR advertisements and requests share the caller's voltage and power limits.
use super::*;
use source_capabilities::{Augmented, PowerDataObject};

/// EPR AVS range starts at 15 V (section 10.2.3.2).
pub const EPR_AVS_MIN_MV: u32 = 15_000;
/// EPR fixed voltages defined by section 6.5.15.1; never invent a 30 V fixed PDO.
const EPR_FIXED_MV: [u32; 3] = [28_000, 36_000, 48_000];
/// EPR AVS APDO discriminator and voltage field width (Table 6.12).
const EPR_AVS_TYPE: u32 = 0xd000_0000;
/// Sink capability in every RDO, not in the Sink Fixed Supply PDO (section 6.4.2.7).
pub const EPR_RDO_CAPABLE: u32 = 1 << 22;
/// SPR prefix length in EPR Capabilities (section 6.5.15.1).
pub const SPR_OBJECTS: usize = MAX_OBJECTS;
/// PD3 Sink/UFP wire headers without MessageID (Tables 6.2, 6.5, 6.47).
pub const EPR_REQUEST_HEADER: u16 = 0x2089;
pub const EPR_MODE_HEADER: u16 = 0x108a;
pub const EPR_MODE_KIND: u16 = 0x100a;
/// EPR_KeepAlive Extended Control, Chunk 0, Data Size 2 (section 6.5.14.3).
pub const EPR_KEEP_ALIVE_HEADER: u16 = 0x9090;
pub const EPR_KEEP_ALIVE_BODY: [u8; 4] = [2, 0x80, 3, 0];
/// EPR_Mode actions, carried in byte 3 of the Data Object (Table 6.49).
pub const EPR_ENTER: u8 = 1;
pub const EPR_ENTER_ACK: u8 = 2;
pub const EPR_ENTER_SUCCEEDED: u8 = 3;
pub const EPR_ENTER_FAILED: u8 = 4;
pub const EPR_EXIT: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EprState {
    Spr,
    Entering,
    Active,
    Exiting,
    /// Source's EPR_Mode Enter Failed data byte (section 6.4.10).
    Failed(u8),
    Timeout,
    Deferred,
}

/// SKEDB minimum, operational and maximum PDP, in whole watts. The maximum
/// covers every selectable EPR voltage at the 5 A cable ceiling. Operational
/// describes the trigger's normal minimum-current function, not its maximum.
pub fn epr_pdps<L: Limits>() -> Option<(u8, u8, u8)> {
    if !limits_valid::<L>()
        || !L::EPR
        || !(EPR_FIXED_MV[0]..=EPR_FIXED_MV[2]).contains(&L::MAX_REQUEST_MV)
    {
        return None;
    }
    let watts = |mv: u32, ma: u32| (mv * ma).div_ceil(MV_MA_PER_WATT).min(u8::MAX as u32) as u8;
    Some((
        watts(VSAFE5V_MV, L::REQUEST_MA),
        watts(L::MAX_REQUEST_MV, L::REQUEST_MA),
        watts(L::MAX_REQUEST_MV, fixed_sink_ma::<L>()),
    ))
}

/// EPR Sink_Capabilities: identical SPR prefix, zero padded to seven, then
/// fixed EPR targets and a 15 V..caller ceiling AVS APDO (section 6.5.15.1).
pub fn epr_sink_capabilities<L: Limits>() -> Option<([u32; EPR_OBJECTS], usize)> {
    let (_, operational, _) = epr_pdps::<L>()?;
    let (spr, count) = sink_capabilities::<L>(true)?;
    let mut objects = [0; EPR_OBJECTS];
    objects[..count].copy_from_slice(&spr[..count]);
    let mut count = MAX_OBJECTS;
    for mv in EPR_FIXED_MV {
        if mv <= L::MAX_REQUEST_MV && L::FIXED_TARGETS.contains(&(mv as u16)) {
            objects[count] = ((mv / FIXED_VOLTAGE_UNIT_MV) << 10)
                | (fixed_sink_ma::<L>() / FIXED_CURRENT_UNIT_MA);
            count += 1;
        }
    }
    // EPR AVS Sink PDP is the power needed for full function, not a current field.
    objects[count] = EPR_AVS_TYPE
        | (L::MAX_REQUEST_MV / PPS_PDO_VOLTAGE_UNIT_MV) << 17
        | (EPR_AVS_MIN_MV / PPS_PDO_VOLTAGE_UNIT_MV) << 8
        | u32::from(operational);
    Some((objects, count + 1))
}

/// The EPR AVS current allowed at a voltage, rounded down to 50 mA. Both the
/// Source's PDP and Sink Maximum PDP bind, as does the cable's 5 A ceiling.
pub fn epr_avs_target_ma<L: Limits>(raw: u32, mv: u32) -> Option<u16> {
    let pdo = source_capabilities::EprAdjustableVoltageSupply(raw);
    let min = u32::from(pdo.raw_min_voltage()) * PPS_PDO_VOLTAGE_UNIT_MV;
    let max = u32::from(pdo.raw_max_voltage()) * PPS_PDO_VOLTAGE_UNIT_MV;
    let (_, _, sink_pdp) = epr_pdps::<L>()?;
    if raw & PPS_APDO_TYPE_MASK != EPR_AVS_TYPE
        || min != EPR_AVS_MIN_MV
        || min > max
        || !EPR_FIXED_MV.contains(&max)
        || !(min..=max.min(L::MAX_REQUEST_MV)).contains(&mv)
        || !mv.is_multiple_of(AVS_VOLTAGE_STEP_MV)
    {
        return None;
    }
    let pdp = u32::from(pdo.raw_pd_power()).min(u32::from(sink_pdp));
    let ma = (pdp * MV_MA_PER_WATT / mv).min(fixed_sink_ma::<L>()) / PPS_CURRENT_UNIT_MA
        * PPS_CURRENT_UNIT_MA;
    (ma >= L::REQUEST_MA).then_some(ma as u16)
}

/// Construct EPR_Request from the latest EPR capabilities, including the exact
/// PDO copy. `Supply::Avs` positions 8..11 are EPR AVS; lower ones remain SPR AVS.
pub fn request_epr<L: Limits>(
    caps: &source_capabilities::SourceCapabilities,
    target: Target,
) -> Result<request::PowerSource, Error> {
    if epr_pdps::<L>().is_none()
        || caps.pdos().len() > EPR_OBJECTS
        || caps.has_epr_pdo_in_spr_positions()
        || u32::from(target.mv) > L::MAX_REQUEST_MV
        || !matches!(caps.pdos().first(), Some(PowerDataObject::FixedSupply(pdo))
            if u32::from(pdo.raw_voltage()) * FIXED_VOLTAGE_UNIT_MV == VSAFE5V_MV)
    {
        return Err(Error::UnsafeCapabilities);
    }
    let spr = source_capabilities::SourceCapabilities::new_with_pdos(
        caps.pdos().iter().take(MAX_OBJECTS).copied().collect(),
    );
    let (rdo, pdo) = match target.supply {
        Supply::Fixed if u32::from(target.mv) > SPR_FIXED_MAX_MV => {
            let Some((index, PowerDataObject::FixedSupply(pdo))) =
                caps.pdos().iter().enumerate().find(|(index, pdo)| {
                    *index >= MAX_OBJECTS
                        && matches!(pdo, PowerDataObject::FixedSupply(pdo)
                    if u32::from(pdo.raw_voltage()) * FIXED_VOLTAGE_UNIT_MV == u32::from(target.mv))
                })
            else {
                return Err(Error::UnsafeCapabilities);
            };
            if !EPR_FIXED_MV.contains(&u32::from(target.mv))
                || !L::FIXED_TARGETS.contains(&target.mv)
                || u32::from(pdo.raw_max_current()) * FIXED_CURRENT_UNIT_MA < L::REQUEST_MA
                || u32::from(pdo.raw_max_current()) * FIXED_CURRENT_UNIT_MA > FIXED_SINK_MAX_MA
            {
                return Err(Error::UnsafeCapabilities);
            }
            let current =
                (fixed_request_ma::<L>(u32::from(pdo.raw_max_current()) * FIXED_CURRENT_UNIT_MA)
                    / FIXED_CURRENT_UNIT_MA) as u16;
            let rdo = request::FixedVariableSupply(0)
                .with_object_position(index as u8 + 1)
                .with_raw_operating_current(current)
                .with_raw_max_operating_current(current)
                .with_usb_communications_capable(L::USB_COMMUNICATIONS_CAPABLE)
                .with_no_usb_suspend(L::NO_USB_SUSPEND)
                .with_epr_mode_capable(true)
                .0;
            (rdo, PowerDataObject::FixedSupply(*pdo))
        }
        Supply::Avs(object) if usize::from(object) > MAX_OBJECTS => {
            let Some(PowerDataObject::Augmented(Augmented::Epr(pdo))) =
                caps.pdos().get(usize::from(object) - 1)
            else {
                return Err(Error::UnsafeCapabilities);
            };
            let ma = epr_avs_target_ma::<L>(pdo.0, u32::from(target.mv))
                .ok_or(Error::UnsafeCapabilities)?;
            if ma != target.ma {
                return Err(Error::UnsafeCapabilities);
            }
            let rdo = request::Avs(0)
                .with_object_position(object)
                .with_raw_output_voltage(target.mv / AVS_RDO_VOLTAGE_UNIT_MV as u16)
                .with_raw_operating_current(ma / PPS_CURRENT_UNIT_MA as u16)
                .with_usb_communications_capable(L::USB_COMMUNICATIONS_CAPABLE)
                .with_no_usb_suspend(L::NO_USB_SUSPEND)
                .with_epr_mode_capable(true)
                .0;
            (rdo, PowerDataObject::Augmented(Augmented::Epr(*pdo)))
        }
        _ => {
            let request = target.request::<L>(&spr)?;
            let rdo = request_raw(&request).ok_or(Error::UnsafeCapabilities)?;
            let pdo = *spr
                .pdos()
                .get(request.object_position() as usize - 1)
                .ok_or(Error::UnsafeCapabilities)?;
            (rdo, pdo)
        }
    };
    Ok(request::PowerSource::EprRequest(
        request::EprRequestDataObject { rdo, pdo },
    ))
}

/// Independent wire review: regenerate the request from semantic values and
/// compare both the RDO and PDO copy, never trusting an externally supplied RDO.
pub fn review_epr_request<L: Limits>(
    objects: [u32; EPR_OBJECTS],
    bytes: &[u8],
    target: Target,
) -> bool {
    if bytes.len() != 10 {
        return false;
    }
    let header = u16::from_le_bytes([bytes[0], bytes[1]]);
    // Two objects, EPR_Request (type 9), PD3, Sink/UFP (Tables 6.2, 6.5).
    if header & HEADER_WITHOUT_ID_MASK != EPR_REQUEST_HEADER {
        return false;
    }
    let caps = source_capabilities::SourceCapabilities::new_with_pdos(
        objects
            .into_iter()
            .map(source_capabilities::parse_raw_pdo)
            .collect(),
    );
    matches!(request_epr::<L>(&caps, target), Ok(request::PowerSource::EprRequest(request))
        if bytes[2..6] == request.rdo.to_le_bytes() && bytes[6..10] == request.pdo.to_raw().to_le_bytes())
}
