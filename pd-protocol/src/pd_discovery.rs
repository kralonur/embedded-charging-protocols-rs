//! Cable inspection sequencing through caller-owned transport.
//! The caller owns power/safety gates, one transport/FIFO, storage and results.
//! Transport errors/cancellation never authorize cleanup, retries or escalation.
use crate::{
    pd_cable,
    pd_cable_modes::{Discovery, Extended, Modes, Status, Sync},
    pd_rx::{self, Frame, Sop},
    pd_svids::Store,
};
use core::future::Future;
use usbpd_traits::{CableReport, CableStatus};

// PD3 Cable Plug Soft_Reset: Control type 13, revision 2, MessageID 0 (Tables 6.2, 6.4).
const CABLE_SOFT_RESET: [u8; 2] = [0x8d, 0];
// Accept Control Message type (Table 6.4).
const ACCEPT: u16 = 3;
// Mask for Control type, zero objects/Extended and MessageID 0; ignore revision
// and reserved/data-role bits for a Cable Plug Accept (Tables 6.2, 6.4).
const RESET_ACCEPT_MASK: u16 = 0xfe1f;
// Cable Plug bit in a SOP'/SOP'' Message Header (Table 6.3).
const CABLE_PLUG: u16 = 1 << 8;
// Message Type field and its combination with Extended; object count and
// MessageID are checked separately (Table 6.2).
const MESSAGE_TYPE_MASK: u16 = 0x1f;
const MESSAGE_KIND_MASK: u16 = 0x801f;
// ID Header Modal Operation Supported bit, allowing SVID discovery (Table 6.35).
const MODAL_OPERATION_SUPPORTED: u32 = 1 << 26;
// Vendor_Defined Data Message type (Table 6.5).
const VENDOR_DEFINED: u16 = 15;
// VDM request template: one Data Object, type 15; revision and MessageID supplied (Table 6.2).
const VDM_REQUEST_HEADER: u16 = 0x100f;
// Standard Structured VDM discovery commands (Table 6.34).
const DISCOVER_SVIDS: u8 = 2;
const DISCOVER_MODES: u8 = 3;
// Discover Identity request VDMs: USB-IF SVID, Structured, SVDM 2.1 or 1.0,
// Command Type Initiator and Command Discover Identity (Table 6.33).
const IDENTITY_PD3: u32 = 0xff00_a801;
const IDENTITY_PD2: u32 = 0xff00_8001;
// Not_Supported Control Message type and mask excluding MessageID/revision (Tables 6.2, 6.4).
const NOT_SUPPORTED: u16 = 16;
const CONTROL_KIND_MASK: u16 = 0x701f;
// Extended Message bit in the Message Header (Table 6.2).
const EXTENDED: u16 = 1 << 15;
// Extended Status and Manufacturer_Info Message types (Table 6.47).
const STATUS_MESSAGE: u8 = 2;
const MANUFACTURER_INFO_MESSAGE: u8 = 7;
// Supported complete Manufacturer_Info block: VID/PID plus at least a terminator,
// bounded by the single-Chunk payload ceiling (Tables 6.48, 6.57).
const MIN_MANUFACTURER_BYTES: usize = 5;
const MAX_CHUNK_BYTES: usize = 26;
// Cable Status requires the first two SDB bytes in this profile (Table 6.51).
const MIN_STATUS_BYTES: usize = 2;
// Get_Manufacturer_Info: PD3 Extended, one object, type 6 (Tables 6.2, 6.47).
const GET_MANUFACTURER_HEADER: u16 = 0x9086;
// Chunk 0, Data Size 2; Manufacturer Info Target/Ref 0 (Tables 6.48, 6.56).
const GET_MANUFACTURER_BODY: [u8; 4] = [2, 0x80, 0, 0];
// Get_Status Control Message at PD3, ID 0 (Tables 6.2, 6.4).
const GET_STATUS_HEADER: u16 = 0x0092;
// The second plug's first request after reset Accept: Get_Status, PD3, ID 1.
const SECOND_GET_STATUS: [u8; 2] = [0x92, 2];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Reset,
    Identity,
    SecondReset,
    SecondStatus,
}
#[derive(Clone, Copy)]
pub enum Fault {
    Peer,
    Request,
}

/// A complete CRC-checked packet, with its address preserved by the transport.
pub trait Packet {
    fn frame(&self) -> Frame<'_>;
}

/// Exclusive cable transport after a caller-approved fixed5V Ready handoff.
/// `send` completes matching GoodCRC + TX success; `wait` returns CRC-checked
/// packets and enforces the supplied deadline, rejecting late packets.
pub trait Transport {
    /// Caller-selected receive deadline in milliseconds.
    const RESPONSE_TIMEOUT_MS: u64;
    /// Caller-selected bound on repeated SVID pages without progress, in milliseconds.
    const SVID_NO_PROGRESS_TIMEOUT_MS: u64;
    type Error;
    type Packet: Packet;
    fn fault(fault: Fault) -> Self::Error;
    fn is_tx_timeout(error: &Self::Error) -> bool;
    fn now_ms(&self) -> u64;
    /// Transport capability AND caller permission for scoped SOP'' synchronization.
    fn supports_second(&self) -> bool;
    fn send(&mut self, bytes: &[u8], sop: Sop) -> impl Future<Output = Result<(), Self::Error>>;
    fn wait(
        &mut self,
        timeout_ms: u64,
    ) -> impl Future<Output = Result<Option<Self::Packet>, Self::Error>>;
    fn discovery(&mut self) -> &mut Discovery;
    fn store(&self) -> Option<&dyn Store>;
    fn receive_id(&self) -> u8;
    fn set_receive_id(&mut self, id: u8);
    fn set_phase(&mut self, phase: Phase);
    fn phase(&self) -> Phase;
    fn allow_extended(&mut self, enabled: bool);
    fn set_report(&mut self, report: Option<CableReport>);
    /// Fresh voltage/safety check after possibly long primary enumeration.
    fn recheck_power(&mut self) -> impl Future<Output = Result<(), Self::Error>>;
    /// Select second-plug receive only, without flush/reset/power changes.
    fn select_second(&mut self) -> impl Future<Output = Result<(), Self::Error>>;
}

/// Reset the Cable Plug's Protocol Layer before talking to it (§6.3.13, §7.1.1:
/// a SOP' Soft_Reset resets that plug only). A plug kept powered by the Source's
/// VCONN keeps the last MessageID it stored, also from the Source's own SOP'
/// traffic, and drops a repeat after GoodCRC (§7.32.3), so a fresh MessageID 0
/// request can go unanswered. Soft_Reset is exempt from that check (§7.32.3) and
/// carries our highest revision; the Accept carries the plug's (§6.1.3.2).
/// `Ok(Err(..))` reports why no identity request may follow.
async fn synchronize<T: Transport>(t: &mut T) -> Result<Result<u8, Sync>, T::Error> {
    t.set_phase(Phase::Reset);
    let sent = match t.send(&CABLE_SOFT_RESET, Sop::Cable).await {
        Ok(()) => true,
        Err(error) if T::is_tx_timeout(&error) => false,
        Err(error) => return Err(error),
    };
    let packet = if sent {
        t.wait(T::RESPONSE_TIMEOUT_MS).await?
    } else {
        None
    };
    t.set_phase(Phase::Identity);
    let Some(packet) = packet else {
        return Ok(Err(if sent {
            Sync::NoAccept
        } else {
            Sync::NoGoodCrc
        }));
    };
    let frame = packet.frame();
    // Accept: Control, MessageID 0, from a Cable Plug, on SOP'.
    if frame.sop() != Sop::Cable
        || frame.header() & RESET_ACCEPT_MASK != ACCEPT
        || frame.header() & CABLE_PLUG == 0
    {
        return Err(T::fault(Fault::Peer));
    }
    Ok(Ok(
        pd_rx::revision(frame.header()).map_err(|_| T::fault(Fault::Peer))?
    ))
}

async fn identity_response<T: Transport>(t: &mut T) -> Result<Option<T::Packet>, T::Error> {
    let Some(packet) = t.wait(T::RESPONSE_TIMEOUT_MS).await? else {
        return Ok(None);
    };
    if packet.frame().sop() == Sop::Cable && packet.frame().header() & CABLE_PLUG != 0 {
        Ok(Some(packet))
    } else {
        Err(T::fault(Fault::Peer))
    }
}

async fn query<T: Transport>(
    t: &mut T,
    svid: u16,
    command: u8,
    id: u8,
    revision: u8,
    version: u32,
) -> Result<Option<([u32; pd_cable::MAX_DATA_OBJECTS], u8)>, T::Error> {
    let header = VDM_REQUEST_HEADER | ((revision as u16) << 6) | ((id as u16) << 9);
    let vdm = ((svid as u32) << 16) | pd_cable::STRUCTURED_VDM | version | command as u32;
    let mut bytes = [0u8; 6];
    bytes[..2].copy_from_slice(&header.to_le_bytes());
    bytes[2..].copy_from_slice(&vdm.to_le_bytes());
    t.send(&bytes, Sop::Cable).await?;
    let Some(packet) = identity_response(t).await? else {
        return Ok(None);
    };
    let frame = packet.frame();
    if frame.header() & MESSAGE_KIND_MASK != VENDOR_DEFINED
        || pd_rx::revision(frame.header()).map_err(|_| T::fault(Fault::Peer))? != revision
    {
        return Err(T::fault(Fault::Peer));
    }
    let mut objects = [0u32; pd_cable::MAX_DATA_OBJECTS];
    let count = frame.data().len() / 4;
    for (slot, bytes) in objects.iter_mut().zip(frame.data().as_chunks::<4>().0) {
        *slot = u32::from_le_bytes(*bytes);
    }
    if crate::pd_cable_modes::response(&objects[..count], svid, command).is_none() {
        return Err(T::fault(Fault::Peer));
    }
    let rx_id = ((frame.header() >> 9) & 7) as u8;
    if rx_id == t.receive_id() {
        t.set_receive_id((t.receive_id() + 1) & 7);
    } else {
        let previous = t.discovery();
        let same = command == DISCOVER_SVIDS
            && count == pd_cable::MAX_DATA_OBJECTS
            && previous.svid_count as usize == pd_cable::MAX_DATA_OBJECTS
            && objects == previous.svid_objects;
        if !same || rx_id != (t.receive_id() + 7) & 7 {
            return Err(T::fault(Fault::Peer));
        }
    }
    Ok(Some((objects, count as u8)))
}

async fn discover_svids<T: Transport>(t: &mut T, report: &CableReport) -> Result<(), T::Error> {
    if t.store().is_none() {
        return Err(T::fault(Fault::Request));
    }
    let version = pd_cable::svdm_version(report.objects[0]).map_err(|_| T::fault(Fault::Peer))?;
    let mut last_progress = t.now_ms();
    loop {
        let id = t.discovery().next_id;
        let reply = query(
            t,
            pd_cable::USB_IF_SVID,
            DISCOVER_SVIDS,
            id,
            report.revision,
            version,
        )
        .await?;
        t.discovery().next_id = (id + 1) & 7;
        let Some((objects, count)) = reply else {
            t.discovery().status = Status::Timeout;
            return Ok(());
        };
        let progress = t
            .store()
            .ok_or_else(|| T::fault(Fault::Request))?
            .response(&objects[..count as usize])
            .ok_or_else(|| T::fault(Fault::Peer))?;
        let discovery = t.discovery();
        discovery.status = progress.status;
        discovery.count = progress.count;
        discovery.svid_objects = objects;
        discovery.svid_count = count;
        if progress.status != Status::Continuing {
            break;
        }
        if !progress.repeated {
            last_progress = t.now_ms();
        } else if t.now_ms().saturating_sub(last_progress) >= T::SVID_NO_PROGRESS_TIMEOUT_MS {
            return Err(T::fault(Fault::Peer));
        }
    }
    Ok(())
}

async fn read_mode<T: Transport>(
    t: &mut T,
    report: &CableReport,
    svid: u16,
) -> Result<(), T::Error> {
    let version = pd_cable::svdm_version(report.objects[0]).map_err(|_| T::fault(Fault::Peer))?;
    let id = t.discovery().next_id;
    let reply = query(t, svid, DISCOVER_MODES, id, report.revision, version).await?;
    t.discovery().next_id = (id + 1) & 7;
    t.discovery().mode = match reply {
        Some((objects, count)) => Modes {
            svid,
            objects,
            count,
            status: crate::pd_cable_modes::response(
                &objects[..count as usize],
                svid,
                DISCOVER_MODES,
            )
            .ok_or_else(|| T::fault(Fault::Peer))?,
        },
        None => Modes {
            svid,
            status: Status::Timeout,
            ..Default::default()
        },
    };
    Ok(())
}

async fn extended_query<T: Transport>(
    t: &mut T,
    bytes: &[u8],
    message: u8,
) -> Result<Extended, T::Error> {
    let second = t.phase() == Phase::SecondStatus;
    let sop = if second {
        Sop::CableDoublePrime
    } else {
        Sop::Cable
    };
    t.send(bytes, sop).await?;
    let Some(packet) = t.wait(T::RESPONSE_TIMEOUT_MS).await? else {
        return Ok(Extended {
            status: Status::Timeout,
            ..Extended::EMPTY
        });
    };
    let frame = packet.frame();
    let rx_id = if second { 1 } else { t.receive_id() };
    if frame.sop() != sop
        || frame.header() & CABLE_PLUG == 0
        || (frame.header() >> 6) & 3 != u16::from(pd_cable::PD3_REVISION)
        || (frame.header() >> 9) & 7 != rx_id as u16
    {
        return Err(T::fault(Fault::Peer));
    }
    if !second {
        t.set_receive_id((t.receive_id() + 1) & 7);
    }
    if frame.header() & CONTROL_KIND_MASK == NOT_SUPPORTED && frame.header() & EXTENDED == 0 {
        return Ok(Extended {
            status: Status::NotSupported,
            header: frame.header(),
            ..Extended::EMPTY
        });
    }
    if frame.header() & MESSAGE_KIND_MASK != EXTENDED | message as u16 {
        return Err(T::fault(Fault::Peer));
    }
    let data = frame.extended_data().map_err(|_| T::fault(Fault::Peer))?;
    if message == MANUFACTURER_INFO_MESSAGE
        && !(MIN_MANUFACTURER_BYTES..=MAX_CHUNK_BYTES).contains(&data.len())
        || message == STATUS_MESSAGE && data.len() < MIN_STATUS_BYTES
    {
        return Err(T::fault(Fault::Peer));
    }
    let mut result = Extended {
        status: Status::Ack,
        header: frame.header(),
        count: data.len() as u8,
        ..Extended::EMPTY
    };
    result.data[..data.len()].copy_from_slice(data);
    Ok(result)
}

async fn extended_info<T: Transport>(t: &mut T, report: &CableReport) -> Result<(), T::Error> {
    let discovery = *t.discovery();
    if discovery.status == Status::Timeout {
        return Ok(());
    }
    let id = discovery.next_id;
    t.allow_extended(true);
    let header = GET_MANUFACTURER_HEADER | ((id as u16) << 9);
    let mut bytes = [0; 6];
    bytes[..2].copy_from_slice(&header.to_le_bytes());
    bytes[2..].copy_from_slice(&GET_MANUFACTURER_BODY);
    let manufacturer = extended_query(t, &bytes, MANUFACTURER_INFO_MESSAGE).await?;
    t.discovery().manufacturer = manufacturer;
    if manufacturer.status != Status::Timeout
        && (report.objects[1] >> 27) & 7 == pd_cable::ACTIVE_CABLE
    {
        let header = GET_STATUS_HEADER | ((((id + 1) & 7) as u16) << 9);
        let status = extended_query(t, &header.to_le_bytes(), STATUS_MESSAGE).await?;
        t.discovery().cable_status = status;
    }
    t.allow_extended(false);
    Ok(())
}

async fn second_status<T: Transport>(t: &mut T, report: &CableReport) -> Result<(), T::Error> {
    if pd_cable::second_controller(report) != Some(true) {
        return Ok(());
    }
    if !t.supports_second() || report.revision != pd_cable::PD3_REVISION {
        t.discovery().second_status = Extended {
            status: Status::NotSupported,
            ..Extended::EMPTY
        };
        return Ok(());
    }
    let discovery = *t.discovery();
    if discovery.status == Status::Timeout
        || discovery.manufacturer.status == Status::Timeout
        || discovery.cable_status.status == Status::Timeout
    {
        return Ok(());
    }
    t.recheck_power().await?;
    t.set_phase(Phase::SecondReset);
    t.select_second().await?;
    t.send(&CABLE_SOFT_RESET, Sop::CableDoublePrime).await?;
    let Some(packet) = t.wait(T::RESPONSE_TIMEOUT_MS).await? else {
        t.discovery().second_sync = Status::Timeout;
        return Err(T::fault(Fault::Peer));
    };
    let frame = packet.frame();
    if frame.sop() != Sop::CableDoublePrime
        || frame.header() & RESET_ACCEPT_MASK != ACCEPT
        || frame.header() & CABLE_PLUG == 0
        || pd_rx::revision(frame.header()).map_err(|_| T::fault(Fault::Peer))?
            != pd_cable::PD3_REVISION
    {
        return Err(T::fault(Fault::Peer));
    }
    t.discovery().second_sync = Status::Ack;
    t.set_phase(Phase::SecondStatus);
    t.allow_extended(true);
    let status = extended_query(t, &SECOND_GET_STATUS, STATUS_MESSAGE).await?;
    t.discovery().second_status = status;
    t.allow_extended(false);
    t.set_phase(Phase::Identity);
    Ok(())
}

/// Initialize caller-owned discovery storage before power/controller handoff.
/// Selected-mode reads retain the existing complete set until identity is checked.
pub fn begin<T: Transport>(
    t: &mut T,
    requested_mode: Option<u16>,
    previous_report: Option<CableReport>,
) -> Result<(), T::Error> {
    if let Some(svid) = requested_mode {
        if t.discovery().status != Status::Ack
            || previous_report.is_none()
            || !t.store().is_some_and(|store| store.contains(svid))
        {
            return Err(T::fault(Fault::Request));
        }
    } else {
        if let Some(store) = t.store() {
            store.clear();
        }
        *t.discovery() = Discovery::default();
    }
    Ok(())
}

/// Inspect identity, or reread identity before one explicitly selected mode.
/// Power-up and normal verified restoration belong to the caller, not this engine.
/// The sole initial SOP'' reset is synchronization, never error recovery.
pub async fn inspect<T: Transport>(
    t: &mut T,
    next_id: u8,
    last_id: Option<u8>,
    requested_mode: Option<u16>,
    previous_report: Option<CableReport>,
) -> Result<CableReport, T::Error> {
    // MessageID is a three-bit field, bits 11..9 of the Message Header (Table 6.2).
    if next_id > 7 || last_id.is_some_and(|id| id > 7) || t.phase() != Phase::Identity {
        return Err(T::fault(Fault::Request));
    }
    if let Some(svid) = requested_mode
        && (t.discovery().status != Status::Ack
            || previous_report.is_none()
            || !t.store().is_some_and(|store| store.contains(svid)))
    {
        return Err(T::fault(Fault::Request));
    }
    let mut report = CableReport {
        status: CableStatus::Timeout,
        objects: [0; pd_cable::MAX_DATA_OBJECTS],
        count: 0,
        revision: 0,
        next_tx_id: next_id,
        last_rx_id: last_id,
    };
    let accepted = match synchronize(t).await? {
        Ok(revision) => {
            t.discovery().sync = Sync::Accepted;
            Some(revision)
        }
        Err(failure) => {
            t.discovery().sync = failure;
            None
        }
    };
    // Discover Identity at the Accept's revision with the first MessageID after
    // the reset; a PD3 plug that does not answer gets one PD2 request (§6.1.3.2 notes).
    let mut id = 1u8;
    let mut packet = None;
    for (revision, vdm) in [
        (pd_cable::PD3_REVISION, IDENTITY_PD3),
        (pd_cable::PD2_REVISION, IDENTITY_PD2),
    ] {
        if packet.is_some() || accepted.is_none_or(|accepted| revision > accepted) {
            continue;
        }
        let header = VDM_REQUEST_HEADER | (u16::from(revision) << 6) | (u16::from(id) << 9);
        let mut bytes = [0u8; 6];
        bytes[..2].copy_from_slice(&header.to_le_bytes());
        bytes[2..].copy_from_slice(&vdm.to_le_bytes());
        id += 1;
        packet = match t.send(&bytes, Sop::Cable).await {
            Ok(()) => identity_response(t).await?,
            Err(error) if T::is_tx_timeout(&error) => None,
            Err(error) => return Err(error),
        };
    }
    t.discovery().next_id = id;
    if let Some(packet) = packet {
        let frame = packet.frame();
        if frame.header() & MESSAGE_TYPE_MASK != VENDOR_DEFINED
            || pd_rx::revision(frame.header()).is_err()
        {
            return Err(T::fault(Fault::Peer));
        }
        t.set_receive_id((((frame.header() >> 9) as u8 & 7) + 1) & 7);
        report.revision = pd_rx::revision(frame.header()).map_err(|_| T::fault(Fault::Peer))?;
        report.count = (frame.data().len() / 4) as u8;
        for (slot, bytes) in report
            .objects
            .iter_mut()
            .zip(frame.data().as_chunks::<4>().0)
        {
            *slot = u32::from_le_bytes(*bytes);
        }
        report.status =
            match pd_cable::decode(&report.objects[..report.count as usize], report.revision)
                .map_err(|_| T::fault(Fault::Peer))?
            {
                pd_cable::Response::Identity(_) => CableStatus::Identity,
                pd_cable::Response::Nak => CableStatus::Nak,
                pd_cable::Response::Busy => CableStatus::Busy,
            };
    }
    if let Some(svid) = requested_mode {
        let old = previous_report.ok_or_else(|| T::fault(Fault::Request))?;
        if report.status != CableStatus::Identity
            || report.objects != old.objects
            || report.count != old.count
            || report.revision != old.revision
        {
            if let Some(store) = t.store() {
                store.clear();
            }
            *t.discovery() = Discovery::EMPTY;
            t.set_report(None);
            return Err(T::fault(Fault::Peer));
        }
        t.discovery().next_id = id;
        read_mode(t, &report, svid).await?;
    } else if report.status == CableStatus::Identity
        && report.objects[1] & MODAL_OPERATION_SUPPORTED != 0
    {
        discover_svids(t, &report).await?;
    }
    t.set_report(Some(report));
    if requested_mode.is_none() && report.status == CableStatus::Identity {
        if report.revision == pd_cable::PD3_REVISION {
            extended_info(t, &report).await?;
        }
        second_status(t, &report).await?;
    }
    Ok(report)
}
