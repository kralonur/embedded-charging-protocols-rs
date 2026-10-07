//! Read-only Structured VDM discovery storage and response validation.
//! No Enter Mode, authentication or vendor-specific commands are issued.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Status {
    #[default]
    NotQueried,
    Ack,
    Nak,
    Busy,
    Timeout,
    Continuing,
    NotSupported,
}

/// How far the SOP' Soft_Reset before Discover Identity got (§6.1.3.2,
/// §6.3.13). It tells "no cable answering at all" apart from "cable
/// acknowledged but sent no reply".
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Sync {
    /// Not sent: no inspection, or the transport could not power the cable.
    #[default]
    NotSent,
    /// No GoodCRC after all retries: nothing on SOP' received the Soft_Reset.
    NoGoodCrc,
    /// GoodCRC but no Accept in time; identity was not requested.
    NoAccept,
    /// Accepted; the Cable Plug's Protocol Layer and MessageIDs are reset.
    Accepted,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Modes {
    pub svid: u16,
    pub status: Status,
    /// Entire response, including its header; unknown modes remain lossless.
    pub objects: [u32; 7],
    pub count: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Extended {
    pub status: Status,
    pub header: u16,
    pub data: [u8; 26],
    pub count: u8,
}
impl Extended {
    pub const EMPTY: Self = Self {
        status: Status::NotQueried,
        header: 0,
        data: [0; 26],
        count: 0,
    };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Discovery {
    pub status: Status,
    /// Latest complete Discover SVIDs response, not a complete wire transcript.
    pub svid_objects: [u32; 7],
    pub svid_count: u8,
    /// Only the selected SVID's reply is cached; all SVIDs live in pd_svids.
    pub mode: Modes,
    pub next_id: u8,
    pub count: u16,
    pub manufacturer: Extended,
    pub cable_status: Extended,
    /// SOP'' protocol synchronization and its allowed Get_Status response.
    pub second_sync: Status,
    pub second_status: Extended,
    /// SOP' synchronization before identity.
    pub sync: Sync,
}

/// Validate the requested SVID/command, structured header and ACK/NAK/BUSY.
pub fn response(objects: &[u32], svid: u16, command: u8) -> Option<Status> {
    let &header = objects.first()?;
    if objects.len() > 7
        || header >> 16 != svid as u32
        || header & 0x8000 == 0
        || header & 0x1f != command as u32
        || crate::pd_cable::svdm_version(header).is_err()
    {
        return None;
    }
    match (header >> 6) & 3 {
        1 if objects.len() >= 2 => Some(Status::Ack),
        2 if objects.len() == 1 => Some(Status::Nak),
        3 if objects.len() == 1 => Some(Status::Busy),
        _ => None,
    }
}

impl Default for Discovery {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Discovery {
    pub const EMPTY: Self = Self {
        status: Status::NotQueried,
        svid_objects: [0; 7],
        svid_count: 0,
        mode: Modes {
            svid: 0,
            status: Status::NotQueried,
            objects: [0; 7],
            count: 0,
        },
        next_id: 1,
        count: 0,
        manufacturer: Extended::EMPTY,
        cable_status: Extended::EMPTY,
        second_sync: Status::NotQueried,
        second_status: Extended::EMPTY,
        sync: Sync::NotSent,
    };
}
