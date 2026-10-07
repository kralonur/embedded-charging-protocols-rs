//! Lossless identity inspection. Numeric values are field encodings, not
//! measured properties; unknown layouts are exposed only as raw objects.
//! Modern layouts: USB PD R3.2 V1.2 (2026-08-19), Tables 6.34, 6.42–6.44.
use crate::pd_cable::{CableReport, Layout, Response};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Field {
    pub name: &'static str,
    pub value: u32,
}

/// All received words followed by all defined fields of supported layouts.
/// Returns None past the end, or for invalid identity responses.
pub fn field(report: &CableReport, index: usize) -> Option<Field> {
    if report.count > 7 || report.status != crate::pd_cable::CableStatus::Identity {
        return None;
    }
    let Response::Identity(identity) =
        crate::pd_cable::decode(&report.objects[..report.count as usize], report.revision).ok()?
    else {
        return None;
    };
    let raw_names = [
        "VDM RAW",
        "ID RAW",
        "XID RAW",
        "PRODUCT RAW",
        "CABLE1 RAW",
        "CABLE2 RAW",
        "EXTRA RAW",
    ];
    if index < report.count as usize {
        return Some(Field {
            name: raw_names[index],
            value: report.objects[index],
        });
    }
    let index = index - report.count as usize;
    // (caption, object, least significant bit, width).
    let common = [
        ("PD REV CODE", 0, 0, 0),
        ("CURRENT ASSUMED", 0, 1, 0),
        ("LAYOUT CODE", 0, 2, 0),
        ("SVDM MAJOR", 0, 13, 2),
        ("SVDM MINOR", 0, 11, 2),
        ("USB HOST", 1, 31, 1),
        ("USB DEVICE", 1, 30, 1),
        ("PRODUCT TYPE", 1, 27, 3),
        ("MODAL", 1, 26, 1),
        ("ID CONNECTOR RAW", 1, 21, 2),
        ("VID", 1, 0, 16),
        ("XID", 2, 0, 32),
        ("PID", 3, 16, 16),
        ("BCD DEVICE", 3, 0, 16),
        ("HW VERSION", 4, 28, 4),
        ("FW VERSION", 4, 24, 4),
    ];
    let modern = [
        ("VDO VERSION", 4, 21, 3),
        ("PLUG CODE", 4, 18, 2),
        ("EPR BIT", 4, 17, 1),
        ("LATENCY CODE", 4, 13, 4),
        ("TERMINATION", 4, 11, 2),
        ("VOLTAGE CODE", 4, 9, 2),
        ("CURRENT CODE", 4, 5, 2),
        ("SPEED CODE", 4, 0, 3),
    ];
    let active1 = [
        ("SBU UNSUPPORTED", 4, 8, 1),
        ("SBU ACTIVE", 4, 7, 1),
        ("VBUS THROUGH", 4, 4, 1),
        ("SOP2 PRESENT", 4, 3, 1),
    ];
    let active2 = [
        ("MAX TEMP C", 5, 24, 8),
        ("SHUTDOWN TEMP C", 5, 16, 8),
        ("U3 CLD POWER CODE", 5, 12, 3),
        ("U3 VIA U3S", 5, 11, 1),
        ("OPTICAL", 5, 10, 1),
        ("RETIMER", 5, 9, 1),
        ("USB4 UNSUPPORTED", 5, 8, 1),
        ("USB2 HUB HOPS", 5, 6, 2),
        ("USB2 UNSUPPORTED", 5, 5, 1),
        ("USB3 UNSUPPORTED", 5, 4, 1),
        ("TWO LANES", 5, 3, 1),
        ("OPTICALLY ISOLATED", 5, 2, 1),
        ("ASYMMETRIC USB4", 5, 1, 1),
        ("GEN2 OR HIGHER", 5, 0, 1),
    ];
    let passive = identity.kind == crate::pd_cable::CableKind::Passive;
    let legacy = [
        ("PLUG CODE", 4, 18, 2),
        (
            if passive {
                "PLUG RECEPTACLE"
            } else {
                "B17 RAW"
            },
            4,
            17,
            1,
        ),
        ("LATENCY CODE", 4, 13, 4),
        ("TERMINATION", 4, 11, 2),
        ("SSTX1 RAW", 4, 10, 1),
        ("SSTX2 RAW", 4, 9, 1),
        ("SSRX1 RAW", 4, 8, 1),
        ("SSRX2 RAW", 4, 7, 1),
        ("CURRENT CODE", 4, 5, 2),
        ("VBUS THROUGH", 4, 4, 1),
        (if passive { "B3 RAW" } else { "SOP2 PRESENT" }, 4, 3, 1),
        ("SPEED CODE", 4, 0, 3),
    ];
    let modern_layout = matches!(identity.layout, Layout::PassiveV10 | Layout::ActiveV13);
    let descriptor = if index < common.len() {
        common.get(index)
    } else {
        let index = index - common.len();
        if identity.layout == Layout::Pd2 {
            legacy.get(index)
        } else if !modern_layout {
            return None;
        } else if index < modern.len() {
            modern.get(index)
        } else {
            let index = index - modern.len();
            if identity.layout != Layout::ActiveV13 {
                return None;
            }
            if index < active1.len() {
                active1.get(index)
            } else {
                // VDO2 is present only in the modern active layout, not PD2.
                if report.count < 6 {
                    return None;
                }
                active2.get(index - active1.len())
            }
        }
    }?;
    let &(name, object, shift, width) = descriptor;
    let value = if width == 0 {
        match shift {
            1 => identity.current_assumed as u32,
            2 => identity.layout as u32,
            _ => report.revision as u32,
        }
    } else if width == 32 {
        report.objects[object]
    } else {
        (report.objects[object] >> shift) & ((1u32 << width) - 1)
    };
    Some(Field { name, value })
}

pub fn field_count(report: &CableReport) -> usize {
    (0..64)
        .take_while(|&index| field(report, index).is_some())
        .count()
}
