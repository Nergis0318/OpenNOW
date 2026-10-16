//! DualSense (DS5) input report synthesis.
//!
//! The layout is the Sony USB input report (report id `0x01`, 64 bytes) exactly as decoded by
//! `SDL_hidapi_ps5.c` `PS5StatePacket_t` and `hid-playstation.c` `dualsense_input_report`; the two
//! agree on every offset used here. The DS4 and DS5 reports are the same length, so the wire
//! envelope needs no size change — only the layout and the interface byte differ.

use crate::SonySnapshot;
use crate::ds4::{Ds4ReportState, centered_axis, encode_contact};

pub const DS5_REPORT_ID: u8 = 0x01;
pub const DS5_REPORT_BYTES: usize = 64;

/// Server-side HID interface byte that selects the synthesized device class.
///
/// shortcut: `DS5_INTERFACE` is unproved — the DS4 target uses `DS4_INTERFACE` (4) and nothing in
/// captured traffic names a DS5 value. Confirm from a capture of the official client driving a
/// DualSense before this ships; if the pad stops being recognised at all, set
/// `SonyCapability::cross_synthesize_ds5_to_ds4` back to true to return to the proved DS4 target.
pub const DS5_INTERFACE: u8 = 5;

const DS5_BUTTON_TOUCHPAD: u8 = 0x02;

impl Ds4ReportState {
    /// Builds the DualSense report for `snapshot`, reusing the DS4 contact tracking so a source
    /// keeps its touch identity when it moves between the two targets.
    pub fn build_ds5(&mut self, snapshot: &SonySnapshot) -> [u8; DS5_REPORT_BYTES] {
        let next = self.advance(snapshot);
        let (face, shoulders) = Self::shared_button_bytes(snapshot);

        let mut report = [0_u8; DS5_REPORT_BYTES];
        report[0] = DS5_REPORT_ID;
        report[1] = centered_axis(snapshot.left_stick_x);
        report[2] = centered_axis(snapshot.left_stick_y);
        report[3] = centered_axis(snapshot.right_stick_x);
        report[4] = centered_axis(snapshot.right_stick_y);
        report[5] = snapshot.left_trigger;
        report[6] = snapshot.right_trigger;
        report[7] = (snapshot.observed_at_us.wrapping_mul(3).wrapping_shr(4)) as u8;
        report[8] = face;
        report[9] = shoulders;
        report[10] = if snapshot.touchpad_click {
            DS5_BUTTON_TOUCHPAD
        } else {
            0
        };
        report[33..37].copy_from_slice(&encode_contact(self.tracking_id(0), next[0]));
        report[37..41].copy_from_slice(&encode_contact(self.tracking_id(1), next[1]));
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SonyContact;

    fn snapshot() -> SonySnapshot {
        SonySnapshot::neutral(0, 1, 4_000)
    }

    #[test]
    fn dualsense_report_places_every_field_at_the_decoded_offset() {
        let mut state = Ds4ReportState::default();
        let mut snapshot = snapshot();
        snapshot.left_stick_x = 32767;
        snapshot.right_stick_y = -32768;
        snapshot.left_trigger = 0x40;
        snapshot.right_trigger = 0x80;
        snapshot.touchpad_click = true;
        snapshot.contacts = [
            SonyContact {
                active: true,
                x: -32768,
                y: -32768,
            },
            SonyContact::default(),
        ];

        let report = state.build_ds5(&snapshot);

        assert_eq!(report[0], DS5_REPORT_ID);
        assert_eq!(report[1], 0xff);
        assert_eq!(report[2], 0x80);
        assert_eq!(report[3], 0x80);
        assert_eq!(report[4], 0x00);
        // Triggers sit at 5/6 on the DualSense, not 8/9 as on the DualShock.
        assert_eq!(report[5], 0x40);
        assert_eq!(report[6], 0x80);
        assert_eq!(report[10], DS5_BUTTON_TOUCHPAD);
        // Contact 0 occupies 33..37 and carries the inactive flag in its high bit.
        assert_eq!(report[33], 0x01);
        assert_eq!(
            report[37], 0x80,
            "contact 1 is inactive and still untracked"
        );
    }

    #[test]
    fn dualsense_report_carries_no_touchpad_click_outside_the_button_byte() {
        let mut state = Ds4ReportState::default();
        let mut snapshot = snapshot();
        snapshot.touchpad_click = true;
        let report = state.build_ds5(&snapshot);
        assert_eq!(report[10], DS5_BUTTON_TOUCHPAD);
        assert_eq!(report[33], 0x80, "a neutral contact stays inactive");
    }
}
