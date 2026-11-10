from __future__ import annotations

import unittest

from swarmlink import JSONLinesTelemetryDecoder, NodeTelemetry, Position, VehicleKind, decode_packet, encode_compact_packet, encode_packet


def telemetry(node_id: str = "esp32-rover") -> NodeTelemetry:
    return NodeTelemetry(node_id, 1_735_689_600.0, Position(1.5, -2.0), vehicle_kind=VehicleKind.ROVER)


class JSONLinesBridgeTests(unittest.TestCase):
    def test_decodes_packet_split_across_serial_reads(self) -> None:
        packet = encode_packet(telemetry()) + b"\n"
        decoder = JSONLinesTelemetryDecoder()
        self.assertEqual(decoder.feed(packet[:12]), [])
        self.assertEqual(decoder.feed(packet[12:]), [telemetry()])
        self.assertEqual(decoder.accepted_packets, 1)

    def test_decodes_multiple_messages_and_ignores_bad_line(self) -> None:
        first, second = telemetry("one"), telemetry("two")
        decoder = JSONLinesTelemetryDecoder()
        received = decoder.feed(encode_packet(first) + b"\nnot-json\n" + encode_packet(second) + b"\r\n")
        self.assertEqual(received, [first, second])
        self.assertEqual(decoder.rejected_packets, 1)

    def test_ignores_incomplete_but_valid_json_packet(self) -> None:
        decoder = JSONLinesTelemetryDecoder()
        self.assertEqual(decoder.feed(b'{"type":"swarmlink.telemetry.compact.v1","n":"missing-fields"}\n'), [])
        self.assertEqual(decoder.rejected_packets, 1)

    def test_drops_unbounded_incomplete_frame(self) -> None:
        decoder = JSONLinesTelemetryDecoder(max_line_bytes=64)
        self.assertEqual(decoder.feed(b"x" * 65), [])
        self.assertEqual(decoder.rejected_packets, 1)

    def test_compact_radio_packet_round_trips_under_espnow_limit(self) -> None:
        original = NodeTelemetry(
            "esp32-drone-01", 1_735_689_600.125, Position(123.125, -42.75, 8.5),
            vehicle_kind=VehicleKind.DRONE,
        )
        packet = encode_compact_packet(original)
        self.assertLessEqual(len(packet), 250)
        self.assertEqual(decode_packet(packet), original)


if __name__ == "__main__":
    unittest.main()
