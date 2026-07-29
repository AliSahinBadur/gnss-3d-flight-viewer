import struct
import unittest

from gnss_3d_visualizer import (
    FOOTER,
    HEADER,
    HYIPacketDecoder,
    PACKET_LENGTH,
    packet_checksum,
    parse_hyi_packet,
    representative_trajectory_coordinates,
)


def make_packet(
    counter: int = 42,
    latitude: float = 40.962105,
    longitude: float = 29.129418,
    altitude: float = 1_523.5,
) -> bytes:
    packet = bytearray(PACKET_LENGTH)
    packet[:4] = HEADER
    packet[4] = 126
    packet[5] = counter
    struct.pack_into("<f", packet, 22, altitude)
    struct.pack_into("<f", packet, 26, latitude)
    struct.pack_into("<f", packet, 30, longitude)
    packet[-2:] = FOOTER
    packet[75] = packet_checksum(packet)
    return bytes(packet)


class HYIPacketTests(unittest.TestCase):
    def test_parses_expected_gnss_fields(self):
        counter, latitude, longitude, altitude = parse_hyi_packet(make_packet())

        self.assertEqual(counter, 42)
        self.assertAlmostEqual(latitude, 40.962105, places=4)
        self.assertAlmostEqual(longitude, 29.129418, places=4)
        self.assertAlmostEqual(altitude, 1_523.5, places=2)

    def test_decoder_handles_noise_and_fragmentation(self):
        decoder = HYIPacketDecoder()
        packet = make_packet()

        self.assertEqual(decoder.feed(b"noise" + packet[:17]), [])
        decoded = decoder.feed(packet[17:])

        self.assertEqual(decoded, [packet])

    def test_rejects_bad_checksum(self):
        packet = bytearray(make_packet())
        packet[20] ^= 0x01

        with self.assertRaisesRegex(ValueError, "checksum"):
            parse_hyi_packet(bytes(packet))


class RepresentativeTrajectoryTests(unittest.TestCase):
    def test_reference_path_has_launch_apogee_and_landing(self):
        east, north, up = representative_trajectory_coordinates()

        self.assertEqual(len(east), 121)
        self.assertEqual((east[0], north[0], up[0]), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(east[-1], 80.0)
        self.assertAlmostEqual(up[-1], 0.0, places=6)
        self.assertAlmostEqual(max(up), 120.0, places=6)


if __name__ == "__main__":
    unittest.main()
