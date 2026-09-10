import struct
import threading
import unittest
from types import SimpleNamespace
from typing import Optional
from unittest.mock import patch

import gnss_3d_visualizer as viewer

from gnss_3d_visualizer import (
    FOOTER,
    HEADER,
    DemoReceiver,
    HYIPacketDecoder,
    MIN_SCENE_AXIS_RATIO,
    MAX_POINT_CAPACITY,
    MAX_RENDER_POINTS,
    MAX_REPLAY_DURATION_S,
    MAX_REQUEST_BYTES,
    PACKET_LENGTH,
    ReplayController,
    SerialReceiver,
    SourceManager,
    TrajectoryOverlayPoint,
    TrajectoryStore,
    TelemetryPoint,
    TelemetryStore,
    _format_clock,
    _latitude_span_and_center,
    _longitude_span_and_center,
    _map_zoom,
    _downsample_for_display,
    _unwrap_longitudes,
    build_dash_app,
    balanced_scene_aspect_ratio,
    create_map_figure,
    create_trajectory_figure,
    enumerate_serial_port_options,
    llh_to_enu,
    load_dashboard_dependencies,
    normalize_baud_rate,
    normalize_serial_port,
    packet_checksum,
    parse_arguments,
    parse_hyi_packet,
    representative_trajectory_coordinates,
    serial_baud_options,
    terrain_map_style,
)
from telemetry_csv import ReplaySample
from trajectory_io import LoadedTrajectory, TrajectoryRecord


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

    def test_rejects_zero_fix_and_implausible_altitude(self):
        with self.assertRaisesRegex(ValueError, "fix"):
            parse_hyi_packet(make_packet(latitude=0.0, longitude=0.0, altitude=0.0))
        with self.assertRaisesRegex(ValueError, "altitude"):
            parse_hyi_packet(make_packet(altitude=2_000_000.0))


class SerialReceiverTests(unittest.TestCase):
    class FakeConnection:
        def __init__(self, data: bytes, receiver: SerialReceiver):
            self.data = bytearray(data)
            self.receiver = receiver
            self.closed = False

        @property
        def in_waiting(self):
            return len(self.data)

        def read(self, size):
            if not self.data:
                self.receiver.stop_event.set()
                return b""
            chunk = bytes(self.data[:size])
            del self.data[:size]
            return chunk

        def close(self):
            self.closed = True

    def run_receiver_with(
        self,
        data: bytes,
        *,
        baud_rate: int = 19_200,
        max_plausible_speed_mps: float = 0.0,
        recording: bool = False,
        store: Optional[TelemetryStore] = None,
    ):
        if store is None:
            store = TelemetryStore(
                max_points=100,
                max_plausible_speed_mps=max_plausible_speed_mps,
            )
        if recording:
            store.start_recording()
        receiver = SerialReceiver(store, "TEST", baud_rate, reconnect_delay_s=0.0)
        connection = self.FakeConnection(data, receiver)
        old_serial = viewer.serial
        viewer.serial = SimpleNamespace(Serial=lambda *args, **kwargs: connection)
        try:
            receiver.run()
        finally:
            viewer.serial = old_serial
        return store, connection

    def test_backlogged_packets_receive_wire_spaced_monotonic_timestamps(self):
        baud_rate = 19_200
        data = b"".join(make_packet(counter=index) for index in range(60))

        store, connection = self.run_receiver_with(data, baud_rate=baud_rate)
        points, status = store.snapshot()

        self.assertTrue(connection.closed)
        self.assertEqual(status["status"], "connected")
        self.assertEqual(len(points), 60)
        expected_span_s = 59 * PACKET_LENGTH * 10.0 / baud_rate
        self.assertAlmostEqual(
            points[-1].time_s - points[0].time_s,
            expected_span_s,
            delta=0.02,
        )
        self.assertTrue(
            all(left.time_s < right.time_s for left, right in zip(points, points[1:]))
        )

    def test_open_but_silent_port_stays_waiting(self):
        store, _ = self.run_receiver_with(b"not telemetry")
        points, status = store.snapshot()

        self.assertEqual(points, [])
        self.assertEqual(status["status"], "waiting")
        self.assertIn("first valid fix", status["message"])

    def test_rejected_gnss_payload_is_not_also_counted_as_missing(self):
        data = b"".join(
            (
                make_packet(counter=0),
                make_packet(counter=1, latitude=0.0, longitude=0.0, altitude=0.0),
                make_packet(counter=2),
            )
        )

        store, _ = self.run_receiver_with(data)
        points, status = store.snapshot()

        self.assertEqual([point.packet_counter for point in points], [0, 2])
        self.assertEqual(status["rejected"], 1)
        self.assertEqual(status["missing"], 0)

    def test_noise_between_packets_contributes_to_wire_timestamp_spacing(self):
        baud_rate = 19_200
        noise = b"\x00" * 780
        data = (
            make_packet(counter=0, latitude=40.0)
            + noise
            + make_packet(counter=1, latitude=40.0045)
        )

        store, _ = self.run_receiver_with(
            data,
            baud_rate=baud_rate,
            max_plausible_speed_mps=3_000.0,
        )
        points, status = store.snapshot()

        self.assertEqual(status["accepted"], 2)
        expected_spacing_s = (PACKET_LENGTH + len(noise)) * 10.0 / baud_rate
        self.assertAlmostEqual(
            points[1].time_s - points[0].time_s,
            expected_spacing_s,
            delta=0.01,
        )

    def test_recording_skips_packets_backdated_before_button_click(self):
        data = b"".join(make_packet(counter=index) for index in range(60))

        store, _ = self.run_receiver_with(data, recording=True)
        recorded = store.recording_snapshot()

        self.assertLess(len(recorded), 60)
        self.assertTrue(all(point.time_s >= 0.0 for point in recorded))
        self.assertTrue(
            all(
                left.time_s < right.time_s
                for left, right in zip(recorded, recorded[1:])
            )
        )

    def test_reconnect_backlog_cannot_bypass_motion_filter_with_old_time(self):
        store = TelemetryStore(max_points=200, max_plausible_speed_mps=3_000.0)
        self.assertTrue(
            store.append(
                0,
                40.0,
                29.0,
                100.0,
                monotonic_s=viewer.time.monotonic(),
            )
        )
        backlog = b"".join(
            make_packet(counter=index, latitude=40.2)
            for index in range(1, 101)
        )

        self.run_receiver_with(backlog, store=store)
        points, status = store.snapshot()

        self.assertEqual(len(points), 1)
        self.assertEqual(status["accepted"], 1)
        self.assertEqual(status["rejected"], 100)


class DemoReceiverTests(unittest.TestCase):
    class OneSampleStore:
        def __init__(self):
            self.altitudes = []

        def set_status(self, _status, _message):
            return None

        def append(
            self,
            _counter,
            _latitude_deg,
            _longitude_deg,
            altitude_m,
        ):
            self.altitudes.append(altitude_m)
            return True

    class OneIterationEvent:
        def __init__(self):
            self.finished = False

        def is_set(self):
            return self.finished

        def set(self):
            self.finished = True

        def wait(self, _timeout):
            self.finished = True
            return True

    def _run_one_sample(self, receiver, now_s):
        receiver.stop_event = self.OneIterationEvent()
        with patch("gnss_3d_visualizer.time.monotonic", return_value=now_s):
            receiver.run()

    def test_recreated_demo_receiver_continues_shared_mission_clock(self):
        store = self.OneSampleStore()
        factory = viewer.create_demo_source_factory(
            store,
            mission_started_monotonic=100.0,
        )

        self._run_one_sample(factory(), 130.0)
        self._run_one_sample(factory(), 131.0)

        self.assertEqual(len(store.altitudes), 2)
        first_altitude_m, resumed_altitude_m = store.altitudes

        self.assertAlmostEqual(
            first_altitude_m,
            1_400.0 + DemoReceiver.altitude_profile(30.0),
        )
        self.assertAlmostEqual(
            resumed_altitude_m,
            1_400.0 + DemoReceiver.altitude_profile(31.0),
        )
        self.assertLess(resumed_altitude_m, first_altitude_m)


class GeometryAndStoreTests(unittest.TestCase):
    def test_wgs84_enu_handles_antimeridian(self):
        east_m, north_m, up_m = llh_to_enu(
            0.0, -179.999, 0.0, 0.0, 179.999, 0.0
        )

        self.assertAlmostEqual(east_m, 222.64, delta=0.5)
        self.assertAlmostEqual(north_m, 0.0, places=3)
        self.assertAlmostEqual(up_m, 0.0, delta=0.01)

    def test_store_tracks_packet_gaps_and_rejects_duplicates(self):
        store = TelemetryStore(max_points=20, max_plausible_speed_mps=0.0)
        counters = (254, 255, 0, 2)
        for index, counter in enumerate(counters):
            self.assertTrue(
                store.append(
                    counter,
                    40.0,
                    29.0 + index * 0.00001,
                    100.0,
                    elapsed_s=float(index),
                )
            )

        self.assertFalse(
            store.append(2, 40.0, 29.1, 100.0, elapsed_s=5.0)
        )
        self.assertFalse(
            store.append(1, 40.0, 29.1, 100.0, elapsed_s=6.0)
        )
        points, status = store.snapshot()
        self.assertEqual(len(points), 4)
        self.assertEqual(status["missing"], 1)
        self.assertEqual(status["duplicates"], 1)
        self.assertEqual(status["out_of_order"], 1)

    def test_sequence_resynchronizes_after_a_large_forward_loss(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        self.assertTrue(store.append(0, 40.0, 29.0, 100.0, elapsed_s=0.0))
        self.assertFalse(store.append(129, 40.0, 29.0, 100.0, elapsed_s=1.0))
        self.assertFalse(store.append(130, 40.0, 29.0, 100.0, elapsed_s=2.0))
        self.assertTrue(store.append(131, 40.0, 29.0, 100.0, elapsed_s=3.0))
        self.assertTrue(store.append(132, 40.0, 29.0, 100.0, elapsed_s=4.0))

        points, status = store.snapshot()
        self.assertEqual([point.packet_counter for point in points], [0, 131, 132])
        self.assertEqual(status["missing"], 128)
        self.assertEqual(status["out_of_order"], 0)

    def test_sequence_resynchronizes_across_counter_wrap(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        counters = (0, 254, 255, 0, 1)

        accepted = [
            counter
            for index, counter in enumerate(counters)
            if store.append(
                counter,
                40.0,
                29.0 + index * 0.00001,
                100.0,
                elapsed_s=float(index),
            )
        ]
        _, status = store.snapshot()

        self.assertEqual(accepted, [0, 0, 1])
        self.assertEqual(status["missing"], 253)
        self.assertEqual(status["duplicates"], 0)
        self.assertEqual(status["out_of_order"], 0)

    def test_stale_packet_before_wrap_does_not_invent_large_loss(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        counters = (0, 255, 0, 1)

        for index, counter in enumerate(counters):
            store.append(
                counter,
                40.0,
                29.0 + index * 0.00001,
                100.0,
                elapsed_s=float(index),
            )
        points, status = store.snapshot()

        self.assertEqual([point.packet_counter for point in points], [0, 1])
        self.assertEqual(status["missing"], 0)
        self.assertEqual(status["duplicates"], 1)
        self.assertEqual(status["out_of_order"], 1)
        self.assertEqual(status["rejected"], 2)

    def test_store_rejects_impractical_capacity(self):
        with self.assertRaisesRegex(ValueError, "max_points"):
            TelemetryStore(max_points=MAX_POINT_CAPACITY + 1)

    def test_launch_and_apogee_survive_rolling_window(self):
        store = TelemetryStore(max_points=2, max_plausible_speed_mps=0.0)
        for index, altitude in enumerate((100.0, 200.0, 150.0)):
            store.append(
                index,
                40.0,
                29.0,
                altitude,
                elapsed_s=float(index),
            )

        points, status = store.snapshot()
        self.assertEqual(len(points), 2)
        self.assertEqual(status["launch_point"].altitude_m, 100.0)
        self.assertAlmostEqual(status["max_up_m"], 100.0, delta=0.01)

    def test_store_rejects_invalid_fix_before_setting_origin(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)

        self.assertFalse(store.append(0, 0.0, 0.0, 100.0, elapsed_s=0.0))
        self.assertFalse(
            store.append(1, 40.0, 29.0, 2_000_000.0, elapsed_s=1.0)
        )
        self.assertTrue(store.append(2, 40.0, 29.0, 100.0, elapsed_s=2.0))

        points, status = store.snapshot()
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0].up_m, 0.0)
        self.assertEqual(status["rejected"], 2)

    def test_motion_rejection_does_not_double_count_sequence_gap(self):
        store = TelemetryStore(max_plausible_speed_mps=100.0)
        self.assertTrue(store.append(0, 40.0, 29.0, 100.0, elapsed_s=0.0))
        self.assertFalse(store.append(2, 40.0, 30.0, 100.0, elapsed_s=1.0))
        self.assertTrue(
            store.append(3, 40.0, 29.00001, 100.0, elapsed_s=2.0)
        )

        points, status = store.snapshot()
        self.assertEqual(len(points), 2)
        self.assertEqual(status["missing"], 1)
        self.assertEqual(status["rejected"], 1)

    def test_sequence_tracking_can_be_reset_after_reconnect(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        self.assertTrue(store.append(100, 40.0, 29.0, 100.0, elapsed_s=0.0))

        store.reset_sequence_tracking()

        self.assertTrue(store.append(0, 40.0, 29.00001, 100.0, elapsed_s=1.0))
        _, status = store.snapshot()
        self.assertEqual(status["out_of_order"], 0)

    def test_store_rejects_nonincreasing_live_timestamps(self):
        store = TelemetryStore(max_plausible_speed_mps=3_000.0)
        self.assertTrue(
            store.append(0, 40.0, 29.0, 100.0, monotonic_s=100.0)
        )

        self.assertFalse(
            store.append(1, 40.1, 29.0, 100.0, monotonic_s=99.0)
        )
        points, status = store.snapshot()
        self.assertEqual(len(points), 1)
        self.assertEqual(status["rejected"], 1)

    def test_recording_is_empty_until_started_and_bounded(self):
        store = TelemetryStore(max_points=2, max_plausible_speed_mps=0.0)
        store.append(0, 40.0, 29.0, 100.0, elapsed_s=0.0)
        self.assertEqual(store.recording_snapshot(), [])

        store.start_recording()
        for index in range(1, 4):
            store.append(
                index,
                40.0,
                29.0 + index * 0.00001,
                100.0,
                elapsed_s=float(index),
            )
        recorded = store.recording_snapshot()
        self.assertEqual([point.packet_counter for point in recorded], [2, 3])
        self.assertLessEqual(recorded[0].time_s, recorded[1].time_s)

    def test_existing_track_recording_is_rebased(self):
        store = TelemetryStore(max_points=3, max_plausible_speed_mps=0.0)
        for index in range(3):
            store.append(index, 40.0, 29.0, 100.0, elapsed_s=10.0 + index)

        count = store.start_recording(include_existing=True)
        recorded = store.recording_snapshot()

        self.assertEqual(count, 3)
        self.assertEqual([point.time_s for point in recorded], [0.0, 1.0, 2.0])

    def test_existing_track_recording_preserves_silence_before_next_fix(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        store.append(0, 40.0, 29.0, 100.0, monotonic_s=100.0)
        store.append(1, 40.0, 29.00001, 100.0, monotonic_s=110.0)

        with patch("gnss_3d_visualizer.time.monotonic", return_value=210.0):
            store.start_recording(include_existing=True)
        store.append(2, 40.0, 29.00002, 100.0, monotonic_s=211.0)

        self.assertEqual(
            [point.time_s for point in store.recording_snapshot()],
            [0.0, 10.0, 111.0],
        )

    def test_replay_can_seek_and_rebuild_track(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        replay = ReplayController(store)
        samples = [
            ReplaySample(float(index), index, 40.0, 29.0 + index * 0.00001, 100.0)
            for index in range(3)
        ]

        replay.load(samples, "flight.csv")
        replay.seek(1.5)
        points, _ = store.snapshot()

        self.assertEqual(len(points), 2)
        self.assertEqual(replay.status()["index"], 2)
        self.assertTrue(replay.toggle())

    def test_descending_only_replay_is_marked_partial_without_reversing_data(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        replay = ReplayController(store)
        samples = [
            ReplaySample(
                float(index),
                index,
                40.0 + index * 0.00001,
                29.0 + index * 0.00001,
                200.0 - index * 20.0,
            )
            for index in range(6)
        ]

        replay.load(samples, "descent.csv")
        replay.seek(replay.status()["duration_s"])
        points, _ = store.snapshot()

        self.assertFalse(replay.status()["launch_known"])
        self.assertLess(points[-1].up_m, 0.0)
        self.assertGreater(points[-1].east_m, 0.0)
        self.assertGreater(points[-1].north_m, 0.0)
        self.assertEqual([point.time_s for point in points], list(map(float, range(6))))

    def test_full_flight_replay_keeps_launch_origin_classification(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        replay = ReplayController(store)
        altitude = [100.0, 130.0, 180.0, 130.0, 101.0]
        replay.load(
            [
                ReplaySample(float(index), index, 40.0, 29.0, value)
                for index, value in enumerate(altitude)
            ],
            "full-flight.csv",
        )

        self.assertTrue(replay.status()["launch_known"])

    def test_return_from_replay_starts_fresh_live_view_with_recording_off(self):
        class IdleSource:
            def __init__(self):
                self.started = False

            def start(self):
                self.started = True

            def stop(self):
                return None

            def is_alive(self):
                return False

            def join(self, timeout=None):
                return None

        store = TelemetryStore(max_plausible_speed_mps=0.0)
        store.append(10, 40.0, 29.0, 100.0, monotonic_s=100.0)
        store.append(11, 40.0001, 29.0001, 220.0, monotonic_s=110.0)
        source = IdleSource()
        manager = SourceManager(store, lambda: source, "Test live")
        self.assertEqual(store.start_recording(include_existing=True), 2)
        self.assertEqual(store.stop_recording(), 2)

        manager.load_replay(
            [
                ReplaySample(0.0, 1, 41.0, 30.0, 500.0),
                ReplaySample(1.0, 2, 41.0001, 30.0001, 450.0),
            ],
            "temporary.csv",
        )
        manager.seek_replay(1.0)
        manager.start_live()

        live_points, status = store.snapshot()
        self.assertTrue(source.started)
        self.assertEqual(live_points, [])
        self.assertIsNone(status["launch_point"])
        self.assertEqual(status["accepted"], 0)
        self.assertFalse(status["recording"])

        saved_before_new_live_fix = store.recording_snapshot()
        self.assertEqual(
            [point.packet_counter for point in saved_before_new_live_fix],
            [10, 11],
        )
        store.append(12, 40.0002, 29.0002, 230.0, monotonic_s=120.0)
        new_live_points, status = store.snapshot()
        self.assertEqual([point.packet_counter for point in new_live_points], [12])
        self.assertAlmostEqual(new_live_points[0].up_m, 0.0)
        self.assertEqual(status["launch_point"].packet_counter, 12)
        self.assertEqual(store.recording_snapshot(), saved_before_new_live_fix)

        self.assertTrue(manager.toggle_recording())
        self.assertEqual(store.recording_snapshot(), [])

    def test_replay_rejects_nonfinite_or_excessive_duration(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        replay = ReplayController(store)
        base = ReplaySample(0.0, 0, 40.0, 29.0, 100.0)
        for end_time in (float("inf"), MAX_REPLAY_DURATION_S + 1.0):
            with self.subTest(end_time=end_time):
                with self.assertRaisesRegex(ValueError, "7 days"):
                    replay.load(
                        [
                            base,
                            ReplaySample(end_time, 1, 40.0, 29.0, 101.0),
                        ],
                        "bad.csv",
                    )

    def test_trajectory_store_rejects_extreme_time_and_coordinates(self):
        store = TrajectoryStore()
        valid_origin = TrajectoryRecord(time_s=0.0, east_m=0.0, north_m=0.0, up_m=0.0)
        cases = (
            TrajectoryRecord(
                time_s=MAX_REPLAY_DURATION_S + 1.0,
                east_m=1.0,
                north_m=0.0,
                up_m=1.0,
            ),
            TrajectoryRecord(time_s=1.0, east_m=100_000_001.0, north_m=0.0, up_m=1.0),
        )
        for record in cases:
            with self.subTest(record=record):
                with self.assertRaises(ValueError):
                    store.load(LoadedTrajectory("bad", (valid_origin, record), "enu"))

    def test_live_source_reference_is_retained_when_thread_will_not_stop(self):
        class StuckSource:
            stopped = False
            joined = False

            def start(self):
                return None

            def stop(self):
                self.stopped = True

            def is_alive(self):
                return True

            def join(self, timeout=None):
                self.joined = True

        store = TelemetryStore()
        manager = SourceManager(store, lambda: StuckSource(), "Test live")
        stuck = StuckSource()
        manager._live_source = stuck

        with self.assertRaisesRegex(RuntimeError, "still stopping"):
            manager.start_live()

        self.assertIs(manager._live_source, stuck)
        self.assertTrue(stuck.stopped)
        self.assertTrue(stuck.joined)

    def test_invalid_replay_does_not_stop_the_live_source(self):
        class TrackingSource:
            stopped = False

            def stop(self):
                self.stopped = True

            def is_alive(self):
                return True

        store = TelemetryStore()
        manager = SourceManager(store, lambda: TrackingSource(), "Test live")
        source = TrackingSource()
        manager._live_source = source
        invalid = [
            ReplaySample(0.0, 0, 40.0, 29.0, 100.0),
            ReplaySample(
                MAX_REPLAY_DURATION_S + 1.0,
                1,
                40.0,
                29.0,
                101.0,
            ),
        ]

        with self.assertRaisesRegex(ValueError, "7 days"):
            manager.load_replay(invalid, "invalid.csv")

        self.assertFalse(source.stopped)
        self.assertIs(manager._live_source, source)
        self.assertEqual(manager.status()["mode"], "live")

    def test_failed_live_start_restores_loaded_replay(self):
        class FailingSource(threading.Thread):
            def start(self):
                raise RuntimeError("port unavailable")

        store = TelemetryStore()
        manager = SourceManager(store, lambda: FailingSource(), "Test live")
        manager.load_replay(
            [
                ReplaySample(0.0, 0, 40.0, 29.0, 100.0),
                ReplaySample(1.0, 1, 40.0, 29.00001, 101.0),
            ],
            "flight.csv",
        )
        manager.seek_replay(1.0)

        with self.assertRaisesRegex(RuntimeError, "failed to start"):
            manager.start_live()

        points, status = store.snapshot()
        self.assertEqual(manager.status()["mode"], "replay")
        self.assertTrue(manager.status()["loaded"])
        self.assertEqual(len(points), 2)
        self.assertEqual(status["status"], "error")

    def test_controller_and_store_enforce_import_point_limits(self):
        store = TelemetryStore()
        replay = ReplayController(store)
        samples = [
            ReplaySample(float(index), index, 40.0, 29.0, 100.0)
            for index in range(2)
        ]
        with patch.object(viewer, "MAX_REPLAY_SAMPLES", 1):
            with self.assertRaisesRegex(ValueError, "sample limit"):
                replay.load(samples, "too-many.csv")

        trajectory = LoadedTrajectory(
            "too-many",
            (
                TrajectoryRecord(east_m=0.0, north_m=0.0, up_m=0.0),
                TrajectoryRecord(east_m=1.0, north_m=1.0, up_m=1.0),
            ),
            "enu",
        )
        with patch.object(viewer, "MAX_TRAJECTORY_POINTS", 1):
            with self.assertRaisesRegex(ValueError, "point limit"):
                TrajectoryStore().load(trajectory)

    def test_recording_toggle_is_atomic_with_source_mode(self):
        store = TelemetryStore()
        manager = SourceManager(store, lambda: None, "Test live")

        self.assertTrue(manager.toggle_recording())
        self.assertTrue(store.snapshot()[1]["recording"])

        manager.load_replay(
            [ReplaySample(0.0, 0, 40.0, 29.0, 100.0)],
            "flight.csv",
        )
        self.assertFalse(manager.toggle_recording())
        self.assertEqual(manager.status()["mode"], "replay")
        self.assertFalse(store.snapshot()[1]["recording"])

    def test_recording_toggle_only_captures_points_received_after_click(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        store.append(
            10,
            40.0,
            29.0,
            100.0,
            monotonic_s=90.0,
        )
        store.append(
            11,
            40.0,
            29.00001,
            110.0,
            monotonic_s=99.0,
        )
        manager = SourceManager(store, lambda: None, "Test live")

        with patch("gnss_3d_visualizer.time.monotonic", return_value=100.0):
            self.assertTrue(manager.toggle_recording())

        self.assertEqual(store.recording_snapshot(), [])
        store.append(
            12,
            40.0,
            29.00002,
            120.0,
            monotonic_s=101.0,
        )

        recorded = store.recording_snapshot()
        self.assertEqual([point.packet_counter for point in recorded], [12])
        self.assertEqual([point.time_s for point in recorded], [1.0])


class SerialSettingsTests(unittest.TestCase):
    def test_serial_settings_are_normalized_and_bounded(self):
        self.assertEqual(normalize_serial_port("  COM7  "), "COM7")
        self.assertEqual(normalize_baud_rate("115200"), 115_200)

        for invalid_port in (None, "", "COM7\n", "X" * 256):
            with self.subTest(invalid_port=invalid_port):
                with self.assertRaises(ValueError):
                    normalize_serial_port(invalid_port)
        for invalid_baud in (True, 299, 4_000_001, 19_200.5, float("inf")):
            with self.subTest(invalid_baud=invalid_baud):
                with self.assertRaises(ValueError):
                    normalize_baud_rate(invalid_baud)

    def test_port_options_are_naturally_sorted_and_keep_missing_config(self):
        port_provider = SimpleNamespace(
            comports=lambda: [
                SimpleNamespace(device="COM10", description="Telemetry radio"),
                SimpleNamespace(device="COM2", description="USB Serial"),
                SimpleNamespace(device="COM2", description="Duplicate"),
            ]
        )

        with patch.object(viewer, "serial_list_ports", port_provider):
            options = enumerate_serial_port_options("COM9")

        self.assertEqual(
            [option["value"] for option in options],
            ["COM2", "COM9", "COM10"],
        )
        self.assertIn("not detected", options[1]["label"])
        self.assertIn("Telemetry radio", options[2]["label"])

    def test_port_enumeration_failure_keeps_configured_port(self):
        def fail_to_enumerate():
            raise OSError("device scan failed")

        with patch.object(
            viewer,
            "serial_list_ports",
            SimpleNamespace(comports=fail_to_enumerate),
        ):
            options = enumerate_serial_port_options("/dev/ttyUSB0")

        self.assertEqual(options[0]["value"], "/dev/ttyUSB0")
        self.assertIn("not detected", options[0]["label"])

    def test_baud_options_include_a_valid_nonstandard_cli_value(self):
        options = serial_baud_options(250_000)

        self.assertIn(250_000, [option["value"] for option in options])

    def test_reconfigure_serial_stops_recording_and_uses_selected_settings(self):
        class IdleSource:
            def __init__(self):
                self.stopped = False

            def stop(self):
                self.stopped = True

            def is_alive(self):
                return False

            def join(self, timeout=None):
                return None

        store = TelemetryStore()
        store.start_recording()
        old_source = IdleSource()
        manager = SourceManager(
            store,
            lambda: IdleSource(),
            "Serial source / COM9 / 19200 baud",
            serial_port="COM9",
            baud_rate=19_200,
        )
        manager._live_source = old_source

        with patch.object(SerialReceiver, "start", return_value=None) as start:
            applied = manager.configure_serial("  COM5 ", 115_200)

        self.assertEqual(applied, ("COM5", 115_200))
        self.assertTrue(old_source.stopped)
        self.assertEqual(start.call_count, 1)
        self.assertIsInstance(manager._live_source, SerialReceiver)
        self.assertEqual(manager._live_source.serial_port, "COM5")
        self.assertEqual(manager._live_source.baud_rate, 115_200)
        status = manager.status()
        self.assertEqual(status["serial_port"], "COM5")
        self.assertEqual(status["baud_rate"], 115_200)
        self.assertIn("COM5 / 115200 baud", status["source_label"])
        self.assertFalse(store.snapshot()[1]["recording"])
        manager.stop()


class MapAndFormattingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        load_dashboard_dependencies()

    def test_map_zoom_and_center_use_short_path_across_antimeridian(self):
        span, center = _longitude_span_and_center([179.9999, -179.9999])

        self.assertAlmostEqual(span, 0.0002, places=6)
        self.assertAlmostEqual(abs(center), 180.0, places=6)
        self.assertEqual(_map_zoom([0.0, 0.0], [179.9999, -179.9999]), 16.5)

    def test_partial_replay_labels_start_and_hides_misleading_builtin_reference(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        replay = ReplayController(store)
        replay.load(
            [
                ReplaySample(
                    float(index),
                    index,
                    40.0 + index * 0.00001,
                    29.0 + index * 0.00001,
                    200.0 - index * 20.0,
                )
                for index in range(6)
            ],
            "descent.csv",
        )
        replay.seek(replay.status()["duration_s"])
        points, status = store.snapshot()

        figure = create_trajectory_figure(
            points,
            show_reference=True,
            launch_point=status["launch_point"],
            launch_label="Replay start",
            reference_available=False,
            reference_notice=viewer.PARTIAL_REPLAY_PLOT_NOTICE,
        )
        trace_names = [trace.name for trace in figure.data]

        self.assertIn("Replay start", trace_names)
        self.assertNotIn("Built-in reference", trace_names)
        self.assertTrue(
            any(
                viewer.PARTIAL_REPLAY_PLOT_NOTICE in annotation.text
                for annotation in figure.layout.annotations
            )
        )

    def test_map_zoom_handles_equal_longitudes(self):
        span, center = _longitude_span_and_center([29.0, 29.0])

        self.assertEqual(span, 0.0)
        self.assertEqual(center, 29.0)
        self.assertEqual(_map_zoom([40.0, 40.001], [29.0, 29.0]), 16.5)

    def test_terrain_map_uses_dem_hillshade_and_pitched_camera(self):
        figure = create_map_figure([], terrain_enabled=True)
        style = figure.layout.map.style

        self.assertEqual(style["sources"]["terrain-dem"]["type"], "raster-dem")
        self.assertEqual(style["terrain"]["source"], "terrain-dem")
        self.assertTrue(
            any(layer["type"] == "hillshade" for layer in style["layers"])
        )
        self.assertGreater(figure.layout.map.pitch, 45.0)
        self.assertEqual(figure.layout.uirevision, "keep-map-terrain")

    def test_empty_map_keeps_maplibre_terrain_visible_while_waiting_for_gnss(self):
        figure = create_map_figure([], terrain_enabled=True)

        self.assertEqual(len(figure.data), 1)
        self.assertEqual(figure.data[0].type, "scattermap")
        self.assertEqual(figure.data[0].name, "Map anchor")
        self.assertEqual(figure.data[0].marker.opacity, 0.0)
        self.assertGreater(figure.layout.map.zoom, 10.0)
        self.assertEqual(
            figure.layout.annotations[0].text,
            "Waiting for a live GNSS fix",
        )

    def test_empty_map_uses_persistent_launch_as_terrain_center(self):
        launch = TelemetryPoint(
            time_s=0.0,
            epoch_s=0.0,
            packet_counter=1,
            latitude_deg=41.2,
            longitude_deg=28.8,
            altitude_m=120.0,
            east_m=0.0,
            north_m=0.0,
            up_m=0.0,
        )

        figure = create_map_figure([], launch_point=launch)

        self.assertAlmostEqual(figure.layout.map.center.lat, 41.2)
        self.assertAlmostEqual(figure.layout.map.center.lon, 28.8)
        self.assertEqual(figure.data[0].name, "Map anchor")

    def test_flat_map_fallback_disables_pitch_and_custom_dem(self):
        figure = create_map_figure([], terrain_enabled=False)

        self.assertEqual(figure.layout.map.style, "open-street-map")
        self.assertEqual(figure.layout.map.pitch, 0.0)
        self.assertEqual(figure.layout.map.bearing, 0.0)
        self.assertEqual(figure.layout.uirevision, "keep-map-flat")

    def test_dark_terrain_style_preserves_osm_attribution(self):
        style = terrain_map_style("dark")

        self.assertIn("OpenStreetMap", style["sources"]["osm"]["attribution"])
        self.assertLess(
            style["layers"][1]["paint"]["raster-brightness-max"],
            0.6,
        )

    def test_map_fit_uses_web_mercator_scale_at_high_latitude(self):
        span, center = _latitude_span_and_center([80.0, 85.0])
        zoom = _map_zoom([80.0, 85.0], [0.0, 0.0])

        self.assertGreater(span, 0.1)
        self.assertAlmostEqual(center, 82.9267, places=3)
        self.assertLess(zoom, 3.0)

        reference = [
            TrajectoryOverlayPoint(0.0, 0.0, 0.0, 0.0, 80.0, 0.0, 100.0),
            TrajectoryOverlayPoint(1.0, 1.0, 1.0, 1.0, 85.0, 0.0, 101.0),
        ]
        figure = create_map_figure([], trajectory_points=reference)
        self.assertAlmostEqual(figure.layout.map.center.lat, center)
        self.assertAlmostEqual(figure.layout.map.zoom, zoom)

    def test_map_fit_is_conservative_for_mobile_longitude_extent(self):
        longitude_span = 20.0
        zoom = _map_zoom([40.0, 40.0], [0.0, longitude_span])
        projected_width = (
            512.0 * (2.0**zoom) * (longitude_span / 360.0)
        )

        self.assertLessEqual(projected_width, 300.0 * 0.85 + 1e-6)

    def test_reference_only_map_centers_on_uploaded_llh_path(self):
        reference = [
            TrajectoryOverlayPoint(0.0, 0.0, 0.0, 0.0, 40.0, 29.0, 100.0),
            TrajectoryOverlayPoint(1.0, 1.0, 1.0, 1.0, 40.01, 29.01, 101.0),
        ]

        figure = create_map_figure([], trajectory_points=reference)

        self.assertAlmostEqual(figure.layout.map.center.lat, 40.005, places=4)
        self.assertAlmostEqual(figure.layout.map.center.lon, 29.005)
        self.assertFalse(figure.layout.annotations)

    def test_local_reference_is_projected_onto_map_from_launch_fix(self):
        launch = TelemetryPoint(
            time_s=0.0,
            epoch_s=0.0,
            packet_counter=1,
            latitude_deg=40.0,
            longitude_deg=29.0,
            altitude_m=100.0,
            east_m=0.0,
            north_m=0.0,
            up_m=0.0,
        )
        reference = [
            TrajectoryOverlayPoint(0.0, 0.0, 0.0, 0.0),
            TrajectoryOverlayPoint(1.0, 100.0, 50.0, 20.0),
        ]

        figure = create_map_figure(
            [launch],
            trajectory_points=reference,
            launch_point=launch,
            trajectory_name="ENU reference",
        )
        reference_trace = next(
            trace for trace in figure.data if trace.name == "ENU reference"
        )

        self.assertAlmostEqual(reference_trace.lat[0], 40.0)
        self.assertAlmostEqual(reference_trace.lon[0], 29.0)
        self.assertGreater(reference_trace.lat[1], 40.0)
        self.assertGreater(reference_trace.lon[1], 29.0)

    def test_builtin_reference_is_projected_onto_map_when_enabled(self):
        launch = TelemetryPoint(
            0.0, 0.0, 1, 40.0, 29.0, 100.0, 0.0, 0.0, 0.0
        )

        figure = create_map_figure(
            [launch],
            launch_point=launch,
            include_builtin_reference=True,
            trajectory_name="Built-in reference",
        )
        reference_trace = next(
            trace for trace in figure.data if trace.name == "Built-in reference"
        )

        self.assertEqual(len(reference_trace.lat), 121)
        self.assertAlmostEqual(reference_trace.lat[0], 40.0)
        self.assertAlmostEqual(reference_trace.lon[0], 29.0)

    def test_map_uses_persistent_launch_point_outside_rolling_window(self):
        store = TelemetryStore(max_points=2, max_plausible_speed_mps=0.0)
        for index in range(3):
            store.append(
                index,
                40.0,
                29.0 + index * 0.001,
                100.0,
                elapsed_s=float(index),
            )
        points, status = store.snapshot()

        figure = create_map_figure(points, launch_point=status["launch_point"])
        launch_trace = next(trace for trace in figure.data if trace.name == "Launch")

        self.assertAlmostEqual(launch_trace.lon[0], 29.0)

    def test_map_center_matches_the_full_launch_to_current_extent(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        store.append(0, 0.1, -100.0, 100.0, elapsed_s=0.0)
        store.append(1, 0.1, 100.0, 100.0, elapsed_s=1.0)
        points, status = store.snapshot()

        figure = create_map_figure(points, launch_point=status["launch_point"])

        self.assertAlmostEqual(abs(figure.layout.map.center.lon), 180.0)
        self.assertGreater(figure.layout.map.zoom, 0.0)
        self.assertLess(figure.layout.map.zoom, 1.0)

    def test_map_trace_unwraps_antimeridian_segments(self):
        store = TelemetryStore(max_plausible_speed_mps=0.0)
        store.append(0, 0.1, 179.999, 100.0, elapsed_s=0.0)
        store.append(1, 0.1, -179.999, 100.0, elapsed_s=1.0)
        points, _ = store.snapshot()

        figure = create_map_figure(points)
        ground_track = next(
            trace for trace in figure.data if trace.name == "Ground track"
        )

        self.assertLess(abs(ground_track.lon[1] - ground_track.lon[0]), 1.0)
        self.assertEqual(_unwrap_longitudes([179.999, -179.999]), [179.999, 180.001])

    def test_3d_range_includes_launch_outside_rolling_window(self):
        store = TelemetryStore(max_points=2, max_plausible_speed_mps=0.0)
        for index in range(3):
            store.append(
                index,
                40.0,
                29.0 + index * 0.012,
                100.0,
                elapsed_s=float(index),
            )
        points, status = store.snapshot()

        figure = create_trajectory_figure(
            points,
            launch_point=status["launch_point"],
        )

        self.assertLessEqual(figure.layout.scene.xaxis.range[0], 0.0)

    def test_3d_scene_keeps_a_low_variation_axis_visible(self):
        points = [
            TelemetryPoint(0.0, 0.0, 0, 40.0, 29.0, 100.0, 0.0, 0.0, 0.0),
            TelemetryPoint(1.0, 1.0, 1, 40.0, 29.0, 0.0, 0.0, 0.0, -100.0),
        ]

        figure = create_trajectory_figure(points, show_reference=True)
        ratio = figure.layout.scene.aspectratio

        self.assertEqual(figure.layout.scene.aspectmode, "manual")
        self.assertGreaterEqual(ratio.x, MIN_SCENE_AXIS_RATIO)
        self.assertGreaterEqual(ratio.y, MIN_SCENE_AXIS_RATIO)
        self.assertGreaterEqual(ratio.z, MIN_SCENE_AXIS_RATIO)
        self.assertAlmostEqual(max(ratio.x, ratio.y, ratio.z), 1.0)

    def test_3d_camera_revision_resets_persisted_camera(self):
        point = TelemetryPoint(
            0.0, 0.0, 0, 40.0, 29.0, 100.0, 0.0, 0.0, 0.0
        )

        figure = create_trajectory_figure([point], camera_revision=7)

        self.assertEqual(figure.layout.uirevision, "trajectory-camera-7")
        self.assertEqual(figure.layout.scene.dragmode, "turntable")

    def test_balanced_scene_aspect_ratio_validates_minimum(self):
        with self.assertRaisesRegex(ValueError, "within"):
            balanced_scene_aspect_ratio([0.0], [0.0], [0.0], 0.0)

    def test_clock_handles_nonfinite_values(self):
        self.assertEqual(_format_clock(float("inf")), "--:--")
        self.assertEqual(_format_clock(float("nan")), "--:--")

    def test_display_downsampling_is_bounded_and_keeps_endpoints(self):
        values = list(range(MAX_RENDER_POINTS + 25))
        displayed = _downsample_for_display(values)

        self.assertEqual(len(displayed), MAX_RENDER_POINTS)
        self.assertEqual(displayed[0], values[0])
        self.assertEqual(displayed[-1], values[-1])

    def test_dash_rejects_oversized_requests_before_callbacks(self):
        store = TelemetryStore()
        manager = SourceManager(
            store,
            lambda: None,
            "Serial source / COM9 / 19200 baud",
            serial_port="COM9",
            baud_rate=19_200,
        )
        app = build_dash_app(store, manager, TrajectoryStore())

        self.assertEqual(app.server.config["MAX_CONTENT_LENGTH"], MAX_REQUEST_BYTES)

        component_ids = set()

        def visit(component):
            component_id = getattr(component, "id", None)
            if component_id:
                component_ids.add(component_id)
            children = getattr(component, "children", None)
            if isinstance(children, (list, tuple)):
                for child in children:
                    visit(child)
            elif children is not None:
                visit(children)

        visit(app.layout)
        self.assertTrue(
            {
                "serial-port-select",
                "serial-baud-select",
                "serial-refresh",
                "serial-connect",
                "fullscreen-toggle",
            }.issubset(component_ids)
        )

    def test_dash_upload_decoder_enforces_decoded_size_limit(self):
        encoded = "data:text/csv;base64," + "QUFB" * 3
        with patch.object(viewer, "MAX_UPLOAD_BYTES", 4):
            with self.assertRaisesRegex(ValueError, "16 MiB"):
                viewer._decode_dash_upload(encoded)


class RepresentativeTrajectoryTests(unittest.TestCase):
    def test_reference_path_has_launch_apogee_and_landing(self):
        east, north, up = representative_trajectory_coordinates()

        self.assertEqual(len(east), 121)
        self.assertEqual((east[0], north[0], up[0]), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(east[-1], 80.0)
        self.assertAlmostEqual(up[-1], 0.0, places=6)
        self.assertAlmostEqual(max(up), 120.0, places=6)

    def test_reference_path_requires_two_samples(self):
        with self.assertRaisesRegex(ValueError, "at least 2"):
            representative_trajectory_coordinates(1)


class CommandLineTests(unittest.TestCase):
    def test_rejects_nonfinite_speed_and_stale_timeout(self):
        for option in ("--max-speed", "--stale-timeout"):
            for value in ("nan", "inf"):
                with self.subTest(option=option, value=value):
                    with patch("sys.argv", ["viewer", option, value]):
                        with self.assertRaises(SystemExit):
                            parse_arguments()

    def test_rejects_impractically_large_max_points(self):
        with patch(
            "sys.argv",
            ["viewer", "--max-points", str(MAX_POINT_CAPACITY + 1)],
        ):
            with self.assertRaises(SystemExit):
                parse_arguments()

    def test_rejects_baud_outside_supported_runtime_range(self):
        for baud_rate in ("299", "4000001"):
            with self.subTest(baud_rate=baud_rate):
                with patch("sys.argv", ["viewer", "--baud", baud_rate]):
                    with self.assertRaises(SystemExit):
                        parse_arguments()


if __name__ == "__main__":
    unittest.main()
