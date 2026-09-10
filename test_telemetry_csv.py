import math
import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest.mock import patch

from telemetry_csv import ReplaySample, parse_telemetry_csv, telemetry_points_to_csv


class ReplaySampleTests(unittest.TestCase):
    def test_is_immutable(self):
        sample = ReplaySample(0.0, 1, 40.0, 29.0, 100.0)

        with self.assertRaises(FrozenInstanceError):
            sample.time_s = 2.0


class ParseTelemetryCsvTests(unittest.TestCase):
    def test_parses_comma_csv_and_aliases(self):
        samples = parse_telemetry_csv(
            "time,counter,lat,lon,alt\n"
            "0.0,7,40.1,29.2,120.5\n"
            "0.25,8,40.2,29.3,121.5\n"
        )

        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0], ReplaySample(0.0, 7, 40.1, 29.2, 120.5))
        self.assertEqual(samples[1].time_s, 0.25)

    def test_parses_semicolon_turkish_headers_decimal_comma_and_cp1254(self):
        text = (
            "# Eski uçuş kaydı\n"
            "elapsed;packet_counter;ENLEM;BOYLAM;İRTİFA\n"
            '0,0;12;"40,962105";"29,129418";"1523,5"\n'
        )

        samples = parse_telemetry_csv(text.encode("cp1254"))

        self.assertEqual(samples[0].packet_counter, 12)
        self.assertAlmostEqual(samples[0].latitude_deg, 40.962105)
        self.assertAlmostEqual(samples[0].longitude_deg, 29.129418)
        self.assertAlmostEqual(samples[0].altitude_m, 1523.5)

    def test_parses_utf8_bom_tab_delimited_file_and_comments(self):
        data = (
            "\ufeff// generated telemetry\n"
            "time_s\tgps_latitude\tgps_longitude\tgnss_altitude\n"
            "1.5\t41.0\t30.0\t250\n"
        ).encode("utf-8")

        samples = parse_telemetry_csv(data)

        self.assertEqual(samples, [ReplaySample(1.5, 0, 41.0, 30.0, 250.0)])

    def test_generates_time_and_counter_when_columns_are_absent(self):
        samples = parse_telemetry_csv(
            "latitude,longitude,altitude\n40,29,100\n40.1,29.1,101\n",
            default_interval_s=0.2,
        )

        self.assertEqual([sample.time_s for sample in samples], [0.0, 0.2])
        self.assertEqual([sample.packet_counter for sample in samples], [0, 1])

    def test_epoch_timestamp_is_retained_and_converted_to_elapsed_time(self):
        samples = parse_telemetry_csv(
            "timestamp,latitude,longitude,altitude,counter\n"
            "1700000000.25,40,29,100,3\n"
            "1700000000.75,40.1,29.1,101,4\n"
        )

        self.assertEqual([sample.time_s for sample in samples], [0.0, 0.5])
        self.assertEqual(
            [sample.epoch_s for sample in samples],
            [1700000000.25, 1700000000.75],
        )

    def test_small_timestamp_is_treated_as_elapsed_time(self):
        samples = parse_telemetry_csv(
            "timestamp,latitude,longitude,altitude\n"
            "3.5,40,29,100\n4.0,40,29,101\n"
        )

        self.assertEqual([sample.time_s for sample in samples], [3.5, 4.0])
        self.assertEqual([sample.epoch_s for sample in samples], [None, None])

    def test_iso_timestamp_is_supported(self):
        samples = parse_telemetry_csv(
            "timestamp,latitude,longitude,altitude\n"
            "2026-09-09T10:00:00Z,40,29,100\n"
            "2026-09-09T10:00:01.250+00:00,40,29,101\n"
        )

        self.assertEqual([sample.time_s for sample in samples], [0.0, 1.25])
        self.assertIsNotNone(samples[0].epoch_s)

    def test_explicit_elapsed_and_epoch_columns_are_both_preserved(self):
        samples = parse_telemetry_csv(
            "time_s,epoch_s,latitude_deg,longitude_deg,altitude_m\n"
            "2.5,1700000002.5,40,29,100\n"
            "3.0,,40,29,101\n"
        )

        self.assertEqual(samples[0].time_s, 2.5)
        self.assertEqual(samples[0].epoch_s, 1700000002.5)
        self.assertEqual(samples[1].epoch_s, None)

    def test_skips_invalid_rows_without_consuming_generated_sequence(self):
        samples = parse_telemetry_csv(
            "latitude,longitude,altitude\n"
            "91,29,100\n"
            "40,nan,100\n"
            "40,29,not-a-number\n"
            "40,29,100\n"
            "40.1,29.1,101\n",
            default_interval_s=0.5,
        )

        self.assertEqual(len(samples), 2)
        self.assertEqual([sample.time_s for sample in samples], [0.0, 0.5])
        self.assertEqual([sample.packet_counter for sample in samples], [0, 1])

    def test_skips_non_integral_or_negative_packet_counter(self):
        samples = parse_telemetry_csv(
            "latitude,longitude,altitude,counter\n"
            "40,29,100,1.5\n"
            "40,29,100,-1\n"
            "40,29,100,2\n"
        )

        self.assertEqual([sample.packet_counter for sample in samples], [2])

    def test_skips_zero_fix_and_implausible_altitude(self):
        samples = parse_telemetry_csv(
            "latitude,longitude,altitude\n"
            "0,0,100\n"
            "40,29,2000000\n"
            "40,29,100\n"
        )

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].altitude_m, 100.0)

    def test_uses_excel_sep_declaration_and_ignores_metadata_before_header(self):
        samples = parse_telemetry_csv(
            "sep=;\nmetadata row\nlatitude;longitude;altitude\n40;29;100\n"
        )

        self.assertEqual(len(samples), 1)

    def test_rejects_file_without_required_header(self):
        with self.assertRaisesRegex(ValueError, "latitude, longitude, and altitude"):
            parse_telemetry_csv("time,value\n0,1\n")

    def test_rejects_file_with_no_valid_rows(self):
        with self.assertRaisesRegex(ValueError, "No valid telemetry rows"):
            parse_telemetry_csv(
                "latitude,longitude,altitude\n91,29,100\n40,181,100\n"
            )

    def test_rejects_invalid_default_interval(self):
        data = "latitude,longitude,altitude\n40,29,100\n"
        for value in (0, -0.1, math.inf, math.nan, "bad"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "default_interval_s"):
                    parse_telemetry_csv(data, value)

    def test_rejects_oversized_input_before_parsing(self):
        with patch("telemetry_csv.MAX_CSV_BYTES", 8):
            with self.assertRaisesRegex(ValueError, "16 MiB"):
                parse_telemetry_csv("x" * 9)

    def test_rejects_more_than_the_sample_limit(self):
        data = (
            "latitude,longitude,altitude\n"
            "40,29,100\n"
            "40.1,29.1,101\n"
            "40.2,29.2,102\n"
        )
        with patch("telemetry_csv.MAX_REPLAY_SAMPLES", 2):
            with self.assertRaisesRegex(ValueError, "sample limit"):
                parse_telemetry_csv(data)


class ExportTelemetryCsvTests(unittest.TestCase):
    def test_exports_duck_typed_main_application_points(self):
        point = SimpleNamespace(
            time_s=1.25,
            epoch_s=1700000001.25,
            packet_counter=9,
            latitude_deg=40.962105,
            longitude_deg=29.129418,
            altitude_m=1523.5,
            east_m=4.0,
            north_m=5.0,
            up_m=6.0,
        )

        exported = telemetry_points_to_csv([point])

        self.assertEqual(
            exported,
            "time_s,packet_counter,latitude_deg,longitude_deg,altitude_m,epoch_s\n"
            "1.25,9,40.962105,29.129418,1523.5,1700000001.25\n",
        )

    def test_export_round_trip_preserves_samples_and_optional_epoch(self):
        expected = [
            ReplaySample(0.0, 0, 40.0, 29.0, 100.0),
            ReplaySample(0.1, 1, 40.1, 29.1, 101.0, 1700000000.1),
        ]

        actual = parse_telemetry_csv(telemetry_points_to_csv(expected))

        self.assertEqual(actual, expected)

    def test_empty_export_contains_a_replayable_header(self):
        exported = telemetry_points_to_csv([])

        self.assertEqual(
            exported,
            "time_s,packet_counter,latitude_deg,longitude_deg,altitude_m,epoch_s\n",
        )

    def test_export_rejects_invalid_coordinates(self):
        invalid = ReplaySample(0.0, 1, 100.0, 29.0, 50.0)

        with self.assertRaisesRegex(ValueError, "latitude"):
            telemetry_points_to_csv([invalid])

    def test_export_rejects_zero_fix_and_implausible_altitude(self):
        with self.assertRaisesRegex(ValueError, "GNSS fix"):
            telemetry_points_to_csv([ReplaySample(0.0, 1, 0.0, 0.0, 50.0)])
        with self.assertRaisesRegex(ValueError, "altitude"):
            telemetry_points_to_csv(
                [ReplaySample(0.0, 1, 40.0, 29.0, 2_000_000.0)]
            )

    def test_export_reports_missing_fields_as_value_error(self):
        with self.assertRaisesRegex(ValueError, "missing a required field"):
            telemetry_points_to_csv([SimpleNamespace(time_s=0.0)])


if __name__ == "__main__":
    unittest.main()
