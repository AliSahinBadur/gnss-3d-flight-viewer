import math
import unittest
from dataclasses import FrozenInstanceError
from unittest.mock import patch

from trajectory_io import LoadedTrajectory, TrajectoryRecord, parse_trajectory_csv


class TrajectoryCsvTests(unittest.TestCase):
    def test_enu_csv_accepts_unit_headers_and_optional_time(self):
        loaded = parse_trajectory_csv(
            "time_s,east_m,north_m,up_m\n0,0,0,0\n0.5,4,3,12\n",
            r"C:\flights\qualification.csv",
        )

        self.assertEqual(loaded.name, "qualification")
        self.assertEqual(loaded.mode, "enu")
        self.assertEqual(
            loaded.records[1],
            TrajectoryRecord(time_s=0.5, east_m=4.0, north_m=3.0, up_m=12.0),
        )

    def test_enu_time_column_is_optional(self):
        loaded = parse_trajectory_csv("East (m);North [m];Up (m)\n0;0;0\n1;2;3")

        self.assertEqual(loaded.mode, "enu")
        self.assertIsNone(loaded.records[0].time_s)

    def test_llh_cp1254_turkish_headers_and_decimal_commas(self):
        source = (
            "Uçuş Zamanı (s);Enlem (deg);Boylam (deg);İrtifa (m)\n"
            "0;40,962105;29,129418;1523,5\n"
            "1;40,962205;29,129518;1530,0\n"
        ).encode("cp1254")

        loaded = parse_trajectory_csv(source, "uçuş.csv")

        self.assertEqual(loaded.mode, "llh")
        self.assertAlmostEqual(loaded.records[0].latitude_deg, 40.962105)
        self.assertAlmostEqual(loaded.records[0].longitude_deg, 29.129418)
        self.assertAlmostEqual(loaded.records[0].altitude_m, 1523.5)

    def test_utf8_bom_and_tab_delimiter(self):
        source = "\ufefflatitude_deg\tlongitude_deg\taltitude_m\n40\t29\t10\n41\t30\t20\n"

        loaded = parse_trajectory_csv(source.encode("utf-8"), "bom.tsv")

        self.assertEqual(loaded.name, "bom")
        self.assertEqual(loaded.mode, "llh")
        self.assertEqual(len(loaded.records), 2)

    def test_openrocket_commented_header_and_event_comments(self):
        source = """# OpenRocket simulation export
# Simulation: Test flight
# Time (s),Altitude (m),Lateral distance (m),Vertical velocity (m/s)
0.0,0.0,0.0,0.0
# Event IGNITION at 0.0 seconds
0.5,10.0,1.25,20.0
1.0,25.0,3.5,30.0
"""

        loaded = parse_trajectory_csv(source, "openrocket-export.csv")

        self.assertEqual(loaded.mode, "openrocket_2d")
        self.assertEqual(
            loaded.records[-1],
            TrajectoryRecord(time_s=1.0, east_m=3.5, north_m=0.0, up_m=25.0),
        )

    def test_openrocket_downrange_alias_and_imperial_units_are_converted(self):
        loaded = parse_trajectory_csv(
            "Time (ms);Altitude (ft);Downrange (ft)\n"
            "0;0;0\n1000;100;25\n"
        )

        self.assertEqual(loaded.mode, "openrocket_2d")
        self.assertAlmostEqual(loaded.records[1].time_s, 1.0)
        self.assertAlmostEqual(loaded.records[1].up_m, 30.48)
        self.assertAlmostEqual(loaded.records[1].east_m, 7.62)

    def test_enu_altitude_is_accepted_as_vertical_coordinate(self):
        loaded = parse_trajectory_csv(
            "Position East of launch (m),Position North of launch (m),Altitude (m)\n"
            "0,0,0\n1,2,3\n"
        )

        self.assertEqual(loaded.mode, "enu")
        self.assertEqual(loaded.records[1].up_m, 3.0)

    def test_malformed_missing_nonfinite_and_out_of_range_rows_are_skipped(self):
        loaded = parse_trajectory_csv(
            "time,lat,lon,alt\n"
            "0,40,29,100\n"
            "bad,40.1,29.1,101\n"
            "2,nan,29.2,102\n"
            "3,95,29.3,103\n"
            "4,40.4,29.4,inf\n"
            "5,40.5,29.5,105\n"
        )

        self.assertEqual(len(loaded.records), 2)
        self.assertEqual([record.time_s for record in loaded.records], [0.0, 5.0])

    def test_optional_time_may_be_blank(self):
        loaded = parse_trajectory_csv(
            "time,east,north,up\n,0,0,0\n1,1,1,1\n"
        )

        self.assertIsNone(loaded.records[0].time_s)

    def test_radian_geodetic_headers_are_converted_to_degrees(self):
        loaded = parse_trajectory_csv(
            "Latitude (rad),Longitude (rad),Altitude (m)\n"
            f"{math.radians(40)},{math.radians(29)},100\n"
            f"{math.radians(41)},{math.radians(30)},110\n"
        )

        self.assertAlmostEqual(loaded.records[0].latitude_deg, 40.0)
        self.assertAlmostEqual(loaded.records[0].longitude_deg, 29.0)

    def test_requires_two_valid_points(self):
        with self.assertRaisesRegex(ValueError, "at least 2 valid points; found 1"):
            parse_trajectory_csv("east,north,up\n0,0,0\nnan,1,2\n")

    def test_unrecognized_header_has_actionable_error(self):
        with self.assertRaisesRegex(ValueError, "header not recognized.*East/North/Up"):
            parse_trajectory_csv("foo,bar,baz\n1,2,3\n4,5,6\n")

    def test_data_type_is_checked(self):
        with self.assertRaisesRegex(TypeError, "str or bytes"):
            parse_trajectory_csv(123)  # type: ignore[arg-type]

    def test_rejects_oversized_input_before_parsing(self):
        with patch("trajectory_io.MAX_CSV_BYTES", 8):
            with self.assertRaisesRegex(ValueError, "16 MiB"):
                parse_trajectory_csv("x" * 9)

    def test_rejects_more_than_the_point_limit(self):
        data = "east,north,up\n0,0,0\n1,1,1\n2,2,2\n"
        with patch("trajectory_io.MAX_TRAJECTORY_POINTS", 2):
            with self.assertRaisesRegex(ValueError, "point limit"):
                parse_trajectory_csv(data)

    def test_result_objects_are_immutable_and_records_are_a_tuple(self):
        loaded = parse_trajectory_csv("east,north,up\n0,0,0\n1,1,1\n")

        self.assertIsInstance(loaded, LoadedTrajectory)
        self.assertIsInstance(loaded.records, tuple)
        with self.assertRaises(FrozenInstanceError):
            loaded.mode = "llh"  # type: ignore[misc]
        with self.assertRaises(FrozenInstanceError):
            loaded.records[0].east_m = 10.0  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
