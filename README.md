# Proist Roket Takımı Bilimsel Görev Yazılımı

A local Dash dashboard for live HYI GNSS telemetry, flight recording and replay,
custom reference trajectories, and synchronized 3D/2D visualization.

## Highlights

- Live serial input for the existing 78-byte HYI packet, with automatic reconnect
  and runtime port/baud selectors.
- Built-in demo flight for setup and UI checks without hardware.
- Serial port and baud controls appear only in `Real flight`; Demo mode never
  exposes or opens a COM port.
- Ground-station location from the browser/Windows location service, with a
  manual latitude, longitude, and altitude fallback when permission or a fix is
  unavailable.
- Bounded in-memory recording and one-click export to a replayable CSV file.
- A live DVR timeline: inspect an earlier moment while reception and an active
  recording continue, then select `LIVE` to jump to the newest packet.
- Saved-file replay controls: play/pause, restart, seek, and 0.25x to 10x speed.
- Interactive WGS84/ECEF-to-ENU 3D flight corridor with a rocket marker.
- Open-by-default pitched OpenStreetMap ground track with real DEM terrain and
  hillshade; no map token is required and a flat-map fallback is included.
- Reference paths are visible by default; local ENU/OpenRocket paths are anchored
  to the launch fix and projected onto the terrain map as well as the 3D corridor.
- Uploadable ENU, latitude/longitude/altitude, or OpenRocket-style 2D trajectories.
- Altitude profile, flight-phase estimate, packet loss/duplicate counters, stale-data
  status, apogee, ground range, speed, flight time, and distance flown.
- Responsive dark/light interface. Telemetry files stay on the computer; the browser
  requests OSM tiles for the currently viewed map area.
- Browser fullscreen mode with automatic Plotly resizing; press `Esc` or select
  `Exit fullscreen` to return to the normal dashboard.

## Install on Windows

Python 3.9 or newer is required. In PowerShell, from this directory:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

If PowerShell blocks venv activation, the interpreter can be called directly:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

For PyCharm, open this repository folder as the project and select the `.venv`
interpreter once. Then select the included `Rocket Flight Viewer` configuration and
press **Run**. No program parameters are required. The browser opens on port 8071
and the dashboard asks whether to use `Demo flight` or `Real flight`.

Demo and Real flight are not separate programs or Git branches. They are two data
sources inside the same `gnss_3d_visualizer.py` application and share the same
dashboard, maps, recording, replay, and trajectory tools.

## Start the application

```powershell
.\.venv\Scripts\python.exe gnss_3d_visualizer.py
```

The dashboard opens at <http://127.0.0.1:8071> without starting a telemetry source.
Select `Demo flight` to start a fresh synthetic mission without hardware. Demo
points are marked `DEMO` throughout the interface and are never receiver
measurements. The COM port and baud controls are hidden in this mode. Select
`Real flight` to reveal those controls, choose the port and baud rate, and then
select `Connect` to start real telemetry. Switching modes clears the displayed
track and always leaves recording off.

Command-line options remain available for advanced or automated use, but they are
not needed for normal PyCharm operation. For example, `--demo` starts the synthetic
source immediately and `--no-browser` prevents automatic browser opening.

Useful options:

```text
--http-port 8071       Dashboard port
--max-points 20000     Number of live/recording points retained in memory
--max-speed 3000       Reject physically implausible 3D jumps (m/s); 0 disables
--stale-timeout 2.0    Mark the stream stale after this many seconds
```

Run `python gnss_3d_visualizer.py --help` for the complete command reference.
The `SERIAL` controls in the command bar list detected ports, retain the configured
port when it is temporarily unplugged, and apply a new port/baud only when
`Connect` is selected. Use `Ports ↻` after plugging in a receiver. The app never
silently switches to another detected port. These controls are shown only after
`Real flight` is selected; Demo and saved-file replay do not need serial settings.

## Ground-station location

When the dashboard opens, it asks the browser for the computer's current location
and marks an accepted fix as `Ground station` on the map. This uses the browser and
Windows location service rather than the rocket receiver. The browser may request
permission, and a desktop estimate can be less accurate than a GNSS fix. Open the
compact `Yer istasyonu` popover from the upper-right header to see or change it.

If permission is denied, the request times out, or no position is available, enter
the ground-station latitude and longitude in the popover and select `Konumu kaydet`.
Altitude is optional because browsers frequently do not provide it. `Bilgisayardan
al` requests a fresh automatic fix. The ground-station marker is independent of the
launch datum, so changing it does not rotate, translate, or rewrite recorded rocket
telemetry and reference trajectories.

## Record and replay

1. Select `Start recording` while the live or demo source is running.
2. Select `Stop recording`, then `Export CSV`.
3. Later, choose `Load replay CSV` and select the exported file.
4. Use Play/Pause, Restart, the timeline, and the speed selector. `Return to live`
   or `Return to demo` leaves saved-file replay and reconnects the previously
   selected source.

The live DVR timeline is separate from saved-file replay. While a live or Demo
source is running, drag its timeline backward to inspect buffered telemetry. The
viewer shows `BEHIND LIVE`, but the serial/demo receiver keeps accepting new
packets in the background. If `REC ON` is active, recording also continues without
gaps. Select `LIVE` to jump directly to the newest buffered packet; this does not
disconnect the receiver, start a new flight, or toggle recording. The rolling DVR
history is bounded by `--max-points`, so its oldest samples eventually expire.

Recording is strict opt-in: only fixes received after `Start recording` are added
to the export buffer. Returning from replay never restores the old live or replay
path; it restarts the previously selected source with a clean plot and leaves
recording stopped. A new real flight appears only after the receiver supplies a new
fix. `REC OFF` and `REC ON` are shown explicitly.
Previously saved points can still be exported while the indicator says
`REC OFF · N saved`. Recording is capped by `--max-points`, preventing an
unattended session from growing memory indefinitely. Uploaded CSV files are limited
to 16 MiB and 250,000 replay samples.

Legacy files that contain only a descending segment cannot reveal the original
launch datum. The viewer keeps their recorded order and coordinates unchanged,
labels the first point `Replay start`, and hides the non-comparable built-in launch
reference with an explicit warning. A user-loaded trajectory remains available for
intentional comparison.

The default `3D Map + Flight` workspace keeps the terrain recovery map, ENU flight
corridor, altitude profile, and replay cursor synchronized. The terrain control in
the map header switches to a flat OpenStreetMap view when DEM tiles are unavailable
or a lighter GPU workload is preferred. OSM and DEM tiles require internet access;
telemetry processing and replay remain local.

The canonical recording columns are:

```csv
time_s,packet_counter,latitude_deg,longitude_deg,altitude_m,epoch_s
```

Latitude, longitude, and altitude are required. Common aliases such as `time`,
`timestamp`, `lat`, `lon`, `alt`, `enlem`, `boylam`, and `irtifa` are accepted.
The reader supports UTF-8, Windows-1254, comma, semicolon, and tab-delimited files.
See [`examples/replay_sample.csv`](examples/replay_sample.csv) for a ready-to-load
example.

## Load a reference trajectory

Use `Load trajectory` for any of these formats:

- Local ENU: `Time (s), East (m), North (m), Up (m)`
- Geodetic: `Time (s), Latitude (deg), Longitude (deg), Altitude (m)`
- OpenRocket-style 2D: `Time (s), Altitude (m), Downrange (m)`

Recognized length units include metres, kilometres, centimetres, millimetres, and
feet. Time can be seconds, milliseconds, or minutes; geodetic angles can be degrees
or radians. OpenRocket 2D downrange is placed on the local east axis. Comment lines
beginning with `#` are allowed. See
[`examples/openrocket_trajectory_sample.csv`](examples/openrocket_trajectory_sample.csv).
Trajectory imports are limited to 50,000 points; dense paths are automatically
downsampled for responsive display while their full extent is retained for fitting.

## HYI packet layout

- Header: `FF FF 54 52`
- Packet counter: byte `5`
- GNSS altitude: bytes `22..25`, little-endian float32
- GNSS latitude: bytes `26..29`, little-endian float32
- GNSS longitude: bytes `30..33`, little-endian float32
- Checksum: byte `75`, `sum(bytes[4..74]) % 256`
- Footer: `0D 0A`
- Total size: 78 bytes

Invalid checksums, out-of-range coordinates, zero fixes, impossible altitudes,
duplicates, out-of-order counters, and implausible position jumps are handled before
the track is updated.

## Test

```powershell
.\.venv\Scripts\python.exe -m unittest discover -v
```

The CSV parsers use only the Python standard library, so recordings and trajectories
can also be validated independently of the dashboard dependencies.
