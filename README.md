# GNSS 3D Flight Viewer

Current version: **v2**

This application reads the same 78-byte HYI telemetry packet used by the
existing project and displays the GNSS trajectory in an interactive 3D view.

## Packet fields

- Header: `FF FF 54 52`
- Packet counter: byte `5`
- Payload GNSS altitude: bytes `22..25`, float32 little-endian
- Payload GNSS latitude: bytes `26..29`, float32 little-endian
- Payload GNSS longitude: bytes `30..33`, float32 little-endian
- Checksum: byte `75`, `sum(bytes[4..74]) % 256`
- Footer: `0D 0A`
- Total size: 78 bytes

## Install

Open this directory as a PyCharm project, select a Python 3.9+ interpreter,
then run:

```powershell
python -m pip install -r requirements.txt
```

## Test without hardware

```powershell
python gnss_3d_visualizer.py --demo
```

## Run with the serial receiver

```powershell
python gnss_3d_visualizer.py --serial-port COM7 --baud 19200
```

The dashboard opens at `http://127.0.0.1:8050`. The 3D camera can be rotated
with the mouse and zoomed with the wheel. Use `Reset track` to select the next
valid GNSS fix as the new ENU origin.

## Parser test

```powershell
python -m unittest -v
```
