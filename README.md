# Alicat MFC + Particulate Logger — Web Serial App

A single-file, zero-install web app for controlling an **Alicat mass flow controller (MFC)** *and* logging a **particulate sensor** (PM1.0 / PM2.5 / PM10) directly from your browser over USB/RS‑232 serial ports, using the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

Set flow rates manually, watch live readings, run **timed flow programs**, and **log both devices together** to a timestamped CSV — all from one `index.html` file with no dependencies, build step, or server required.

The UI is organized into three tabs:

| Tab | Purpose |
| --- | --- |
| **Flow Controller** | Connect and command the Alicat MFC (FTDI "USB Serial Port", 19200 baud). |
| **Particulate Sensor** | Connect and read a PM sensor (Silicon Labs CP210x, 115200 baud). |
| **Combined + Logging** | Live snapshot of both devices and timestamped CSV logging to a file. |

![Web Serial · 19200 baud 8N1 · ASCII protocol](https://img.shields.io/badge/Web%20Serial-19200%208N1-3da9fc)

---

## Features

- **Direct serial control** — talks to the controller over the Web Serial API; no drivers, agents, or native apps.
- **Configurable device ID** — single letter `A`–`Z` (factory default `A`; this app defaults to `D`). Every command is re-addressed automatically when you change it.
- **Manual setpoint** — type a flow value and Set (or press Enter); one‑click "Set 0" for a quick stop.
- **Live readings** — poll once or auto-poll at a configurable interval (default 500 ms) to display pressure, temperature, volumetric flow, **mass flow**, setpoint, and gas.
- **Timed flow program** — build a table of *(flow, hold-seconds)* steps; run them in sequence with a live countdown, optional looping, and a Stop button.
- **Serial log** — every transmitted/received line, color-coded, for verifying the protocol and debugging.

---

### Particulate sensor

- **Separate COM port** at **115200 baud, 8N1**, filtered to **Silicon Labs CP210x** (USB vendor ID `0x10C4`) devices in the port picker (toggle off to see all ports).
- Listen-only — the app sends no commands; it just parses each streamed line.
- Default expected line format (once per second): `index,PM1.0,PM2.5,PM10` (the leading index column is ignored). The parser also auto-detects **labeled** (`PM2.5: 12.3`) and **JSON** lines; you can force a specific mode from the **Line format** dropdown.

### Combined logging

- Live snapshot of PM values plus flow setpoint, mass flow, and gas.
- Writes timestamped CSV rows at a configurable interval to a file you choose (via the [File System Access API](https://developer.mozilla.org/en-US/docs/Web/API/File_System_API)); browsers without it collect rows for a one-click download instead.
- CSV columns: `timestamp, pm1_0_ugm3, pm2_5_ugm3, pm10_ugm3, flow_setpoint, mass_flow, gas` (timestamp is ISO‑8601).

## Requirements

- A **Chromium-based desktop browser**: Chrome, Edge, or Opera. Web Serial is **not** available in Firefox or Safari, nor on mobile.
- The page must be served from a **secure context**: `https://`, `localhost`, or opened as a local `file://`.
- An Alicat MFC and/or PM sensor connected via USB/RS‑232 adapters, and the COM ports they enumerate as. The two devices connect independently — you can use either or both.

---

## Quick start

1. **Get the file** — clone the repo or just download `index.html`:
   ```sh
   git clone https://github.com/cjwhome/AllecatControllerWeb.git
   ```
2. **Open it** — double-click `index.html` (opens via `file://`), or serve the folder:
   ```sh
   # optional: any static server works
   python -m http.server 8000
   # then visit http://localhost:8000
   ```
3. **Connect** — confirm the baud rate (**19200** by default) and **Device ID** (`D`), then click **Connect** and pick the controller's COM port in the browser dialog.
4. **Set a flow** — enter a setpoint and click **Set**.
5. **Watch it** — tick **Auto-poll** to stream live readings.

---

## Usage

### Connection
| Control | Purpose |
| --- | --- |
| **Baud rate** | Serial speed. Default **19200**; must match the controller's configured baud. |
| **Device ID** | Single letter addressing the unit. All commands target this ID. |
| **Connect / Disconnect** | Open or close the serial port. |

### Set flow rate
Enter a setpoint and click **Set** (or press Enter). Sends the Alicat command `‹ID›S‹value›` (e.g. `DS5.0`). The controller echoes a data frame that populates **Live Readings**. **Set 0** forces the setpoint to zero.

### Live readings
- **Poll once** — sends the bare ID (`D`) to request a single data frame.
- **Auto-poll** — repeatedly polls at the configured interval (ms). Recommended while running a program so you can watch the flow track the setpoint.

Displayed fields: Pressure, Temperature, Volumetric flow, **Mass flow**, Setpoint, Gas.

### Timed flow program
1. Add steps with **+ Add step**; set each row's **Flow setpoint** and **Hold (seconds)**.
2. Click **Run program**. The app sets the flow for step 1, holds for its duration, then advances — repeating through every step. The active row is highlighted and a countdown shows time remaining.
3. **Loop** repeats the whole program until you press **Stop**.

The app ships seeded with an example program: flow **5** for 10 s → **10** for 10 s → **0** for 5 s.

---

## Serial protocol

This app implements the **Alicat ASCII ("Standard") protocol**. Default serial settings: **19200 baud, 8 data bits, no parity, 1 stop bit (8N1)**. Commands are terminated with a carriage return (`\r`).

| Action | Command sent | Example |
| --- | --- | --- |
| Poll live data | `‹ID›` | `D` |
| Set flow setpoint | `‹ID›S‹value›` | `DS5.0` |

**Data frame** returned by an MFC is whitespace-delimited, parsed as:

```
‹ID›  ‹Pressure›  ‹Temperature›  ‹Volumetric›  ‹Mass›  ‹Setpoint›  ‹Gas›
```

Example:

```
D +014.70 +024.99 +00.000 +00.000 +000.00     Air
```

> **Units** (SCCM/SLPM, PSIA, °C, …) are whatever the controller itself is configured to report. The device transmits numeric values only, so the UI labels them generically.

If your firmware emits columns in a different order, adjust `parseFrame()` in `index.html` — the raw line is visible in the Serial Log to compare against.

---

## How it works

Everything lives in [`index.html`](index.html):

- `connect()` opens the port via `navigator.serial.requestPort()` / `port.open()` and wires a `TextEncoderStream` writer plus a streaming reader.
- `readLoop()` buffers incoming bytes and splits on `\r`/`\n` into complete lines.
- `send()` writes a CR-terminated command and waits briefly for the response line.
- `parseFrame()` / `updateReadout()` decode and display the Alicat data frame.
- `runProgram()` drives the timed sequence with async/await and a per-step countdown.

No external libraries, no bundler — open the file and it runs.

---

## Troubleshooting

- **"This browser does not support the Web Serial API"** — use Chrome/Edge/Opera on desktop over `https://`, `localhost`, or `file://`.
- **No COM port in the dialog** — confirm the USB/serial adapter is installed and the controller is powered; close any other program (e.g. terminal software, vendor tools) holding the port.
- **Connected but no readings** — verify the **Device ID** matches the unit and the **baud rate** is correct. Watch the Serial Log: if `TX D` gets no `RX`, the ID or baud is likely wrong.
- **Garbled / unexpected readings** — compare the raw `RX` line in the log against the expected frame order above and adjust `parseFrame()`.

---

## Safety

This tool commands real gas-flow hardware. Always verify setpoints and units before running, ensure your gas delivery system is safe for the requested flow, and keep the **Set 0** / **Stop** controls within reach. Use at your own risk.

---

## License

MIT — see below or add a `LICENSE` file.

```
MIT License — provided as-is, without warranty of any kind.
```
