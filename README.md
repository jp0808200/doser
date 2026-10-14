# Acid-Dosing Unit Firmware

Firmware for an ESP32-C3 based water-tank acid dosing unit. It counts water
flow, triggers up to 4 daily dosing times, doses a user-defined acid volume
with a stepper pump (NEMA 17), verifies acid-line presence, provides a local
web UI / captive portal for configuration and calibration, and can optionally
report status over MQTT.

- **Platform:** ESP32-C3 Super Mini, Arduino framework, PlatformIO

---

## 1. Hardware

| Signal           | GPIO | Notes                                        |
| ---------------- | ---- | -------------------------------------------- |
| STEP             | 0    | Stepper pump (DRV8825)                       |
| DIR              | 1    | 1 = reverse (purge)                          |
| ENABLE           | 3    | Active **low** (HIGH = motor disabled)       |
| Flow meter pulse | 4    | Falling edge, low-active pulse, ≥200 µs wide |
| Acid line detect | 5    | HIGH = acid present in line                  |
| Tank empty       | 6    | HIGH = tank empty                            |
| DRV fault        | 7    | Driver fault pin                             |
| Status LED       | 10   | Blinks 1 Hz while IDLE/booting               |

Supply 12 V to the driver/pump.

---

## 2. Building & Flashing

Prerequisite: [PlatformIO](https://platformio.org) (CLI or the VS Code
extension). Dependencies (ArduinoJson, ESPAsyncWebServer) are fetched
automatically via `lib_deps` in `platformio.ini`.

```bash
# Production build (no test injection)
pio run -e acid-doser
pio run -e acid-doser -t upload
pio device monitor        # 115200 baud
```

For hardware-in-the-loop / bench testing:

```bash
pio run -e acid-doser-test -t upload   # enables serial test injection
```

Useful VS Code shortcuts: the **PlatformIO** toolbar (build / upload /
monitor), or `Ctrl+Shift+P` → *PlatformIO: Build*, *Upload*, *Serial Monitor*.

---

## 3. First-Time Setup

1. Power up with no saved Wi-Fi credentials. The unit starts an **open
   access point** `AcidDoser-XXXX` (last MAC bytes) at **`192.168.4.1`**.
   Join it from a phone/laptop — most devices pop a captive-portal
   redirect to the setup page automatically; if not, open
   `http://192.168.4.1` manually.
2. On the **Config** page, enter your Wi-Fi SSID/password. Saving reboots
   the unit in STA mode.
3. In STA mode the web UI requires **HTTP Basic auth**. Default
   credentials: **`admin` / `admin123`** — change them on the Config page
   (`Web user/password`) right after first login.

Recovery to setup mode: clear NVS (`doser*` namespaces) or use
`POST /api/reboot` after clearing stored Wi-Fi so a fresh AP is started.

---

## 4. Web UI

Base URL: the unit's LAN IP (STA) or `http://192.168.4.1` (AP).

| Page           | Purpose                                                      |
| -------------- | ------------------------------------------------------------ |
| `/`            | Live status: state, line/tank/fault flags, water since dose, |
|                | dose statistics, Wi-Fi/MQTT status                           |
| `/config`      | Wi-Fi, web auth, schedule slots, motor speed, dose factor,   |
|                | MQTT, NTP/timezone                                           |
| `/calibration` | Flow-meter and pump calibration                              |
| `/stats`       | Cumulative statistics (doses, water, acid volumes)           |
| `/debug/log`   | Recent log entries (JSON)                                    |

The full JSON API includes `GET /api/state`,
`GET /api/stats`, `POST /api/motor_speed`, `/api/manual_dose`,
`/api/schedule`, `/api/dose_factor`, `/api/wifi`, `/api/web_auth`,
`/api/mqtt`, `/api/time`, `/api/cal_*`, `/api/ack`, `/api/reboot`.

Example — manual dose trigger:

```bash
curl -u admin:admin123 -X POST http://<unit>/api/manual_dose
```

---

## 5. Calibration

### 5.1 Flow meter (pulses per liter)

Default: **1380 pulse/litre**.

1. Open `/calibration` → *Flow calibration start*
   (`POST /api/cal_flow_start`).
2. Let a measured, known volume of water pass the meter (or wait ~2.5 s
   after pulses stop for the prompt).
3. Stop the session and enter the exact volume in liters
   (`POST /api/cal_flow_stop` with the volume). The firmware computes and
   stores pulses/liter (valid range 10 … 100 000).

### 5.2 Pump (ml per step)

Default: **0.05 ml/step**. The metered run always pumps a fixed 10000 steps.

1. *Pump calibration start* (`POST /api/cal_pump_start`) — the pump runs.
2. Collect the discharged acid and measure it.
3. *Confirm* with the measured volume (`POST /api/cal_pump_confirm`).
   Accepted result range: 0.001 … 10 ml/step.

---

## 6. Dosing Configuration

| Parameter       | Default               | Range             | Meaning                                                        |
| --------------- | --------------------- | ----------------- | -------------------------------------------------------------- |
| Motor speed     | 800 Hz                | 100 … 800 Hz      | The **only** user-adjustable speed; all pump operations use it |
| Dose factor     | 1.0                   | 0.001 … 1000 ml/L | Acid ml dosed per liter of water passed                        |
| Schedule        | 4 slots, all disabled | HH:MM local       | Up to 4 daily dosing times                                     |
| NTP server      | `pool.ntp.org`        |                   | Time sync; required so schedule slots fire at local time       |
| Timezone offset | 0                     | minutes           | Offset from UTC                                                |

Dosing caps: **200 ml** or 100000 steps per dose. Each dose = forward prime →
line-liquid verify → metered steps → reverse purge (max 30 s each phase).

---

## 7. Operation & Safety

- **Normal cycle:** water flows → flow pulses accumulate → at a schedule
  time (or manual trigger) the unit primes, verifies acid in the line,
  doses the computed volume, then purges.
- **Status LED:** blinks while healthy/IDLE; solid/other patterns indicate
  fault state.
- **Faults** (no acid in line during dosing, DRV8825 fault,
  prime/purge timeouts, loop stalls, watchdog resets) latch a safe state:
  motor disabled, dosing halted. Warnings/faults can be acknowledged via
  `POST /api/ack` when safe.
- **Fatal faults** hold a safe shutdown (2 s) then enter **deep sleep**;
  the unit does not auto-restart in a faulted state.
- A 5 ms hardware watchdog is fed by the main loop; >3 consecutive WDT
  resets trigger the safe-shutdown path.

---

## 8. MQTT Reporting (optional)

Enable on the Config page. Defaults: base topic `acid_doser`, port 1883.
Messages are published with **QoS 1**, not retained.

| Topic               | Content       | Interval (default)                            |
| ------------------- | ------------- | --------------------------------------------- |
| `acid_doser/status` | Unit state    | 30 s                                          |
| `acid_doser/alert`  | Alerts/faults | On event (queue of 8, oldest dropped if full) |
| `acid_doser/stats`  | Statistics    | 300 s                                         |

---

## 9. Serial Console & Test Injection

Baud **115200**. Logs print continuously (heartbeat every 10 s, full log
also at `GET /debug/log`).

With the `acid-doser-test` build (`-DENABLE_TEST_INJECTION`), serial
commands drive the same state machine without hardware (spec §5.2):

```text
flowinject 1000     # inject 1000 flow pulses
line 1 | line 0     # acid line present / not present
tank 1 | tank 0     # tank empty / not empty
fault 1 | fault 0   # driver fault asserted / cleared
manual_dose         # trigger a dose command
state               # dump state machine
stats               # dump statistics
log                 # dump log buffer
reboot              # soft reboot
```

Test injection is **compiled out of the production build**
(`acid-doser` env).

---

## 10. Project Layout

```
src/
  main.cpp              # setup()/loop() entry, super-loop
  app_core.{h,cpp}      # system state machine + loop servicing
  dose_scheduler.*      # up to 4 daily slots once-per-day logic
  dosing_controller.*   # prime → verify → meter → purge state machine
  stepper_engine.*      # ramped motion, HW-timer step ISR, purge
  flow_meter.*          # pulse ISR + volume tracking
  calibration_manager.* # flow & pump calibration sessions
  safety_monitor.*      # sensor latching, fault detection
  clock_manager.*       # NTP, local time, day-of-year
  config_store.*        # NVS persistence (config/stats/boot)
  config_update.*       # web→main-loop config change queue
  wifi_controller.*     # STA/AP, captive portal, reconnect backoff
  web_controller.*      # AsyncWebServer UI + JSON API
  web_json.*            # request-body helpers
  mqtt_reporter.*       # esp-mqtt client, topic publishing
  alert_queue.*         # bounded ring drained by MQTT / HIL
  logger.*              # 64-entry circular log buffer
  watchdog.*            # task WDT + reset-cause tracking
  hal.{h,cpp}           # fixed pin map, motor/sensor access
  pins.h / constants.h / names.h / types.h
```

Design rules: single cooperative loop, **no application
FreeRTOS tasks**, no blocking delays, no dynamic allocation, ISR-safe
shared data via critical sections.
