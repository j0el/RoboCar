# ENHANCEMENTS.md — specs for future work

Specs only — none of this is implemented. Each item says what it is, why
it's worth doing, a design sketch against the current code, and how to test
it. Priorities: **P1** = high value / low risk, next up. **P2** = valuable,
some design risk. **P3** = later / after hardware experience.

Cross-cutting constraints (from CLAUDE.md): Pi code stays lightweight
(2 GB RAM, uv-managed deps), Pico stays single-file MicroPython, safety
invariants (watchdog, 20 Hz stream, `V 0 0 0` on every exit) are untouchable.

---

## 1. Pico firmware (`pico_pi/pico_motor_controller.py`)

### 1.1 Battery voltage telemetry — P1

- **What:** Read pack voltage and stream it as a new telemetry line
  `B <mv>\n` every ~2 s alongside `D` lines.
- **Why:** Li-ion packs sag; a browning-out Pi mid-run looks like a software
  bug. The phone should show battery state, and the Pi can refuse to START
  below a threshold.
- **Design:** The Pico's VSYS/3 divider on ADC3 (GPIO29) reads the board's
  5 V-ish rail, which tracks pack state only loosely — better is a two-resistor
  divider from `VIN` to a free ADC pin (GP26/27/28 are taken by buzzer? GP26
  is the buzzer — verify a free ADC channel on the expansion board first;
  worst case use VSYS/3 as a coarse proxy). Calibrate the divider constant
  once against a multimeter. Firmware: sample in the main loop on a 2 s
  timer, `print("B %d" % mv)`. Pi: parse in `PicoLink._drain()` (same
  pattern as `D`), expose in `/status` JSON as `battery_mv`; app and browser
  page display it, red below threshold.
- **Test:** `sim/pico_exerciser.py` grows a battery readout; compare
  against a multimeter at two charge levels.

### 1.2 State feedback on the WS2812 LEDs + buzzer — P1

- **What:** New `L <mode>\n` command (Pi → Pico): drive the 4 WS2812 LEDs
  (GP11) and buzzer (GP26) to show robot state. Modes: `idle`, `search`,
  `approach`, `arrived` (chirp + blue flash), `blocked` (red), `stopped`.
  Firmware autonomously overrides to `blocked` while the ultrasonic block
  is active and flashes red when the watchdog fires.
- **Why:** During demos and bring-up you can't see the terminal. The robot
  itself should show what it thinks it's doing — and the firmware-level
  override means safety states are visible even if the Pi is wedged.
- **Design:** MicroPython has `neopixel` built in (`machine` + `neopixel`,
  no external lib — keeps the single-file rule). `cone_visitor.py` sends
  `L <state>` only on state *transitions* (not at 20 Hz). Unknown modes
  ignored; `V` parsing untouched. Buzzer: short PWM chirp on `arrived`,
  never continuous.
- **Test:** exerciser gets an `l` key cycling modes; visual check.

### 1.3 Non-blocking ultrasonic via echo IRQ — P2

- **What:** Replace the blocking `time_pulse_us` read (up to ~18 ms) with
  trigger + a rising/falling IRQ on the echo pin recording timestamps.
- **Why:** Today every ultrasonic sample can stall command parsing ~18 ms.
  At 20 Hz commands that's tolerable (worst case one command late), which
  is why this is P2 — but IRQ-based reads remove the jitter entirely and
  make room for faster sampling (~30 Hz) if the safety stop needs it.
- **Design:** `Pin.irq(handler, RISING|FALLING)` storing `ticks_us` deltas;
  main loop triggers, then collects a completed measurement flag next pass.
  Timeout logic (no echo within 20 ms → `D -1`) stays in the loop.
- **Test:** exerciser: confirm `D` cadence rises and hand-tracking still
  works; regression: watchdog test still stops in 0.5 s.

### 1.4 Camera pan servo in SEARCH — P3

- **What:** Use the pan "neck" servo (GP7) — new `S <deg>\n` command
  (0–180, 90 = center). Later: cone_visitor sweeps the camera during SEARCH
  instead of spinning the whole robot, then re-centers and turns the base
  toward the found cone.
- **Why:** Faster, smoother searching; less floor scrub on carpet.
- **Design:** Firmware: 50 Hz PWM, duty span calibrated (typical SG90:
  0.5–2.5 ms); slew-rate-limit motion to protect the mount; center on boot
  and on watchdog. Pi: bearing math must add the servo angle offset before
  steering — do this only after the fixed-camera behavior is proven on
  hardware. The sim should grow a `pan` degree of freedom first.
- **Test:** sim first (add pan to `World.render()`), then hardware.

---

## 2. Pi programs (`raspberry_pi/`)

### 2.1 Runtime config file + live tuning endpoint — P1

- **What:** Move tunables that get adjusted per-environment (HSV bounds,
  arrive height/distance, speeds/gains) from constants into
  `raspberry_pi/robocar_config.json`, loaded at startup. Fold the
  hsv_tuner UI into cone_visitor as a `/tune` page that adjusts values
  live and can persist them (`POST /tune/save`).
- **Why:** Today tuning means editing `vision.py` over SSH and restarting.
  Lighting changes (indoors→outdoors) become a 30-second phone/browser
  operation instead. This is the single biggest bring-up quality-of-life
  win, and it subsumes the "outdoor lighting robustness" backlog item's
  practical half.
- **Design:** `vision.py` gets `load_config()/save_config()`; module-level
  constants become a config object read each frame (cheap dict lookups).
  `cone_visitor.py` serves `/tune` (sliders like hsv_tuner + numeric fields
  for gains) and a live mask preview stream (`/mask`). `hsv_tuner.py` is
  retired (its page moves in). `cone_follower.py` reads the same config.
  Config file is git-ignored; a `robocar_config.default.json` is committed.
- **Test:** sim: change HSV via `/tune` against the synthetic world and
  watch detection break/recover; unit test for load/save round-trip.

### 2.2 Lap counting with a start-marker cone — P2

- **What:** One visually distinct marker (e.g. a green cone or taped cone)
  placed at the start. cone_visitor counts a lap each time the marker cone
  is the one it arrives at, shows `lap N` in `/status` and on the video
  overlay, and optionally stops after `--laps M`.
- **Why:** Long-standing v2 item; turns an endless demo into a finishable
  run ("3 laps and park").
- **Design:** Second HSV range in the config (`marker_low/high`);
  `detect_cones()` gains a `kind` field (`cone`/`marker`) by running both
  masks (one extra inRange per frame — cheap). The FSM treats the marker
  as a normal cone for navigation; `ARRIVED` at a marker increments the lap
  counter (debounced: only counts if ≥ 2 other cones were visited since the
  last marker arrival, so orbiting the marker can't double-count). At
  `--laps M`, transition to a new terminal state `DONE` (stop, `L arrived`
  lights).
- **Test:** sim: make cone index 0 the marker (distinct render color +
  config), assert lap increments once per cycle and `DONE` after M laps.

### 2.3 Phone-heartbeat dead-man for autonomy — P1

- **What:** Optional mode (`--require-heartbeat`, default ON when started
  via the app): autonomy auto-STOPs if no HTTP request (any endpoint) has
  arrived for N seconds (default 5).
- **Why:** Today, if the phone disconnects or the operator walks off, the
  robot keeps circling forever. The Pico watchdog protects against a dead
  *Pi*, not a dead *operator*. Cheap, big safety win for demos.
- **Design:** `Shared.last_request_t` updated in the handler; main loop
  checks it while `running` and flips to STOPPED with a logged reason;
  `/status` reports `stop_reason`. The app already polls `/status` at 2 Hz,
  so a connected phone is automatically a heartbeat. Browser page too.
- **Test:** sim: start, stop polling, assert auto-stop after N s;
  app: toggle WiFi off mid-run and watch the sim robot stop.

### 2.4 Run logging + post-run review — P2

- **What:** `--log` writes a JSONL run log (timestamp, state, target
  bearing/height, command, distance, cone count — one line per control
  tick, ~10 KB/min) and saves a JPEG every N seconds into `runs/<ts>/`.
  `/log` lists runs, `/log/<ts>` downloads a zip.
- **Why:** When a live run misbehaves, today there's nothing to look at
  afterwards. The sim caught three behavior bugs precisely because every
  run was traceable; hardware runs deserve the same.
- **Design:** A `RunLogger` class in cone_visitor, no-op unless enabled;
  ring-limit disk use (delete oldest beyond 200 MB). A small offline
  script `raspberry_pi/replay.py` re-renders a log as an annotated timeline
  (states over time, commands) for the Mac.
- **Test:** sim run with `--log`, replay it, verify the three historical
  bugs would have been visible (state chatter, repeated arrivals).

### 2.5 Camera-failure handling — P1 (small)

- **What:** If `cap.read()` fails ~1 s continuously, transition to STOPPED,
  set `status=camera_error`, keep serving HTTP (page shows the error), try
  to reopen the camera every 5 s.
- **Why:** Today a wedged USB camera leaves the program spinning with a
  stale frame-less loop and no operator-visible signal.
- **Design:** counter in the main loop; `open_camera()` retry with backoff;
  `/status` gains `camera_ok`.
- **Test:** sim can't test this; on hardware, unplug the camera mid-run.

---

## 3. Android app (`android_controller/`)

### 3.1 Virtual joystick manual control — P2

- **What:** Replace (or complement, via a toggle) the six buttons with a
  touch pad: drag vector = vx/vy (full diagonal mecanum crabbing), plus a
  horizontal slider or second small pad for yaw. Sends the same `/drive`
  at 10 Hz while touched.
- **Why:** Buttons can't express diagonals — the whole point of mecanum.
  Manual driving is also the fallback demo when vision is being retuned.
- **Design:** Custom `View` with `onTouchEvent`, dead-zone in the middle,
  values scaled to ±100; still zero third-party deps. Keep STOP button.
  Layout switches to landscape: video left, pads right.
- **Test:** against `sim/pi_server_sim.py` — drive the simulated robot
  diagonally and watch the pose overlay.

### 3.2 Connection robustness + discovery — P2

- **What:** Auto-reconnect the video/status/drive threads with backoff when
  the stream drops; remember the last-good IP; try `10.42.0.1` then the
  remembered IP automatically on launch. Show a clear
  connected/connecting/lost banner.
- **Why:** WiFi handoffs and Pi restarts currently require manual
  reconnect; a demo shouldn't need fiddling.
- **Design:** Wrap the three threads in a supervisor that restarts them
  while "connected" is desired; `SharedPreferences` for the IP. (True mDNS
  discovery is possible via NSD but the AP case makes the fixed IP
  reliable — keep it simple.)
- **Test:** kill/restart the sim server repeatedly; toggle phone WiFi.

### 3.3 Status richness — P3 (small)

- **What:** Show lap count (2.2), battery (1.1), `stop_reason` (2.3), and a
  red "BLOCKED" badge when `distance_cm` < 15; vibrate briefly on block.
- **Design:** All fields already flow through `/status`; UI-only change.

---

## 4. Simulators (`sim/`)

### 4.1 Promote scratch tests into a committed test suite — P1

- **What:** Move the session-scratchpad tests (FSM unit test, circle-layout
  end-to-end, irregular-layout end-to-end, exerciser pty loopback) into
  `tests/` as pytest files, plus a GitHub Actions workflow running them on
  every push (opencv-python-headless + numpy on ubuntu-latest; the pty test
  runs on Linux too).
- **Why:** These tests already caught three real behavior bugs, but they
  currently live in a temp directory and will be lost. Behavior changes
  (gains, thresholds) need a regression gate before hardware runs.
- **Design:** `tests/test_visitor_fsm.py`, `tests/test_sim_ccw.py` (both
  layouts, sim-time injection helper moved into `sim/pi_server_sim.py` as a
  `SimClock`), `tests/test_exerciser_loopback.py` (skipped on non-POSIX).
  A `pyproject.toml` at repo root or reuse `raspberry_pi/`'s with a dev
  group. CI: `uv run pytest`.
- **Test:** the suite is the test; CI must pass on a fresh clone.

### 4.2 Top-down world view in the sim page — P2

- **What:** Add `/map` to the sim's web page: a second image stream showing
  the world from above — cones, robot pose arrow, breadcrumb trail of the
  last ~60 s, sonar beam wedge.
- **Why:** The camera view shows what the robot *sees*; debugging behavior
  needs what it *does*. During this session that view had to be inferred
  from logs.
- **Design:** ~60 lines: render world coords to a 480×480 image each tick
  (already have all state), second key in the `latest` dict, page shows the
  two streams side by side. Sim-only; no change to Pi code.
- **Test:** visual; assert the endpoint returns JPEG parts.

### 4.3 Imperfection modeling — P2

- **What:** CLI flags to inject realism: `--noise` (detection dropout
  probability per frame per cone, bearing jitter), `--latency N` (command
  pipeline delay in ticks), `--motor-skew` (per-axis velocity scale error,
  e.g. strafe 15% slow), `--sonar-noise` (spurious short readings).
- **Why:** The real robot will have all four. The FSM's thresholds
  (3-frame confirmation, 0.8 s lost timeout, hysteresis) exist to survive
  them — but they've only been tested clean. Tuning robustness in sim is
  free; on hardware it costs afternoons.
- **Design:** A thin `Imperfect` wrapper: filters/perturbs the cone list
  after `detect_cones`, delays commands through a deque, scales velocities
  in `World.tick`, corrupts `ultra_cm`. Seeded RNG for reproducibility.
  Add one CI test at moderate noise that must still complete a CCW lap.
- **Test:** sweep noise levels; document the level where behavior breaks
  and adjust confirmation thresholds if it's embarrassingly low.

### 4.4 Program-1 (cone_follower) support in the sim — P3

- **What:** A `--program follower` mode running cone_follower's control law
  (extract its FOLLOW logic into an importable class first, mirroring
  Visitor) and a test asserting it orbits the cluster CCW at standoff
  without contact.
- **Why:** Program 1 has never been exercised anywhere; the sim exists now.
- **Design:** Small refactor of cone_follower into `Follower.step(cones)`
  + thin main; sim gains a flag choosing the brain.

---

## 5. Internet-remote phase (v3 groundwork) — P3

- **What:** Operate the robot from a phone on the internet, not just the
  local AP — the stated later goal. Spec the seam now so nothing built
  above conflicts with it.
- **Design (recommended):** Tailscale on the Pi. The phone joins the same
  tailnet and talks to the *same* HTTP API at the Pi's tailscale IP —
  zero code changes to cone_visitor; the app only needs the IP field it
  already has. WiFi mode flips from AP to client (a second NetworkManager
  profile + a small toggle script `setup_ap.sh --client`). MJPEG over
  tailnet is fine at 640×480 q70 (~1 MB/s); if cellular bandwidth hurts,
  drop to q50/320×240 via the 2.1 config endpoint rather than changing
  protocol. WebRTC would halve latency but adds a signaling server and a
  heavy dependency — explicitly out of scope until MJPEG-over-tailnet is
  proven insufficient.
- **Safety:** heartbeat dead-man (2.3) becomes mandatory in remote mode;
  ultrasonic + watchdog already cover link loss.
- **Test:** phone on cellular, Pi on home WiFi: full start/stop/video/drive
  session; measure video latency and log dropouts.

---

## Suggested order

1. **2.1 config + live tuning** and **4.1 committed test suite + CI** (both
   unblock everything else and protect it),
2. **2.3 heartbeat dead-man**, **2.5 camera-failure handling**, **1.1
   battery telemetry**, **1.2 LED/buzzer feedback** (small, high-value,
   mostly independent),
3. **4.3 imperfection modeling** before first live runs (tune robustness
   cheaply), **2.4 run logging** alongside first live runs,
4. **2.2 lap counting**, **3.1 joystick**, **3.2 reconnect**, **4.2 map
   view** as polish,
5. **1.3 IRQ ultrasonic**, **1.4 pan servo**, **4.4 follower-in-sim**,
   **5 internet remote** after the core loop is proven on hardware.
