# PRD — Motor Control Stack (ROS 2 Jazzy, Ubuntu 24.04, Raspberry Pi 4, SOLO Mini v2 over USB)

**Version:** v1.0  
**Date:** 2025-10-22  
**Owner:** Software Architecture

---

## 0) Scope & Goals

**Goals**
- Drive 1–2 motors via SOLO Mini v2 over USB (default `/dev/solo_mc_1`, configurable).
- Control modes: **speed**, **torque**, **position**.
- Expose all available telemetry and configuration supported by the SOLO public API.
- Provide TUIs for bring-up and testing (direct USB and ROS 2).
- Provide a ROS 2 node wrapping the controller; later combine two for tracked-vehicle control.

**Out of Scope (initial)**
- Advanced motion planning/trajectories
- CAN interface (USB-first; possible future change)

---

## 1) Hardware & Platform

- **Compute:** Raspberry Pi 4 (4–8 GB)
- **OS:** Ubuntu 24.04 LTS (64-bit)
- **ROS 2:** Jazzy
- **Motor Driver:** SOLO Mini v2 (USB)
- **Powertrain:** BLDC or Brushed (per SOLO support)
- **Wiring:** Per SOLO datasheet; encoder/hall as required by motor

---

## 2) External Dependencies

- ROS: `rclcpp`, `diagnostic_updater`, `diagnostic_msgs`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
- Optional future: `ros2_control`, `hardware_interface`
- Serial/USB: `Boost.Asio` (or `libserial` alternative)
- TUIs: `ncurses` (C++)

---

## 3) Datasheets & Protocol (Required Artifacts)

Place in `docs/datasheets/` or request from vendor if missing:
- SOLO Mini v2 User Manual and Register/Command Reference (USB/UART protocol)
- Electrical specs (limits, thermals)
- Motor datasheet (Kv, pole pairs, limits)
- Encoder/Hall datasheets (CPR, electrical)
- Power supply datasheet

If unavailable: add `docs/REQUESTS.md` with vendor request details.

---

## 4) Architecture Overview

**Packages**
- `solo_driver`: USB transport + protocol
- `solo_ros`: ROS node, messages, services, params
- `solo_tools`: TUIs (`solo_tui_direct`, `solo_tui_ros`)
- `tracked_control`: `/cmd_vel` mixer + odom
- `solo_bringup`: configs + launch
- `docs`: PRD, wiring/safety, changelog

**Stages**
1. `solo_tui_direct`: direct USB TUI (bring-up)
2. `solo_node`: ROS 2 node (topics/services)
3. `solo_tui_ros`: TUI via ROS
4. Dual controllers (left/right)
5. `tracked_control_node`: tracked vehicle mixing + odometry

---

## 5) ROS 2 Interfaces

**Topics (per motor, namespaced by `motor_id`)**
- Commands:
  - `/{motor_id}/command_speed` — `std_msgs/Float32` (rad/s)
  - `/{motor_id}/command_torque` — `std_msgs/Float32` (N·m)
  - `/{motor_id}/command_position` — `std_msgs/Float32` (rad)
  - `/{motor_id}/command_stop` — `std_msgs/Empty`
- Telemetry:
  - `/{motor_id}/state` — `sensor_msgs/JointState`
  - `/{motor_id}/driver_status` — `solo_ros/DriverStatus` (custom)
  - `/{motor_id}/diagnostics` — `diagnostic_msgs/DiagnosticArray`
  - `/{motor_id}/raw_registers` — `solo_ros/RegisterDump` (optional)

**Services (per motor)**
- `/{motor_id}/set_mode` — `solo_ros/SetMode` (`speed|torque|position`)
- `/{motor_id}/enable` — `std_srvs/SetBool`
- `/{motor_id}/home` — `std_srvs/Trigger`
- `/{motor_id}/clear_faults` — `std_srvs/Trigger`
- `/{motor_id}/set_limits` — `solo_ros/SetLimits`
- `/{motor_id}/set_pid` — `solo_ros/SetPID`
- `/{motor_id}/save_to_flash` — `std_srvs/Trigger`
- `/{motor_id}/load_defaults` — `std_srvs/Trigger`

**Vehicle (tracked)**
- Input: `/cmd_vel` — `geometry_msgs/Twist`
- Output: left/right command topics (usually speed)

---

## 6) Configuration (YAML)

All nodes read parameters from YAML. Example fragments:

```yaml
solo_node:
  ros__parameters:
    motor_id: "left"
    device: "/dev/solo_mc_1"
    serial: { baudrate: 115200, timeout_ms: 50 }
    control:
      default_mode: "speed"
      ramp_rate: 50.0
      jerk_limit: 0.0
      position_relative: false
    limits:
      max_speed_rad_s: 150.0
      max_torque_nm: 2.5
      max_current_a: 20.0
      dc_bus_volt_min: 12.0
      dc_bus_volt_max: 54.0
    motor:
      type: "bldc"
      pole_pairs: 7
      kv_rpm_per_volt: 400.0
      has_halls: true
      encoder_cpr: 2048
      gear_ratio: 1.0
    pid:
      speed:   {p: 0.2, i: 0.01, d: 0.0, ff: 0.0}
      torque:  {p: 0.1, i: 0.00, d: 0.0, ff: 0.0}
      position:{p: 2.0, i: 0.00, d: 0.0, ff: 0.0}
    publish: { rate_hz: 50.0, diagnostics_hz: 1.0 }
    safety:
      enable_watchdog: true
      watchdog_timeout_s: 0.25
      estop_topic: "/estop"
      stop_on_fault: true
      soft_stop_ramp_s: 0.2
```

Tracked vehicle parameters:

```yaml
tracked_control_node:
  ros__parameters:
    left_motor_namespace: "/left"
    right_motor_namespace: "/right"
    mode: "speed"
    kinematics:
      track_width_m: 0.45
      wheel_radius_m: 0.085
      gear_ratio: 1.0
    limiters:
      max_linear_mps: 0.8
      max_angular_rps: 1.8
      accel_linear_mps2: 1.0
      accel_angular_rps2: 2.5
    mixing:
      open_loop: false
      deadband: 0.01
```

---

## 7) TUI Applications

**`solo_tui_direct` (Stage 1)**
- Direct USB control; mode/select; setpoint sliders/keys; CSV logging; fault banners; E-STOP.

**`solo_tui_ros` (Stage 3)**
- Publishes to/reads from ROS; multi-motor namespace selector; `/cmd_vel` view.

---

## 8) Node Design — `solo_node`

- Threads: IO (non-blocking serial), ROS executor, optional logger
- State machine: INIT → CONNECT → IDLE → ENABLED → FAULT → DISABLED
- Watchdog: ramps to zero if commands stop (timeout configurable)
- Diagnostics: `diagnostic_updater` (voltage, thermal, comm status, faults)
- Poll rates: 50 Hz telemetry (configurable), bulk reads where supported

Optional future: `ros2_control` hardware interface.

---

## 9) Tracked Vehicle — `tracked_control_node`

- Converts `/cmd_vel` to left/right speed setpoints (kinematics, accel limiting, deadband)
- Subscribes to JointState, publishes odometry (`nav_msgs/Odometry`) and optional TF
- Safety: if either motor faults, commands drop to zero and diagnostic ERROR is raised

---

## 10) Safety & Compliance

- E-Stop topic; optional GPIO (future)
- Enforce current/torque/speed/thermal limits in software in addition to SOLO protections
- Power stage remains disabled until explicit `enable`
- Flash writes only on explicit request
- Log all state transitions and faults
- Wiring & EMC/EMI guidance in `docs/safety_and_wiring.md`

---

## 11) Launch Files

- `single.launch.py`: one `solo_node` with `motor1.yaml`
- `dual.launch.py`: two `solo_node` instances + `tracked_control_node`

---

## 12) Telemetry & Logging

- ROS bags via `rosbag2`
- Optional CSV (timestamp, setpoint, actual speed, current, voltage, temps, fault)
- Counters for comm resets/timeouts

---

## 13) Testing

**Unit/Integration**
- Protocol framing/parsing; fault mapping; parameter validation; watchdog; limits

**HIL**
- Spin tests; thermal/current derating; fault injection (safe)

**Vehicle**
- Straight, reverse, in-place spin, radius turns; odometry drift characterization

---

## 14) Performance Targets

- Command latency (ROS→device): < 20 ms typical @ 50 Hz
- State publish jitter: < 5 ms RMS @ 50 Hz
- Watchdog cutoff: ≤ timeout + 20 ms
- Odometry error (short runs): ≤ 2% distance at low speed

---

## 15) Deliverables

- Source code packages, README, configs, launch
- `docs/PRD.md`, `docs/CHANGELOG.md`, `docs/REQUESTS.md`, `docs/safety_and_wiring.md`
- udev rules for stable `/dev/solo_mc_*` names

---

## 16) Assumptions & Open Items

- SOLO USB register map & units; bulk read support; rate limits
- Encoder semantics (units, zeroing/homing)
- Torque command native vs current mapping (motor constants)
- Thermal sensors (which, units)
- Flash endurance guidance
- Licensing constraints on bundling PDFs

---

## 17) Milestones

1. Stage 1 — `solo_tui_direct`
2. Stage 2 — `solo_node`
3. Stage 3 — `solo_tui_ros`
4. Stage 4 — Dual motor
5. Stage 5 — `tracked_control_node`

**Exit Criteria** defined in main PRD body for each stage.

---

## 18) Change Log (snippet)

- v1.0 — Initial PRD for USB-first SOLO Mini v2, single→dual, tracked control.
