# AGENTS.md — PX4-Autopilot

Open-source flight controller (C/C++, NuttX/POSIX/QURT). Docs: https://docs.px4.io

## Build & Test

```bash
make px4_sitl_default          # SITL build
make px4_fmu-v6x_default       # Hardware build
make px4_sitl_default test     # Unit tests (GTest)
make tests TESTFILTER=<name>   # Single test
make distclean                 # Clean rebuild
```

## Code Style

Enforced by AStyle (`Tools/astyle/fix_code_style.sh <file>`) and clang-tidy (warnings = errors).

| Rule | Value |
|------|-------|
| Indentation | Hard tabs, 8-space width |
| Style | Linux / K&R |
| Max line length | 140 chars |
| Pointer alignment | To name: `int *ptr` |
| Braces | Always required |
| C++ / C standard | C++14 / C11 |
| Source files | `*.cpp` (not `*.cc`) |

**Naming:** `lowerCamelCase()` functions, `ClassConstructors()`, `_private_member`, `kConstant`, zero-indent `public:`/`private:`/`protected:`.

## Directory Structure

| Path | Contents |
|------|----------|
| `src/modules/` | Flight control modules (commander, ekf2, mc_*_control, fw_*, …) |
| `src/drivers/` | Hardware drivers (IMU, GPS, baro, …) |
| `src/lib/` | Shared libraries (mathlib, geo, crypto) |
| `msg/*.msg` | uORB message definitions — changing a msg affects every subscriber |
| `boards/` | Board configs (100+ boards) |
| `ROMFS/` | Read-only filesystem & startup scripts |
| `platforms/` | OS abstraction (nuttx, posix, qurt) |
| `test/` | GTest unit tests, MAVSDK integration tests |

Modules register via `px4_add_module()` and communicate through **uORB** pub/sub.

## Commit Messages

Format: `scope: description` — imperative mood, lowercase, `git commit -s` required.

```
ekf2: add GNSS yaw fusion timeout
docs: clarify rover offboard support
boards: remove unused IMU sensors from 6X
fix commander: prevent arming without GPS lock (#26100)
```

Branch naming: `<user>/<description>` off `main`.

## Simulation

### SIH (headless, no GPU)

```bash
make px4_sitl sihsim_quadx             # quad
make px4_sitl sihsim_airplane           # fixed-wing
make px4_sitl sihsim_standard_vtol      # VTOL
PX4_SIM_SPEED_FACTOR=10 make px4_sitl sihsim_quadx  # 10× speed
```

### Gazebo

```bash
make px4_sitl gz_x500
```

### MAVLink Ports

| Port | Purpose |
|------|---------|
| 14550 | GCS (QGroundControl) |
| 14540 | Offboard API (MAVSDK) |
| 4560 | Simulator interface |

## Safety Notes

**This is safety-critical flight controller software.**

### Elevated Care Zones

| Directory | Controls | Before modifying |
|-----------|----------|------------------|
| `src/modules/commander/` | Arming, failsafe, mode transitions | Verify failsafe in SITL for all vehicle types |
| `src/modules/ekf2/` | State estimation | Run EKF replay tests |
| `src/modules/mc_*_control/` | Multicopter controllers | Test in SIH; verify all MC airframes |
| `src/modules/fw_*/` | Fixed-wing controllers | Test FW airframes in SITL |
| `src/modules/vtol_att_control/` | VTOL transitions | Test MC + FW modes and transitions |
| `msg/*.msg` | uORB schemas | Check all publishers/subscribers |
| `ROMFS/px4fmu_common/init.d/` | Startup & default params | Can change behavior on all boards |

### Rules

- Never bypass safety checks (arming, geofence, failsafe) without justification
- Document parameter changes — they affect flight behavior
- Specify units in comments for physical quantities
- No magic numbers — use named `constexpr` constants

## Agent Decision Framework

**Do without asking:** doc fixes, CI/build fixes, formatting, adding tests.

**Ask first:** parameter defaults, control algorithms, failsafe logic, uORB schema changes, board configs.

**Stop — do not proceed:** if you can't verify flight safety, if modifying EKF2/controller math without SITL/SIH, if removing safety guards.
