# Decapper command API schema 1

## Connection and execution
Python selects one TCP or UART route. Each route has its own state, response queue,
timeout and position cache. Connecting sends VERSION, GSTA, SPEED, then PCONFIG.
PCONFIG replies with `1,<Decapper PL base index>`; software uses that base rather
than assuming index 35. An unsupported reply or capability timeout falls back to
legacy queries. Extended teaching, parameter editing and finite LR require schema 1.

CR and LF are checked initially. Each selected terminator is appended to every
command, including handshake and queries. TCP and RS232C commands require CRLF.
TCP/RS232C frames accept at most 253 command bytes plus CRLF; longer frames are
discarded without executing a truncated prefix. USB CLI reserves terminator space
in its existing 50-byte buffer (47 command bytes); either Enter form becomes CRLF.

A motion response acknowledges registration, not mechanical completion. GSTATE
operation/result correlates acceptance and completion. Three-second response timeouts
never resend motion. STOP preempts a pending request. State older than three seconds,
unknown outcomes, Busy and faults block new motion. There is no automatic fault clear,
SAVE, reconnect replay, or transport fallback.

Legacy GSTA has no operation identity. A fast motion may finish between samples;
when completion cannot be established, the UI explicitly asks for STOP then a fresh
state query. It never silently assumes completion or retries the motion. Updating
firmware to schema 1 enables normal completion correlation.

## Commands
All replies start with the received command name and end in CRLF.
Failures use INVALID, UNSUPPORTED, RANGE, BUSY, NO_RPOS, FAIL or an E#### code.

| Request | Reply body / behavior |
|---|---|
| PCONFIG | 1,base |
| GPOS Z / GPOS R | axis,pulse; pure read, never changes the legacy teaching candidate |
| RPOS | Z,R current pulse snapshots; captures both pending targets without changing PL |
| RPOS Z / RPOS R | single-axis pulse snapshot; captures only the requested target |
| SAVE Z | Persist the captured Z position as ZCap_UpPos only |
| SAVE R | Persist the captured signed R position as RDecapPos only |
| SAVE / SAVE Z,R | INVALID. Select one axis with SAVE Z or SAVE R |
| PGET index | index,value from current RAM; float values retain 32-bit precision |
| TEACH index,pulse | OK after changing a whitelisted RAM target; validates idle state and limits |
| SAVE 1 | OK / FAIL after backup/main EEPROM CRC and exact readback; ignores legacy RPOS candidate |
| LOAD | OK HOME_REQUIRED / FAIL; validated settings apply to RAM/drivers under maintenance lock |
| GSTA / ST | busy,enable,homed,error,error_code,isGripCap,isGripBody. The first five fields are unchanged. |
| GSTATE | busy,homed,error,paused,phase,idle,error_code,operation_id,result |
| GERR / ERR | Current error code. A latched Decapper fault takes precedence |
| GERD / ERD | First Decapper fault detail with operation, step and condition. LR includes the CAP/DECAP half and completed count |
| CLR / CLEAR | Clear the fault once idle |
| GSENSOR | CT,ZHigh(Z+ lower, pin 1),ZLow(Z- upper/HOME, pin 2),YForward,YBack,CapOpen,CapClose,BodyOpen,BodyClose (display: limits 1 = reached, Open/Close/CT 1 = detected; raw pins via DI) |
| LR count | LR acknowledgement; count 1..1000000, completes after the final full CAP cycle. FAIL before HOME (homed=0); nothing is registered |
| GLR | completed_count,limit,cap_decap_phase,paused,busy |
| SL | STATE=<GSTATE fields>;SENSOR=<GSENSOR fields> |
| CD | TARGET_Z=value;TARGET_R=value;SPEED=value;LR=<GLR fields> |
| PL | All 22 Decapper index,value pairs separated by semicolons |
| PL index,value | OK, then Python requests PGET index to read back RAM |

GSTATE result: 0 none, 1 accepted/running, 2 completed, 3 stopped, 4 fault.
Operation id is incremented at request registration. Phase numbers are the firmware
FSM enums, not percentages. Sensor bits are electrical observations, not inverted
logical ON values. GLR counts complete DECAP/CAP pairs. Limit 0 means legacy unlimited LR.
USB CLI keeps its detailed SL/CD/PL console output and also returns the summary.

## Decapper motion faults

| Code | Meaning |
|---|---|
| E2800 | Motor stop or arrival confirmation timed out. Z/R move windows are max(5 s, 2 x (distance/speed + speed/accel) + 1 s) at the scaled speed, so slower SPEED or longer travel does not trip it |
| E2810 | Z/R target or observed position is outside its limits, or the limit settings are invalid |
| E2820 | Decapper state fault. LR also stops with this code when the CT vial sensor stays off for CT_DETECT_WAIT_MS (3 s, 2026-11-20) at the start of a DECAP/CAP half (GERD: LR/DECAP step=0 ... - CT vial not detected) |
| E2850 | Z Home input did not activate during HOME search. The fast-search window covers Limit_PosZ at the HOME speed (1.25 x + 1 s, at least 5 s) |
| E2851 | Z Home input did not release during HOME backoff, or is still active at the HOME offset position (HOME step=17) |
| E2860 | Y Forward limit was not reached |
| E2861 | Y Back limit was not reached |
| E2870 | Cap release did not confirm Open=1 and Close=0 |
| E2871 | Cap gripper Open and Close both active (1/1) for 50 ms (idle or running): sensor fault, stops all motion; re-raised after CLR while still 1/1 |
| E2880 | Body hold did not confirm Open=0 and Close=0 |
| E2881 | Body gripper Open and Close both active (1/1) for 50 ms (idle or running): sensor fault, stops all motion; re-raised after CLR while still 1/1 |
| E2890 | Homed Z move blocked or stopped by the active High = Z+ (lower, pin 1) input (either direction) |
| E2891 | Homed Z move blocked or stopped by the active Low = Z- (upper HOME switch, pin 2) input (either direction) |
| E2892 | Z High and Low inputs are both active during a homed Z move |

Z limit raw inputs are active-high (Z_LIMIT_ACTIVE_LEVEL=1); internally the logical value stays 0 = reached,
but GSENSOR and GERD show limits as 1 = reached (2026-11-20).
Z limits are checked only after HOME has completed (isHomed=1, 2026-11-19). Before that,
including the whole HOME sequence, limit inputs never fault and Z moves are not blocked.
Once homed, Z moves check the limits before launch and during travel, and a single active
switch faults regardless of travel direction; recover with CLR and then HOME. HOME
searches in the Z- direction (upward) for the Low = IO list Z- input on pin 2 (2026-11-20: Z pins follow the IO list,
High = Z+ = pin 1 lower, Low = Z- = pin 2 upper HOME switch; wiring and motion unchanged),
finds the Low (HOME) activation point with a slow re-approach, labels it -1000 pulses
(Z_HOME_OFFSET_PULSE, about 0.31 mm at 3200 pulse/mm), moves down (+) to Z=0 at the
slow HOME speed (HOME steps 16/17) and requires Low (HOME) to be released there
(otherwise E2851). Z=0 therefore sits below the switch and normal operation
never touches it: after HOME any Z move while the Low (upper HOME) input is
active faults with E2891, including the return to Z=0 (the former 800-pulse
home-zone exemption is removed). During HOME, the search steps use High as
their normal stop input; Low is not checked and only the step timeouts apply. A limit
fault stops Z/R and turns off both Y outputs. GERD includes the active input
levels and the operation phase.

For E2870, GERD can return LR/CAP step=9 completed=123 - Cap release not confirmed (Open=0 Close=0 expected 1/0).
GSENSOR reports logical sensor values, not raw pins (2026-11-19): each input is
compared with its active level in CDecap.h (Z_LIMIT_ACTIVE_LEVEL=1,
Y_LIMIT_ACTIVE_LEVEL=0, GRIP_SENSOR_ACTIVE_LEVEL=0, CT_DETECT_ACTIVE_LEVEL=1), so
limits are 0 = reached internally and Cap/Body Open/Close and CT are 1 = detected.
Since 2026-11-20 GSENSOR and the GERD sensor details invert the Z H/L and Y F/B
limits for display, so every GSENSOR field reads 1 = detected (e.g. Y at Forward
reports YForward=1,YBack=0; GERD prints H/L/F/B the same way). Use DI
for raw pins. The USB SL display includes the latched Z H/L, Y F/B,
Cap Open/Close and Body Open/Close combination diagnostics. Those flags are
diagnostic only. DECAP Cap engagement uses its existing minimum dwell. Check the first GSENSOR field for Vial detection at Y Forward.
After checking the cause, clear the fault with CLR before requesting motion again.

## Pneumatic commands and outputs

| Function | DO order | Command and output pattern |
|---|---|---|
| Y axis | DO 1, DO 2 | `READY 0` Forward = `10`, `READY 1` Back = `01`, Y stop = `00` |
| Body gripper | DO 3, DO 4 | `BGRIP 1` grips with `10`, `BGRIP 0` releases with `01` |
| Cap gripper | DO 5, DO 6 | `CGRIP 1` grips with `10`, `CGRIP 0` releases with `01` |

In each pair, `1` means the logical output is ON. The gripper `00` pattern
is reserved in the firmware output helper. The gripper commands use only
grip and release, and `STOP` preserves gripper outputs while turning off Y.
`DO` writes individual output channels directly and replies BUSY without touching
any output while the Decapper is not idle (running, paused LR or maintenance); it
stays available in a stopped fault state. Use the pneumatic commands
to control the paired outputs through their normal sequence.

## Decapper parameter offsets from advertised base

| Offset | Field / unit |
|---|---|
| 0,1 | Z/R running current (A, float) |
| 2,3 | Z/R maximum current code: 0=3.1 A range, 1=1.7 A range |
| 4,5 | Z/R hold current percentage, 0..100 |
| 6 | Resolution code 0..8: 51200,25600,12800,6400,3200,1600,800,400,200 pulses/revolution |
| 7,8 | Z/R basic position limits (pulse) |
| 9,10 | Z negative/positive software limits (pulse) |
| 11,12 | R negative/positive software limits (pulse) |
| 13 | Software limits enabled, 0/1 |
| 14,15 | Z acceleration / velocity |
| 16,17,18 | Z cap upper / side / origin position (pulse) |
| 19,20 | R acceleration / velocity |
| 21 | R signed absolute DECAP target (pulse); CAP=0 |

TEACH permits offsets 16,17,18,21 only. The editor exposes the 22 actual Decapper
fields; unrelated legacy PL slots that do not change RAM are not offered.
Resolution and current-selection codes are checked in both UI and firmware.
Position preflight uses current RAM basic limits intersected with enabled software
limits. REL MOVE also requires a recently read position to calculate the endpoint;
firmware remains the final validator.

Z coordinate direction (2026-11-20): Z coordinates are positive downward, matching the
other projects and the IO list (Z- = upper HOME switch on pin 2, Z+ = lower switch on pin 1).
HOME searches toward Z-, Z=0 is Z_HOME_OFFSET_PULSE below the switch, and the cap upper/side
positions (offsets 16/17) and Z software limits are positive below Z=0 (factory defaults
12000/24000). GPOS Z, RPOS Z, MOVE A/R Z, TEACH and PL use this convention. The motor wiring
and physical motion are unchanged: TMC429 inv_dir is shared by all channels, so only the Z
coordinate sign is converted in firmware (Z_COORD_SIGN = -1 in CDecap.h). Values taught
before this change must be re-entered with the opposite sign.

CAP/DECAP Z+R synchronized move (2026-11-20): while Z travels between the cap upper
and side positions, R travels to RDecapPos (DECAP) or 0 (CAP), e.g. 4 turns =
4 x 230400 = 921600 pulse (one R output turn measured as 230400 pulse on 2026-11-20). For that one move the firmware keeps R acceleration (offset 19)
and derives Z velocity/acceleration so both axes finish together; R velocity is trimmed
down by up to one Z velocity step (about 122 pulse/s of Z) to absorb rounding. Z is never faster
than offsets 14/15 and R never faster than offset 20, so set the cycle time with offset 20.
Z approach/return and manual MOVE still use offsets 14/15 unchanged. TMC429 units
(16 MHz, PULSE_DIV=1, RAMP_DIV=6): velocity step 122 pulse/s, acceleration step
3725 pulse/s^2; AMAX below 9 (about 33,500 pulse/s^2) has no valid PMUL/PDIV pair,
so the synchronized Z acceleration is at least 9 steps. Keep SPEED 100 for the
finest timing; other SPEED values stay synchronized within about 0.3 %.

Applying a parameter changes RAM. EEPROM Save confirms saving the whole configuration.
Current/resolution changes may require SAVE 1 followed by LOAD or reboot and a new HOME.
LOAD reinitializes driver coordinates, so the previous home reference is invalid.
Plain RPOS captures the current Z and R coordinates and returns Z,R, including during motion. RPOS Z and
RPOS R still capture individual targets. SAVE Z and SAVE R persist the corresponding captured target. The settings UI uses GPOS for reads and SAVE 1 for an explicit whole-RAM save.

## Screen and verification
Test / Teaching / Debug / Status names are preserved. Status contains only SL/CD/PL.
Debug offers separate Longrun buttons: Unlimited Start sends plain LR and works
with legacy firmware; Count Start sends LR count and requires schema 1. STOP ends
either mode. Completed count is read from GLR on schema-1 firmware, increasing
once per finished DECAP/CAP pair; legacy firmware still counts internally but cannot
provide the new GLR display.
The upper Teaching & Motion panel uses a 60:40 starting ratio with Command,
subject to the page minimum sizes. Motion tabs have no scroll areas. State/F/W/API appear in a small
single line; full state is also available as a tooltip. Minimum window size is 1000x720.

Software tests run with:
`.venv/Scripts/python.exe -m unittest discover -s tests`.

Firmware host tests run with Python and a C compiler:
set ZIG_EXE to Zig 0.13.0, then run every `tests/safety/test_*.py`.
They exercise real parser/handlers/FSM with mocked hardware, including 1000 long-run
cycles, 28 pause/resume phase cases, serial framing and EEPROM failure paths.
STM32 ARM compile/link is separately checked. These are offline tests: mechanical
motion, live sensor polarity, real EEPROM persistence and flashing require equipment validation.


GSTA grip flags are 0/1. isGripCap uses both Cap sensors off. isGripBody records the last CT detection at Y Forward. A reading at Back or between limits does not change the last Vial state. READY 0 is Forward and READY 1 is Back.

## R-axis teaching and absolute endpoints (2026-11-10)

- HOME re-runs the Z/Y homing sequence. It stops R and defines its stationary position as software R=0; it does not search for an R home sensor.
- HOME first releases both the Cap and Body grippers and waits 1.5 s (HOME_GRIP_RELEASE_WAIT_MS, HOME step 18) before Z moves (2026-11-20), so a vial or cap still gripped after a power cycle is not lifted by Z or carried by Y. The wait is time-based only: no gripper sensor is checked and it has no fault code.
- Move R with an absolute teaching move from zero. `RPOS R` captures that signed coordinate as a pending RDecapPos target. Negative coordinates remain negative.
- Plain `RPOS` captures both coordinates and replies Z,R. Use `SAVE Z` or `SAVE R` to write the selected target. `RPOS Z` / `RPOS R` capture only the requested target.
- `SAVE Z` and `SAVE R` validate and save only the selected target. A failed axis save leaves both RAM values and candidates available for retry.
- `SAVE 1` ignores pending snapshots and saves current RAM, preserving the existing explicit API contract. A successful axis save consumes its own candidate. Successful SAVE 1, explicit TEACH/PL, LOAD and accepted HOME cancel both candidates.
- Automatic DECAP/CAP order: CT check, Body release check (step 14; normally immediate, it only waits when a fault left the Body held), Y Back (pull the vial in, step 2), Body grip (step 1) and hold confirmation (Open=0/Close=0 in the Ready step, E2880), Z/R unscrew or screw, Z return, Body release (step 13), Y Forward (step 11). A Body release proceeds when Open=1/Close=0 is seen or after 1 s; it has no fault code. ORG follows the same rule: Z to 0, Body release (ORG step 5), then Y Forward, so a Body left held by a fault is opened before Y moves. HOME does the same after Z is zeroed (HOME step 15, Body release, then Y Forward).
- Motor current: a move raises the axis to the run current. An axis is lowered to the hold current only after its standstill is confirmed (TMC2660 StandStill and TMC429 actual velocity 0) while the FSM is idle (normal completion, STOP, fault stop) or paused; STOP/PAUSE no longer drop the current during deceleration, and short stops between steps of a running sequence keep the run current.
- HOME drives Z with the run current and returns it to the hold current once Z standstill is confirmed. Its acceleration is converted to TMC429 units the same way as normal moves (pulse/s2 / 3725, clamped 1..2047); it is no longer passed raw. Y never moves while the Body grip output is on, and Z/R only move while Y is at Back with the Body held.
- HOME Z reference (2026-11-19): Z=0 is no longer the High detection point but Z_HOME_OFFSET_PULSE (1000 pulses) below it. Z coordinates taught against the old reference are now about that much lower physically, so Z positions must be re-taught after this firmware change. If the High input is still active at Z=0 (switch hysteresis larger than the offset) HOME fails with E2851 at step 17 and stays not homed; increase Z_HOME_OFFSET_PULSE in that case.
- LR alternates the full automatic DECAP and CAP sequences above, the same as standalone DECAP/CAP. After each half completes, LR rests 5 s (LR_INTERVAL_MS, 1 s before 2026-11-20) with no output or motor change before the next half; PAUSE time is excluded and the final CAP of `LR count` ends without the rest. During the rest GSTATE stays busy in the LONGRUN phase. At the start of each half LR then waits up to 3 s (CT_DETECT_WAIT_MS, PAUSE time excluded) for the CT vial sensor instead of reading it once, because Y Forward can report before the vial has settled in front of CT; it continues as soon as CT turns on and stops with E2820 only if CT stays off for the whole wait (2026-11-20).
- Unsolicited LR progress: each completed DECAP/CAP cycle pushes `LRCOUNT n` + CRLF (n = completed count, the same value as GLR completed_count) to the connected app TCP socket, and prints it on the USB console. It is not a reply to any request, so clients must ignore it while matching responses by command name; if no app socket is connected the notification is dropped (GLR still has the count).
- CAP (CAP, UCAP and the CAP half of LR) waits 1 s (CAP_UP_DWELL_MS, step 15) after Z reaches ZCap_UpPos before starting the R rotation and Z descent. UDECAP/UCAP remain the Z/R/Cap-gripper-only unit tests. A standalone DECAP/CAP without a CT vial ends without motion; LR instead stops with E2820 so it never counts empty cycles.
- DECAP, CAP, UDECAP, UCAP and LR require HOME (homed=1). Before HOME they reply FAIL and register no operation (operation_id unchanged); HOME and manual moves stay available.
- Manual `MOVE` (2026-11-19) uses the per-axis PL speed and acceleration: Z uses ZDecapVel/ZDecapAcc (PL base+15/+14) and R uses RDecapVel/RDecapAcc (PL base+20/+19), the same values as automatic DECAP/CAP/LR. `SPEED` percent still scales the velocity. HOME keeps its fixed MOTOR_SPD_HOME_* speeds.
- Automatic DECAP and CAP pull the vial to Y Back, wait 1 s after the Y Back input is detected (Y_BACK_SETTLE_MS, step 16) and only then grip the Body.
- Automatic DECAP, unit UDECAP and the DECAP half of LR target the signed absolute RDecapPos. CAP/UCAP and the CAP half of LR target R=0. Repeated identical operations do not accumulate R movement.
- Examples: taught +3200 → 0, +3200, 0; taught -3200 → 0, -3200, 0. RDecapPos is now an absolute DECAP coordinate, replacing its previous relative-distance meaning; re-teach its direction before physical operation.
- RPOS directly reads the requested current positions and records them as SAVE candidates, including during motion and before HOME. Invalid arguments preserve earlier candidates. SAVE requires idle and validates the captured targets before applying them.
- STANOS wiring: Body gripper outputs use physical DO3/DO4 (zero-based 2/3 in C) and Cap gripper outputs use DO5/DO6 (zero-based 4/5): BGRIP 1 = 10, BGRIP 0 = 01, internal standby = 00. Y remains DO1/DO2. The standard Decapper uses Cap DO3/DO4 and Body DO7/DO8.

### Paired capture and per-axis persistence

After HOME and completion of teaching moves, send `RPOS` without arguments. For Z=12000 and R=-3200 its reply body is `12000,-3200` (Z,R order). Send `SAVE Z` to persist 12000 as ZCap_UpPos, or `SAVE R` to persist -3200 as RDecapPos. A save uses the capture even if the axis has moved since RPOS. Axis save requires idle and validates the selected coordinate against the active and persisted limits. The EEPROM image is based on the latest verified stored record, so unrelated RAM edits are excluded. The selected RAM target changes after verified storage. NO_RPOS indicates a missing capture, RANGE indicates the active limits reject it, and FAIL indicates the persisted limits or write verification failed. The unselected candidate remains available.
