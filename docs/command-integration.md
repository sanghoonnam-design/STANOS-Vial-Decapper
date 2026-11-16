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
| GSENSOR | CT,ZHigh,ZLow,YForward,YBack,CapOpen,CapClose,BodyOpen,BodyClose (raw bits) |
| LR count | LR acknowledgement; count 1..1000000, completes after the final full CAP cycle |
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
| E2800 | Motor stop or arrival confirmation timed out |
| E2810 | Z/R target or observed position is outside its limits, or the limit settings are invalid |
| E2820 | Decapper state fault |
| E2850 | Z Home input did not activate during HOME search |
| E2851 | Z Home input did not release during HOME backoff |
| E2860 | Y Forward limit was not reached |
| E2861 | Y Back limit was not reached |
| E2870 | Cap release did not confirm Open=1 and Close=0 |
| E2880 | Body hold did not confirm Open=0 and Close=0 |

For E2870, GERD can return LR/CAP step=9 completed=123 - Cap release not confirmed (Open=0 Close=0 expected 1/0).
GSENSOR reports raw inputs. The USB SL display includes the latched Z H/L, Y F/B,
Cap Open/Close and Body Open/Close combination diagnostics. Those flags are
diagnostic only. DECAP Cap engagement uses its existing minimum dwell. Check the first GSENSOR field for Vial detection at Y Forward.
After checking the cause, clear the fault with CLR before requesting motion again.

## Pneumatic commands and outputs

| Function | DO order | Command and output pattern |
|---|---|---|
| Y axis | DO 1, DO 2 | `READY 0` Forward = `10`, `READY 1` Back = `01`, Y stop = `00` |
| Cap gripper | DO 3, DO 4 | `CGRIP 1` grips with `10`, `CGRIP 0` releases with `01` |
| Body gripper | DO 7, DO 8 | `BGRIP 1` grips with `10`, `BGRIP 0` releases with `01` |

In each pair, `1` means the logical output is ON. The gripper `00` pattern
is reserved in the firmware output helper. The gripper commands use only
grip and release, and `STOP` preserves gripper outputs while turning off Y.
`DO` writes individual output channels directly. Use the pneumatic commands
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
- Move R with an absolute teaching move from zero. `RPOS R` captures that signed coordinate as a pending RDecapPos target. Negative coordinates remain negative.
- Plain `RPOS` captures both coordinates and replies Z,R. Use `SAVE Z` or `SAVE R` to write the selected target. `RPOS Z` / `RPOS R` capture only the requested target.
- `SAVE Z` and `SAVE R` validate and save only the selected target. A failed axis save leaves both RAM values and candidates available for retry.
- `SAVE 1` ignores pending snapshots and saves current RAM, preserving the existing explicit API contract. A successful axis save consumes its own candidate. Successful SAVE 1, explicit TEACH/PL, LOAD and accepted HOME cancel both candidates.
- Automatic DECAP, unit UDECAP and the DECAP half of LR target the signed absolute RDecapPos. CAP/UCAP and the CAP half of LR target R=0. Repeated identical operations do not accumulate R movement.
- Examples: taught +3200 → 0, +3200, 0; taught -3200 → 0, -3200, 0. RDecapPos is now an absolute DECAP coordinate, replacing its previous relative-distance meaning; re-teach its direction before physical operation.
- RPOS directly reads the requested current positions and records them as SAVE candidates, including during motion and before HOME. Invalid arguments preserve earlier candidates. SAVE requires idle and validates the captured targets before applying them.
- Body gripper outputs use physical DO7/DO8: BGRIP 1 = 10, BGRIP 0 = 01, internal standby = 00. Output indices in C are zero-based 6/7. Cap remains DO3/DO4 and Y remains DO1/DO2.

### Paired capture and per-axis persistence

After HOME and completion of teaching moves, send `RPOS` without arguments. For Z=12000 and R=-3200 its reply body is `12000,-3200` (Z,R order). Send `SAVE Z` to persist 12000 as ZCap_UpPos, or `SAVE R` to persist -3200 as RDecapPos. A save uses the capture even if the axis has moved since RPOS. Axis save requires idle and validates the selected coordinate against the active and persisted limits. The EEPROM image is based on the latest verified stored record, so unrelated RAM edits are excluded. The selected RAM target changes after verified storage. NO_RPOS indicates a missing capture, RANGE indicates the active limits reject it, and FAIL indicates the persisted limits or write verification failed. The unselected candidate remains available.
