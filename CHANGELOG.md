# Project Change Log

Record every completed repository change here so that code, configuration, and
documentation updates remain traceable.

## Entry Format

Add entries to `Unreleased` before handoff or commit:

`- YYYY-MM-DD | Type | Scope | Summary | Validation`

Use `Added`, `Changed`, `Fixed`, `Removed`, `Build`, or `Docs` for **Type**.
Include the exact validation performed, or `Not run: <reason>` when it is not
available. Move completed entries into a dated release section when a version is
cut.

## Unreleased

- 2026-07-19 | Changed | CPU3 subject-2 VAD and local ASR transport |
  Replaced fixed-duration capture with 10 ms frame VAD, 200 ms pre-roll,
  30 ms start confirmation, 400 ms silence completion, 0.8 to 3.5 second
  variable WAV frames, per-clip DC removal, and bounded PCM gain normalization;
  each capture now opens and closes its own TCP connection and waits for 500 ms
  of quiet audio before rearming. The local ASR service accepts the matching
  16 kHz mono PCM range up to 112,044 bytes and logs capture duration and
  normalized PCM metrics | `subject2_command_test` passed; Python Subject-2
  TCP/protocol suite passed (7 tests, including 0.8/3.5 second boundaries);
  ASR `/health` and TCP port 9001 verified after restart; TASKING standalone
  link was not available because its non-commercial license requires ADS

- 2026-07-19 | Fixed | Subject-2 PCM gain and local credentials | Scaled
  centered 12-bit ADC samples into 16-bit PCM units before bounded AGC, moved
  WiFi credentials to an ignored local header, and ignored rendered schematic
  artifacts | `subject2_command_test` and the Python Subject-2 test suite
  passed; ADS-only final link remains pending

- 2026-07-19 | Changed | CPU3 subject-2 transfer test and local ASR service |
  Moved the 3-second PCM buffer to LMU, aligned the TCP/ASR WAV limits to
  96,044 bytes, retained the last received capture for inspection, and restored
  baseline VAD triggering with command dispatch after a successful ASR response |
  Python TCP tests passed with a 96,044-byte WAV; `subject2_command_test`
  passed; ADS rebuilt the final VAD firmware and confirmed the 96,000-byte LMU
  allocation; hardware VAD triggered and uploaded complete 3-second WAV frames
  from the WiFi-SPI client, but low AN4 AC amplitude caused the ASR to echo the
  command context and return command code 0; analog input validation remains
  required
- 2026-07-19 | Changed | CPU3 subject-2 WiFi configuration | Set the ASR
  endpoint to the active LLJJQQ LAN address and configured local WiFi access |
  Firmware rebuilt; `subject2_command_test` passed; local ASR TCP endpoint
  reachability verified; hardware validation pending
- 2026-07-19 | Changed | TC387 JDS debug launch | Restored JTAG0 to 30 MHz
  after the temporary 5 MHz diagnostic setting; retained the detected JDS
  identifier | AURIX Flasher erased and programmed Flash successfully; target
  reset must be performed manually
- 2026-07-18 | Changed | subject-2 H2 lamp board and CPU3 runtime | Restored
  the Board3 H2 pin map, enabled TLD7002 refresh through CPU3 interrupts, and
  added an STM3 comparator-1 1 ms command/output tick so horn patterns and
  display refresh continue during audio capture and network waits; initialized
  the display backup buffer before its first scan | `subject2_command_test`
  passed; temporary ADS/TASKING Debug build passed with 0 errors and 0 warnings
- 2026-07-18 | Changed | TLD7002 subject-2 glyphs | Replaced the `IJK` and
  `WXY` character slots with the supplied three-panel interior-light and wiper
  bitmaps; retained the prior glyphs and display calls as comments | Source
  glyph check, host syntax check, and `subject2_command_test` passed; AURIX
  hardware build not run because TASKING/ADS is unavailable
- 2026-07-18 | Docs | repository rules | Added the mandatory change-log update
  policy and this tracking document | Reviewed locally
- 2026-07-18 | Added | CPU3 subject-2 voice pipeline | Added AN4 VAD/WAV capture,
  WiFi-SPI TCP transport, command dispatch, P21.5 horn adapter, and a guarded
  TLD7002 display adapter; lamp pin mapping and glyphs remain deferred | Host
  C test passed; AURIX build not run because TASKING/ADS is unavailable
- 2026-07-18 | Changed | sound_recognise TCP protocol | Added stable
  `command_code` values for the 17 subject-2 lighting and horn commands and
  documented the MCU frame contract | `python -m unittest tests.test_subject2_protocol`
  passed
- 2026-07-18 | Fixed | CPU3 subject-2 protocol | Added missing-key rejection in
  the JSON parser and wired P21.5 PWM horn output with network failure recovery |
  Host C test and Python protocol tests passed

## 2026-07-18 - Initial Import

- 2026-07-18 | Added | TC387 firmware workspace | Imported application code,
  tests, AURIX project configuration, linker script, and required libraries |
  `ackermann_control_test` passed on the host
- 2026-07-18 | Docs | contributor guidance | Added `AGENTS.md` and `CLAUDE.md`
  with repository and working-method rules | Reviewed locally
- 2026-07-18 | Build | Git hygiene | Added `.gitignore` for local IDE state and
  generated `Debug/` outputs; retained the required `zf_device_config.a` link
  input | `git check-ignore` verified
