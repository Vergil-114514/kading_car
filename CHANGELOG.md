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
