# Repository Guidelines

## Project Structure & Module Organization

Firmware is `TC387_KDC_7_13/`: application and board-control code in `code/`,
CPU entry points and ISRs in `user/`, and host tests and stubs in `tests/`.
`libraries/` is vendor support; change it deliberately. `Debug/` is generated
output; linker layout is `Lcf_Tasking_Tricore_Tc.lsl`.

## Working Method

Before coding, state assumptions and tradeoffs. Present alternatives; ask if
context cannot resolve ambiguity. Call out simpler alternatives and push back
when warranted. Favor the smallest solution: no
unrequested features, configuration, single-use abstractions, or impossible-case handling.

Make task-related edits and match local style. Do not reformat, refactor,
or delete unrelated code. Remove only unused items created by your changes;
report existing dead code.

For multi-step work, publish a short plan with verification per step. Define
observable success criteria; when practical, add a reproducing test and iterate
until checks pass.

## Change Documentation

After every completed change, update `CHANGELOG.md` before handoff or commit.
Record its date, type, affected path or module, concise impact, and validation;
when validation did not run, state why.

## Build, Test, and Development Commands

Open `TC387_KDC_7_13` in AURIX Development Studio, select `Debug`, and use
**Project > Build Project**. The TASKING TC38x build emits
`Debug/TC387_KDC_7_13.elf` and `.hex`. With the same environment:

```powershell
make -C TC387_KDC_7_13/Debug
make -C TC387_KDC_7_13/Debug clean
```

Do not hand-edit generated files under `Debug/`; regenerate them from the IDE.

## Coding Style & Naming Conventions

Follow surrounding C: four-space indentation, opening braces on the next line,
and `if(value)`. Use `snake_case` filenames and private helpers, public APIs
such as `Ackermann_control_step`, and `UPPER_SNAKE_CASE` macros and typedefs.
Use header guards, fixed-width integers, and explicit unsigned literals (`0U`).
No formatter or linter is configured.

## Testing Guidelines

Host tests are standalone assertion programs named `tests/<module>_test.c.host`.
Add nearby hardware stubs and cover normal, boundary, and failsafe behavior.
From `TC387_KDC_7_13/`, for example:

```powershell
gcc -std=c11 -Wall -Wextra -x c -DACKERMANN_HOST_TEST -I code tests/ackermann_control_test.c.host code/ackermann_control.c -lm -o $env:TEMP/ackermann_control_test.exe
& $env:TEMP/ackermann_control_test.exe
```

There is no aggregate runner or coverage threshold. Rebuild and perform
hardware validation for timer, GPIO, motor, or ISR changes.

## Commits & Pull Requests

This workspace has no `.git` history, so no convention can be inferred. Use
concise imperative, scoped subjects, for example `fix(encoder): read TIM4 for
right wheel`. Keep generated artifacts and vendor updates separate. Pull
requests must explain impact, list host and hardware validation, link an issue
when applicable, and include logs or screenshots for observable debug/UI work.
