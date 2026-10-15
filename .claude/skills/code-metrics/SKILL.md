---
name: code-metrics
description: Static code metrics for the ControlHubAA26 firmware - cyclomatic complexity per function, module and class coupling, and the global variables with the functions that touch them. Use when asked for code metrics, complexity, coupling, dependencies between modules or classes, globals / shared state, refactoring candidates, or a code-quality snapshot of this repo.
---

# Code metrics

Three scripts in `scripts/`, one per metric. Each one measures only the hand-written
application layer by default: `Components/`, `Middlewares/SerLink/`, `HAL/`,
`Core/Src/*.cpp`, `Core/Inc/*.hpp` and `Core/Src/httpd_app.c`. CubeMX output, ST's
drivers, FreeRTOS, lwIP and FatFs are left out. They aren't ours to refactor, and they
would swamp the numbers. Every script also takes:

- `--path <dir-or-file>` (repeatable, relative to the CubeIDE project
  `ControlHubAA26_V1/`) to narrow the scope.
- `--all` to add CubeMX's `Core/Src/*.c`.
- `--top N` to set the number of rows.
- `--json` for machine-readable output, e.g. to compare two runs.

Run them from the repo root with Python 3 (`python`). Absolute paths are not needed; the
scripts find the project themselves.

## Prerequisites

- **lizard** (complexity, and the function boundaries the other two use). If a script
  stops with "lizard is not installed", run `python -m pip install --user lizard`.
- **A current Debug build** for `coupling.py` (module view) and `globals.py`. They read
  the objects in `ControlHubAA26_V1/Debug/` with `arm-none-eabi-nm`. A script warns on
  stderr when an object is older than its source. When that happens, rebuild first
  (CLAUDE.md, Build: `make -j8 all` in `ControlHubAA26_V1/Debug`), then re-run. `nm` is
  taken from PATH or found under `C:/ST/STM32CubeIDE_*`.

## 1. Complexity: `scripts/complexity.py`

```bash
python .claude/skills/code-metrics/scripts/complexity.py            # standard CCN
python .claude/skills/code-metrics/scripts/complexity.py --modified # switch = 1
python .claude/skills/code-metrics/scripts/complexity.py --min-ccn 11 --top 40
```

This reports the McCabe cyclomatic complexity (CCN) of every function, banded as
1–10 low, 11–20 moderate, 21–50 high, and >50 very high. It shows the distribution, the
ranked functions (CCN, NLOC, parameter count, file:line), and a per-file max/avg/sum.

Run it both ways and compare. Standard CCN counts each `case`, which makes the SerLink
socket handlers look worse than they are: they are mostly flat `switch`es that dispatch
on a command letter. `--modified` counts a whole switch as one decision. A function that
is still high under `--modified` has real nested logic, and is the better refactoring
candidate.

## 2. Coupling: `scripts/coupling.py`

```bash
python .claude/skills/code-metrics/scripts/coupling.py               # both views
python .claude/skills/code-metrics/scripts/coupling.py --modules --edges
python .claude/skills/code-metrics/scripts/coupling.py --classes
```

There are two complementary views.

- **Module coupling (link level, exact).** The script resolves each undefined symbol in
  a module's objects to the object that defines it. Modules are: one per folder under
  `Components/` and `HAL/`, one per SerLink layer, and each `Core/Src/*.cpp` on its own.
  For each module it reports:
  - **Ce** (efferent): the app modules it depends on.
  - **Ca** (afferent): the app modules that depend on it.
  - **I = Ce/(Ca+Ce)**: instability.
  - Its symbol counts against `<ST HAL>`, `<FreeRTOS>`, `<lwIP>`, `<FatFs>` and
    `<CubeMX Core>`, i.e. how tied it is to the hardware or the RTOS.

  `--edges` lists the symbols behind each dependency. This view cannot see header-only
  code (inline functions, templates, constants).
- **Class coupling (CBO, source level, heuristic).** For each class/struct, this lists
  the other app classes named in its body and its out-of-line member functions, plus the
  classes that name it. It works by name, so two classes with the same name would be
  merged; any duplicates are reported.

How to read it: `Core/main_tasks` has a high Ce and I = 1 by design, because it is the
composition root that builds every object. What matters is coupling *between* drivers
and layers. A driver should have Ce ≈ 0 to other app modules, and stable building blocks
(Frame, Socket) should have high Ca and low I. Flag any dependency that runs the wrong
way: a low-level module depending on application code, or a SerLink layer depending on a
specific link driver.

## 3. Globals: `scripts/globals.py`

```bash
python .claude/skills/code-metrics/scripts/globals.py
python .claude/skills/code-metrics/scripts/globals.py --kind external --top 60
python .claude/skills/code-metrics/scripts/globals.py --unused
python .claude/skills/code-metrics/scripts/globals.py --const   # include read-only data
```

The inventory is exact: every statically allocated variable comes from `nm`, with its
size and section. Each one is classified as **external**, **file static** (including
anonymous-namespace variables), **class static**, or **function static**. Constants are
excluded unless `--const` is given.

Usage is approximate: a function counts as using a global if the global's name appears
in its body (comments and strings stripped). It reports:

- counts and bytes by kind and by module;
- the most shared globals (how many functions and files use each);
- external globals that other files link to (exact, from `nm` `U` entries);
- the functions that touch the most globals;
- with `--unused`, globals no function names. These are used only in another global's
  initialiser, only by address, or not at all.

How to read it: the project's convention is that drivers and sockets are file-scope
objects in `main_tasks.cpp` (no dynamic allocation; see CLAUDE.md), so a large global
count there is expected. So is `initTasks` touching nearly all of them. What to look for:

- external globals that other files link to directly (hidden coupling across modules);
- globals named by many functions outside `initTasks`;
- mutable globals shared between tasks or ISRs. These need a threading contract; check
  the owning class's header comment.

## Producing a report

1. Check the build is current (rebuild if a script warns), then run all three with
   default options. Add `--modified` for complexity.
2. Summarise for the user:
   - the headline numbers;
   - the top refactoring candidates, i.e. functions that rank high on CCN, modified CCN
     and globals touched together;
   - any wrong-way dependencies;
   - any cross-file globals.

   Cite `file:line`, and say which figures are heuristic (CBO, global usage) and which
   are exact (module links, global inventory).
3. When comparing over time, save each run's `--json` output and diff the two.
