"""Shared helpers for the code-metrics scripts: scope, source text, nm."""

import glob
import re
import shutil
import subprocess
import sys
from pathlib import Path

# .claude/skills/code-metrics/scripts/_common.py -> repo root is 4 up.
REPO_ROOT = Path(__file__).resolve().parents[4]
PROJECT = REPO_ROOT / "ControlHubAA26_V1"   # the CubeIDE project root
BUILD = PROJECT / "Debug"

SOURCE_EXTS = {".c", ".cpp", ".h", ".hpp"}

# Application code, relative to PROJECT. Directories are walked recursively.
# CubeMX output, ST's drivers and third-party middleware are left out: they
# are not ours to refactor and would drown the report.
APP_SCOPE = [
    "Components",
    "Middlewares/SerLink",
    "HAL",
    "Core/Src/*.cpp",
    "Core/Inc/*.hpp",
    "Core/Src/httpd_app.c",     # hand-written, not CubeMX
]
# CubeMX-generated C, only with --all.
CUBEMX_SCOPE = ["Core/Src/*.c"]


def scope(args):
    """Scope patterns from the common --path / --all options."""
    return args.path or (APP_SCOPE + (CUBEMX_SCOPE if args.all else []))


def add_scope_args(ap):
    ap.add_argument("--all", action="store_true",
                    help="also include CubeMX-generated Core/Src/*.c")
    ap.add_argument("--path", action="append",
                    help="only this path, relative to the CubeIDE project; repeatable")


def collect(patterns, exts=SOURCE_EXTS):
    files = set()
    for pat in patterns:
        if any(ch in pat for ch in "*?["):
            files.update(PROJECT.glob(pat))
            continue
        p = PROJECT / pat
        if p.is_dir():
            files.update(f for f in p.rglob("*") if f.suffix in exts)
        elif p.is_file():
            files.add(p)
    return sorted(f for f in files if f.suffix in exts)


def rel(path):
    return Path(path).resolve().relative_to(PROJECT).as_posix()


def module_of(relpath):
    """Module a project-relative file belongs to.

    One folder per driver under Components/ and HAL/, one per SerLink layer,
    and each Core/Src/*.cpp on its own (Core is several unrelated modules).
    """
    parts = relpath.split("/")
    if parts[0] in ("Components", "HAL") and len(parts) > 2:
        return "/".join(parts[:2])
    if parts[:2] == ["Middlewares", "SerLink"]:
        return "SerLink/" + parts[2] if len(parts) > 3 else "SerLink"
    if parts[0] == "Core":
        return "Core/" + Path(parts[-1]).stem
    return Path(relpath).parent.as_posix() or relpath


_COMMENT_OR_STRING = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)


def strip_code(text):
    """Blank out comments and string/char literals, keeping line numbers."""
    def blank(m):
        return re.sub(r"[^\n]", " ", m.group(0))
    return _COMMENT_OR_STRING.sub(blank, text)


def read_code(path):
    return strip_code(Path(path).read_text(encoding="utf-8", errors="replace"))


# ---- nm over the Debug build -------------------------------------------------

def find_nm():
    nm = shutil.which("arm-none-eabi-nm")
    if nm:
        return nm
    hits = sorted(glob.glob("C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/"
                            "*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm.exe"))
    if hits:
        return hits[-1]
    sys.exit("arm-none-eabi-nm not found - put the CubeIDE GNU tools on PATH "
             "(see CLAUDE.md, Build)")


def object_for(source):
    """Debug/ object for a source file, or None if it has not been built.

    Headers have none (a .h sharing a .c's stem must not claim its object).
    """
    if Path(source).suffix not in (".c", ".cpp"):
        return None
    obj = BUILD / Path(rel(source)).with_suffix(".o")
    return obj if obj.is_file() else None


def require_build():
    if not BUILD.is_dir():
        sys.exit(f"{BUILD} not found - build the Debug configuration first "
                 "(CLAUDE.md, Build)")


def stale_objects(sources):
    """Sources whose object is missing or older than the source."""
    out = []
    for s in sources:
        if Path(s).suffix not in (".c", ".cpp"):
            continue
        obj = object_for(s)
        if obj is None or obj.stat().st_mtime < Path(s).stat().st_mtime:
            out.append(rel(s))
    return out


# "addr size type name", "addr type name" (size unknown) or "type name" (U).
# Addresses and sizes are fixed 8-digit hex on this 32-bit target.
_NM_LINE = re.compile(r"^\s*([0-9a-f]{8})?\s*([0-9a-f]{8})?\s*([A-Za-z])\s(.+)$")


def nm_symbols(obj, nm=None):
    """[(type, name, size)] for one object, names demangled."""
    nm = nm or find_nm()
    out = subprocess.run([nm, "-C", "-S", str(obj)], capture_output=True,
                         text=True, check=True).stdout
    syms = []
    for line in out.splitlines():
        m = _NM_LINE.match(line)
        if m:
            _, size, typ, name = m.groups()
            syms.append((typ, name, int(size, 16) if size else 0))
    return syms


def all_objects():
    return sorted(BUILD.rglob("*.o"))


def external_bucket(obj_rel):
    """Coarse name for a non-application object (who the app depends on)."""
    p = obj_rel.replace("\\", "/")
    if "STM32F4xx_HAL_Driver" in p:
        return "<ST HAL>"
    if "FreeRTOS" in p:
        return "<FreeRTOS>"
    if "LwIP" in p or p.startswith("LWIP/") or "/lwip/" in p.lower():
        return "<lwIP>"
    if "FatFs" in p or p.startswith("FATFS/"):
        return "<FatFs>"
    if p.startswith("Core/"):
        return "<CubeMX Core>"
    return "<" + p.split("/")[0] + ">"


def function_spans(files):
    """{relpath: [(name, start, end)]} from lizard, for usage lookups."""
    try:
        import lizard
    except ImportError:
        sys.exit("lizard is not installed - run: python -m pip install --user lizard")
    spans = {}
    for info in lizard.analyze_files([str(f) for f in files]):
        spans[rel(info.filename)] = [(fn.name, fn.start_line, fn.end_line)
                                     for fn in info.function_list]
    return spans


# Demangled names and box characters must not trip a cp1252 console.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
