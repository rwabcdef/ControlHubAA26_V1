"""Module and class coupling for the ControlHubAA26 application code.

Two views, because each misses something the other sees:

Module coupling (link level, exact). Every symbol a module's objects leave
undefined (nm 'U') is resolved to the object that defines it in the Debug
build. That gives, per module:
    Ce  efferent - application modules it depends on
    Ca  afferent - application modules that depend on it
    I   instability, Ce / (Ca + Ce): 0 = everything leans on it, 1 = leans
        on everything and nothing leans on it
plus its dependencies outside the app (ST HAL, FreeRTOS, lwIP, FatFs, libc).
It cannot see header-only code: inline functions, templates and constants
that never become a linker symbol.

Class coupling (source level, heuristic). CBO in the Chidamber & Kemerer
sense: for each class/struct defined in the app, the other app classes named
in its body and its out-of-line member functions (comments and strings
ignored), in either direction. Name-based, so two classes with the same name
in different namespaces would be merged.

Usage (from anywhere):
    python coupling.py                  # both views
    python coupling.py --modules --edges
    python coupling.py --classes --top 40
    python coupling.py --json
Needs a current Debug build for the module view (CLAUDE.md, Build).
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

from _common import (APP_SCOPE, BUILD, CUBEMX_SCOPE, PROJECT, add_scope_args, all_objects, collect,
                     external_bucket, find_nm, function_spans, module_of,
                     nm_symbols, object_for, read_code, rel, require_build,
                     scope, stale_objects)

DEFINED = set("TWVBDRCGS")   # global/weak definitions that can satisfy a 'U'
WEAK = set("WV")


# ---- module coupling ---------------------------------------------------------

def module_coupling(files, app_files):
    """Rows for the modules in files; symbols resolve against all of app_files."""
    require_build()
    nm = find_nm()
    app_objs = {}
    for f in app_files:
        obj = object_for(f)
        if obj is not None:
            app_objs[obj.resolve()] = module_of(rel(f))

    strong, weak = {}, {}
    undefined = defaultdict(set)   # app module -> symbols it needs
    for obj in all_objects():
        obj = obj.resolve()
        for typ, name, _ in nm_symbols(obj, nm):
            if typ in DEFINED:
                (weak if typ in WEAK else strong).setdefault(name, obj)
            elif typ == "U" and obj in app_objs:
                undefined[app_objs[obj]].add(name)

    edges = defaultdict(lambda: defaultdict(set))   # from -> to -> symbols
    for mod, names in undefined.items():
        for name in names:
            obj = strong.get(name) or weak.get(name)
            if obj is None:
                to = "<libc/libgcc>"
            elif obj in app_objs:
                to = app_objs[obj]
            else:
                to = external_bucket(obj.relative_to(BUILD).as_posix())
            if to != mod:
                edges[mod][to].add(name)

    modules = sorted(set(app_objs.values()))
    wanted = {module_of(rel(f)) for f in files}
    rows = []
    for m in sorted(wanted & set(modules)):
        out_app = sorted(t for t in edges[m] if not t.startswith("<"))
        out_ext = {t: len(s) for t, s in edges[m].items() if t.startswith("<")}
        in_app = sorted(src for src in modules if m in edges[src])
        ce, ca = len(out_app), len(in_app)
        rows.append({
            "module": m, "ce": ce, "ca": ca,
            "instability": round(ce / (ca + ce), 2) if ca + ce else None,
            "depends_on": out_app, "used_by": in_app,
            "external": dict(sorted(out_ext.items())),
        })
    edge_list = [{"from": a, "to": b, "symbols": sorted(s)}
                 for a, tos in sorted(edges.items()) for b, s in sorted(tos.items())
                 if not b.startswith("<") and (a in wanted or b in wanted)]
    return rows, edge_list


# ---- class coupling ----------------------------------------------------------

_CLASS_DEF = re.compile(
    r"\b(class|struct)\s+(\w+)\s*(?:final\s*)?(?::[^;{()]*)?\{")


def class_extents(files):
    """{name: (kind, relpath, body_text)} for every class/struct definition."""
    classes, dupes = {}, set()
    for f in files:
        code = read_code(f)
        for m in _CLASS_DEF.finditer(code):
            if re.search(r"\benum\s+$", code[:m.start()][-12:]):
                continue   # 'enum class X {' is not a class
            depth, i = 0, m.end() - 1
            while i < len(code):
                if code[i] == "{":
                    depth += 1
                elif code[i] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            name = m.group(2)
            if name in classes:
                dupes.add(name)
            classes[name] = (m.group(1), rel(f), code[m.end():i])
    return classes, dupes


def class_coupling(files):
    classes, dupes = class_extents(files)
    names = set(classes)

    # Out-of-line member bodies: lizard names them 'Class::method'.
    member_text = defaultdict(str)
    code_cache = {}
    for relpath, fns in function_spans(files).items():
        lines = code_cache.setdefault(
            relpath, read_code(PROJECT / relpath).splitlines())
        for fname, start, end in fns:
            parts = fname.split("::")
            if len(parts) >= 2 and parts[-2] in names:
                member_text[parts[-2]] += "\n".join(lines[start - 1:end]) + "\n"

    word = re.compile(r"\b(" + "|".join(sorted(names, key=len, reverse=True)) + r")\b") \
        if names else None
    uses = {}
    for name, (_, _, body) in classes.items():
        found = set(word.findall(body + member_text[name])) if word else set()
        uses[name] = found - {name}

    used_by = defaultdict(set)
    for a, bs in uses.items():
        for b in bs:
            used_by[b].add(a)

    rows = []
    for name, (kind, relpath, _) in classes.items():
        coupled = uses[name] | used_by[name]
        rows.append({
            "class": name, "kind": kind, "file": relpath,
            "cbo": len(coupled), "out": len(uses[name]), "in": len(used_by[name]),
            "uses": sorted(uses[name]), "used_by": sorted(used_by[name]),
        })
    rows.sort(key=lambda r: (-r["cbo"], -r["out"], r["class"]))
    return rows, sorted(dupes)


# ---- report ------------------------------------------------------------------

def print_modules(rows, edges, args):
    print("Module coupling (link level)")
    print(f"  {'Ce':>3} {'Ca':>3} {'I':>5}  {'module':<28} external (symbols)")
    for r in sorted(rows, key=lambda r: (-(r["ce"] + r["ca"]), r["module"]))[:args.top]:
        inst = "  -  " if r["instability"] is None else f"{r['instability']:5.2f}"
        ext = ", ".join(f"{k} {v}" for k, v in r["external"].items())
        print(f"  {r['ce']:3d} {r['ca']:3d} {inst}  {r['module']:<28} {ext}")
    print()
    for r in sorted(rows, key=lambda r: -r["ce"])[:args.top]:
        if r["depends_on"]:
            print(f"  {r['module']} -> {', '.join(r['depends_on'])}")
    print()
    if args.edges:
        print("Edges (from -> to: symbols)")
        for e in edges:
            print(f"  {e['from']} -> {e['to']}: {', '.join(e['symbols'])}")
        print()


def print_classes(rows, dupes, args):
    print("Class coupling (CBO, source level)")
    print(f"  {'CBO':>3} {'out':>3} {'in':>3}  {'class':<28} file")
    for r in rows[:args.top]:
        print(f"  {r['cbo']:3d} {r['out']:3d} {r['in']:3d}  "
              f"{r['kind'][0]}:{r['class']:<26} {r['file']}")
    print()
    for r in rows[:args.top]:
        if r["uses"]:
            print(f"  {r['class']} uses {', '.join(r['uses'])}")
    if dupes:
        print(f"\n  note: defined more than once, merged: {', '.join(dupes)}")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_scope_args(ap)
    ap.add_argument("--modules", action="store_true", help="module view only")
    ap.add_argument("--classes", action="store_true", help="class view only")
    ap.add_argument("--edges", action="store_true",
                    help="list the symbols behind each module dependency")
    ap.add_argument("--top", type=int, default=25, help="rows per table (default 25)")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a report")
    args = ap.parse_args()
    both = not (args.modules or args.classes)

    files = collect(scope(args))
    out = {"scope": scope(args)}
    if both or args.modules:
        stale = stale_objects(files)
        if stale:
            print("warning: Debug objects older than source (rebuild for exact "
                  "results): " + ", ".join(stale), file=sys.stderr)
        app_files = collect(APP_SCOPE + (CUBEMX_SCOPE if args.all else []))
        out["modules"], out["edges"] = module_coupling(files, set(app_files) | set(files))
    if both or args.classes:
        out["classes"], out["duplicate_class_names"] = class_coupling(files)

    if args.json:
        json.dump(out, sys.stdout, indent=2)
        print()
        return
    print(f"Scope: {', '.join(out['scope'])}\n")
    if "modules" in out:
        print_modules(out["modules"], out["edges"], args)
    if "classes" in out:
        print_classes(out["classes"], out["duplicate_class_names"], args)


if __name__ == "__main__":
    main()
