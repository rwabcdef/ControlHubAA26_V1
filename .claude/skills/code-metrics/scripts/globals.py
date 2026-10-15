"""Global variables in the ControlHubAA26 application code, and who touches them.

The inventory is exact: it comes from nm over the Debug build's objects, so
every statically allocated variable is found, whatever the declaration looks
like. Each is one of:
    external          namespace scope, external linkage (nm B/D/C)
    file static       namespace scope, internal linkage ('static', or an
                      anonymous namespace) (nm b/d)
    class static      a static data member
    function static   a 'static' local inside a function
Constants (.rodata, nm R/r) are left out unless --const is given: read-only
data is not shared mutable state.

Usage, i.e. coupling, is found in the source, so it is approximate: a function
"uses" a global when the name appears in its body (comments and strings
ignored). That over-counts if a local or member shadows the name, and misses
access through a pointer or reference handed out earlier. For external
globals, the files that reference them are also taken from the objects' nm 'U'
entries, which is exact.

Usage (from anywhere):
    python globals.py                   # summary, most-shared globals, functions
    python globals.py --top 60 --kind external
    python globals.py --unused          # globals no function names (only init/storage)
    python globals.py --json
Needs a current Debug build (CLAUDE.md, Build).
"""

import argparse
import json
import re
import sys
from collections import defaultdict

from _common import (PROJECT, add_scope_args, collect, find_nm, function_spans,
                     module_of, nm_symbols, object_for, read_code, rel,
                     require_build, scope, stale_objects)

DATA = {"B": "external", "D": "external", "C": "external",
        "b": "file static", "d": "file static"}
CONST = {"R": "external", "r": "file static"}
SKIP_PREFIXES = ("guard variable", "vtable", "typeinfo", "VTT",
                 "construction vtable", "__", ".")


def classify(name, kind):
    anon = "(anonymous namespace)::"
    if name.startswith(anon):
        name = name[len(anon):]
        kind = "file static"
    if "::" in name:
        owner, _, short = name.rpartition("::")
        if owner.endswith(")") or ")::" in name:
            return "function static", owner, short
        return "class static", owner, short
    return kind, None, name


def inventory(files, with_const):
    nm = find_nm()
    table = dict(DATA, **(CONST if with_const else {}))
    found, refs = [], defaultdict(set)
    for f in files:
        obj = object_for(f)
        if obj is None:
            continue
        for typ, name, size in nm_symbols(obj, nm):
            if typ == "U":
                refs[name].add(rel(f))
                continue
            if typ not in table or name.startswith(SKIP_PREFIXES):
                continue
            kind, owner, short = classify(name, table[typ])
            found.append({"name": name, "short": short, "owner": owner,
                          "kind": kind, "const": typ in CONST, "size": size,
                          "section": ".bss" if typ in "Bb" else ".data"
                          if typ in "Dd" else ".rodata" if typ in "Rr" else "common",
                          "file": rel(f), "module": module_of(rel(f))})
    return found, refs


def usage(globals_, files):
    spans = function_spans(files)
    bodies = {}   # (file, function) -> body text
    for relpath, fns in spans.items():
        lines = read_code(PROJECT / relpath).splitlines()
        for fname, start, end in fns:
            bodies[(relpath, fname)] = "\n".join(lines[start - 1:end])

    for g in globals_:
        if g["kind"] == "function static":
            fn = g["owner"].split("(")[0]
            users = {k for k in bodies if k[1] == fn}
        else:
            word = re.compile(r"\b" + re.escape(g["short"]) + r"\b")
            qualified = (re.compile(r"\b" + re.escape(g["owner"].split("::")[-1])
                                    + r"\s*::\s*" + re.escape(g["short"]) + r"\b")
                         if g["kind"] == "class static" else None)
            users = set()
            for (relpath, fname), body in bodies.items():
                if g["kind"] == "file static" and relpath != g["file"]:
                    continue   # internal linkage: invisible elsewhere
                if g["kind"] == "class static":
                    own = fname.split("::")[-2:-1] == [g["owner"].split("::")[-1]]
                    hit = (own and word.search(body)) or qualified.search(body)
                else:
                    hit = word.search(body)
                if hit:
                    users.add((relpath, fname))
        g["functions"] = sorted(f"{fn} ({p})" for p, fn in users)
        g["files"] = sorted({p for p, _ in users})
    return bodies


def summarise(globals_, refs):
    for g in globals_:
        if g["kind"] == "external":
            g["linked_from"] = sorted(refs.get(g["name"], set()))
    per_fn = defaultdict(list)
    for g in globals_:
        for fn in g["functions"]:
            per_fn[fn].append(g["name"])
    functions = sorted(({"function": k, "globals": len(v), "names": sorted(v)}
                        for k, v in per_fn.items()),
                       key=lambda r: (-r["globals"], r["function"]))
    per_mod = defaultdict(lambda: {"count": 0, "bytes": 0})
    for g in globals_:
        per_mod[g["module"]]["count"] += 1
        per_mod[g["module"]]["bytes"] += g["size"]
    modules = sorted(({"module": k, **v} for k, v in per_mod.items()),
                     key=lambda r: (-r["count"], r["module"]))
    return functions, modules


def print_report(globals_, functions, modules, args):
    n = len(globals_)
    print(f"Scope: {', '.join(args.scope)}")
    print(f"{n} globals, {sum(g['size'] for g in globals_)} bytes"
          f"{' (constants included)' if args.const else ''}\n")

    print("By kind")
    for kind in ("external", "file static", "class static", "function static"):
        gs = [g for g in globals_ if g["kind"] == kind]
        print(f"  {kind:<16} {len(gs):4d}  {sum(g['size'] for g in gs):7d} bytes")
    print()

    print("By module")
    for m in modules[:args.top]:
        print(f"  {m['count']:4d}  {m['bytes']:7d} bytes  {m['module']}")
    print()

    shown = [g for g in globals_ if not args.kind or g["kind"] == args.kind]
    if args.unused:
        print("Globals no function body names (used only in another global's "
              "initialiser, by address, or not at all)")
        for g in sorted(shown, key=lambda g: (g["file"], g["name"])):
            if not g["functions"]:
                print(f"  {g['size']:6d}  {g['kind']:<15}  {g['name']}  ({g['file']})")
        print()
        return

    print("Most shared globals (functions / files that name them)")
    print(f"  {'fns':>3} {'files':>5} {'bytes':>6}  {'kind':<15}  global  (defined in)")
    for g in sorted(shown, key=lambda g: (-len(g["functions"]), -len(g["files"]),
                                          g["name"]))[:args.top]:
        print(f"  {len(g['functions']):3d} {len(g['files']):5d} {g['size']:6d}  "
              f"{g['kind']:<15}  {g['name']}  ({g['file']})")
    cross = [g for g in shown if len(g.get("linked_from", [])) > 0]
    if cross:
        print("\n  external globals linked from other files (exact, from nm):")
        for g in cross:
            print(f"    {g['name']} ({g['file']}) <- {', '.join(g['linked_from'])}")
    print()

    print("Functions touching the most globals")
    for f in functions[:args.top]:
        print(f"  {f['globals']:3d}  {f['function']}")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_scope_args(ap)
    ap.add_argument("--const", action="store_true", help="include read-only data")
    ap.add_argument("--kind", choices=["external", "file static", "class static",
                                       "function static"], help="only this kind")
    ap.add_argument("--unused", action="store_true",
                    help="list globals no function names, instead of the usual tables")
    ap.add_argument("--top", type=int, default=25, help="rows per table (default 25)")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a report")
    args = ap.parse_args()
    args.scope = scope(args)

    require_build()
    files = collect(args.scope)
    stale = stale_objects(files)
    if stale:
        print("warning: Debug objects older than source (rebuild for exact "
              "results): " + ", ".join(stale), file=sys.stderr)

    globals_, refs = inventory(files, args.const)
    usage(globals_, files)
    functions, modules = summarise(globals_, refs)

    if args.json:
        json.dump({"scope": args.scope, "globals": globals_, "functions": functions,
                   "modules": modules}, sys.stdout, indent=2)
        print()
    else:
        print_report(globals_, functions, modules, args)


if __name__ == "__main__":
    main()
