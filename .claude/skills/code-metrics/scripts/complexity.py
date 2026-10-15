"""Cyclomatic complexity report for the ControlHubAA26 application code.

Wraps lizard (pip install lizard). By default only the hand-written
application layer is measured - CubeMX output, ST's drivers and the
third-party middleware are not ours to refactor and would drown the
report. Pass --all to add CubeMX's Core/Src/*.c as well.

Usage (from anywhere):
    python complexity.py                 # ranked report, top 20
    python complexity.py --top 50 --min-ccn 8
    python complexity.py --modified      # a switch counts once, not per case
    python complexity.py --path Components/Motor
    python complexity.py --json          # machine-readable, for diffing runs
"""

import argparse
import json
import sys

try:
    import lizard
except ImportError:
    sys.exit("lizard is not installed - run: python -m pip install --user lizard")

from _common import add_scope_args, collect, rel, scope

# Bands follow the usual McCabe / SEI reading of CCN.
BANDS = [
    (10, "low"),
    (20, "moderate"),
    (50, "high"),
    (10**9, "very high"),
]


def band(ccn):
    for limit, name in BANDS:
        if ccn <= limit:
            return name
    return BANDS[-1][1]


def analyse(files, modified):
    exts = lizard.get_extensions(["modified"] if modified else [])
    results = []
    for info in lizard.analyze_files([str(f) for f in files], exts=exts):
        for fn in info.function_list:
            results.append({
                "file": rel(info.filename),
                "function": fn.name,
                "line": fn.start_line,
                "ccn": fn.cyclomatic_complexity,
                "nloc": fn.nloc,
                "params": fn.parameter_count,
                "length": fn.length,
                "band": band(fn.cyclomatic_complexity),
            })
    return results


def per_file(funcs):
    files = {}
    for f in funcs:
        s = files.setdefault(f["file"], {"file": f["file"], "functions": 0,
                                          "nloc": 0, "ccn_sum": 0, "ccn_max": 0})
        s["functions"] += 1
        s["nloc"] += f["nloc"]
        s["ccn_sum"] += f["ccn"]
        s["ccn_max"] = max(s["ccn_max"], f["ccn"])
    for s in files.values():
        s["ccn_avg"] = round(s["ccn_sum"] / s["functions"], 2)
    return sorted(files.values(), key=lambda s: (-s["ccn_max"], -s["ccn_sum"]))


def print_report(funcs, files, args):
    if not funcs:
        print("No functions found in scope.")
        return
    n = len(funcs)
    total_ccn = sum(f["ccn"] for f in funcs)
    mode = "modified (switch = 1)" if args.modified else "standard (each case = 1)"
    print(f"Cyclomatic complexity - {mode}")
    print(f"Scope: {', '.join(args.scope)}")
    print(f"{len(files)} files, {n} functions, "
          f"{sum(f['nloc'] for f in funcs)} NLOC, avg CCN {total_ccn / n:.2f}\n")

    print("Distribution")
    lo = 0
    for limit, name in BANDS:
        count = sum(1 for f in funcs if lo < f["ccn"] <= limit)
        label = f"{lo + 1}-{limit}" if limit < 10**9 else f">{lo}"
        print(f"  {name:<10} {label:>7}  {count:4d}  ({100 * count / n:5.1f}%)")
        lo = limit
    print()

    ranked = [f for f in sorted(funcs, key=lambda f: (-f["ccn"], -f["nloc"]))
              if f["ccn"] >= args.min_ccn][:args.top]
    print(f"Top functions (CCN >= {args.min_ccn}, max {args.top})")
    print(f"  {'CCN':>4} {'NLOC':>5} {'PAR':>3}  {'band':<9}  function  (file:line)")
    for f in ranked:
        print(f"  {f['ccn']:4d} {f['nloc']:5d} {f['params']:3d}  {f['band']:<9}  "
              f"{f['function']}  ({f['file']}:{f['line']})")
    print()

    print("Files (by max CCN)")
    print(f"  {'max':>4} {'avg':>5} {'sum':>5} {'fns':>4} {'NLOC':>5}  file")
    for s in files[:args.top]:
        print(f"  {s['ccn_max']:4d} {s['ccn_avg']:5.2f} {s['ccn_sum']:5d} "
              f"{s['functions']:4d} {s['nloc']:5d}  {s['file']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_scope_args(ap)
    ap.add_argument("--modified", action="store_true",
                    help="modified CCN: a whole switch counts 1, not 1 per case")
    ap.add_argument("--top", type=int, default=20, help="rows per table (default 20)")
    ap.add_argument("--min-ccn", type=int, default=1, help="hide functions below this CCN")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a report")
    args = ap.parse_args()

    args.scope = scope(args)
    funcs = analyse(collect(args.scope), args.modified)
    files = per_file(funcs)

    if args.json:
        json.dump({"modified": args.modified, "scope": args.scope,
                   "functions": funcs, "files": files}, sys.stdout, indent=2)
        print()
    else:
        print_report(funcs, files, args)


if __name__ == "__main__":
    main()
