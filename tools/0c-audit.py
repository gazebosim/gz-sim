#!/usr/bin/env python3
"""
Phase 0c automated first-pass system audit.

Greps every system's .cc/.hh under src/systems/ for patterns that, under
the archetype ECM backend (Phase 0b), could misbehave:

  (1) A component pointer or reference stored as a class member. Under
      deferred semantics the pointer is valid only for the current phase;
      stashing it in a member field is a latent bug that may surface as
      a use-after-free on the next Commit.

  (2) A CreateComponent or RemoveComponent call followed by a Component
      read of the same type, same entity, within the same function. The
      mutation is queued; the read will see the pre-mutation state.

  (3) A RebuildViews() call. The archetype backend has no views in the
      legacy sense; this becomes a no-op and the call should be removed.

Patterns are deliberately loose. This tool is a first-pass signal
generator: false positives are expected, and a reviewer confirms each hit.

Usage:
  python3 tools/0c-audit.py                    # print findings
  python3 tools/0c-audit.py --json             # machine-readable output
  python3 tools/0c-audit.py --fail-on-findings # exit non-zero on any hit
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


# Pattern 1: member field of type `components::X *` or `components::X &`.
# Narrow: must be a declaration (type + whitespace + pointer/ref + ident
# + semicolon). Won't catch aliases or typedef'd aliases — that's OK for
# a first pass.
_MEMBER_PTR = re.compile(
    r"\bcomponents::\w+\s*[*&]\s+\w+\s*[{=;]"
)

# Pattern 2: CreateComponent/RemoveComponent followed within ~15 lines by
# a Component<T>(e) read of the same type+entity. This is an approximation
# — a proper AST tool would be better, but for a first-pass signal this
# catches the common cases without building a compiler.
_MUTATE_KW = re.compile(
    r"\b(?:CreateComponent|RemoveComponent|SetParentEntity)\s*<\s*(\w+)\s*>"
)
_READ_KW = re.compile(
    r"\bComponent\s*<\s*(\w+)\s*>\s*\("
)

# Pattern 3: explicit RebuildViews() call.
_REBUILD = re.compile(r"\bRebuildViews\s*\(")


def audit_file(path: Path) -> list[dict]:
    findings: list[dict] = []
    try:
        lines = path.read_text(errors="replace").splitlines()
    except OSError:
        return findings

    # Pattern 1 — member pointers/refs. To cut false positives (locals
    # of this type are common), only flag if the previous 20 lines have a
    # class-scope access specifier (public:/private:/protected:) and no
    # intervening function brace. Cheap, imperfect, but accurate enough.
    def in_class_scope(idx: int) -> bool:
        depth = 0
        for k in range(idx - 1, max(0, idx - 40), -1):
            prev = lines[k]
            depth += prev.count("}") - prev.count("{")
            if depth > 0:  # exited into outer scope — still class scope
                pass
            if re.search(r"^\s*(?:public|private|protected)\s*:", prev):
                return depth >= 0
        return False

    for i, line in enumerate(lines, 1):
        if _MEMBER_PTR.search(line):
            stripped = line.strip()
            if stripped.startswith(("//", "/*", "*")):
                continue
            if "private:" in line or "public:" in line or "protected:" in line:
                continue
            # Only flag if we're plausibly at class scope.
            if not in_class_scope(i):
                continue
            findings.append({
                "file": str(path),
                "line": i,
                "kind": "member_pointer",
                "text": stripped[:120],
            })

    # Pattern 2 — mutate-then-read of same type within a small window.
    for i, line in enumerate(lines, 1):
        m = _MUTATE_KW.search(line)
        if not m:
            continue
        comp = m.group(1)
        # Scan next 15 lines for a matching read.
        for j in range(i, min(len(lines), i + 15)):
            r = _READ_KW.search(lines[j])
            if r and r.group(1) == comp:
                findings.append({
                    "file": str(path),
                    "line": i,
                    "kind": "mutate_then_read",
                    "text": f"mutate {comp} at L{i}, read at L{j+1}",
                })
                break

    # Pattern 3 — RebuildViews.
    for i, line in enumerate(lines, 1):
        if _REBUILD.search(line):
            findings.append({
                "file": str(path),
                "line": i,
                "kind": "rebuild_views",
                "text": line.strip()[:120],
            })

    return findings


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="src/systems",
        help="Directory to audit (default: src/systems)")
    ap.add_argument("--json", action="store_true",
        help="Emit machine-readable JSON")
    ap.add_argument("--fail-on-findings", action="store_true",
        help="Exit non-zero if any finding reported")
    args = ap.parse_args(argv)

    root = Path(args.root)
    if not root.exists():
        print(f"audit: {root} does not exist", file=sys.stderr)
        return 2

    all_findings: list[dict] = []
    for path in sorted(root.rglob("*.cc")):
        all_findings.extend(audit_file(path))
    for path in sorted(root.rglob("*.hh")):
        all_findings.extend(audit_file(path))

    if args.json:
        json.dump(all_findings, sys.stdout, indent=2)
        sys.stdout.write("\n")
    else:
        by_file: dict[str, list[dict]] = {}
        for f in all_findings:
            by_file.setdefault(f["file"], []).append(f)
        for path, items in sorted(by_file.items()):
            print(f"{path}:")
            for f in items:
                print(f"  L{f['line']:>4}  [{f['kind']}] {f['text']}")
        print(f"\n{len(all_findings)} finding(s) across "
              f"{len(by_file)} file(s)")
        print(f"  ({len(list(root.rglob('*.cc')))} .cc files audited)")

    return 1 if all_findings and args.fail_on_findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
