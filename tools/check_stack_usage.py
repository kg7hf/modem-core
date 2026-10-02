#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Paul R. Decker (KG7HF)
#
# Checks the -fstack-usage (.su) reports under a build directory and fails if any
# function's stack frame exceeds a bound. The signal path has no heap and no
# recursion, so the stack is the last place memory can surprise you; this proves
# every frame is bounded and known. Usage:
#     python tools/check_stack_usage.py <build-dir> [bound-bytes]

import pathlib
import sys

build = pathlib.Path(sys.argv[1])
bound = int(sys.argv[2]) if len(sys.argv) > 2 else 4096

rows = []
for su in build.rglob("*.su"):
    for line in su.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split("\t")
        if len(parts) < 3:
            continue
        try:
            size = int(parts[-2])
        except ValueError:
            continue
        rows.append((size, parts[0], parts[-1]))

rows.sort(reverse=True)
print(f"stack-usage: {len(rows)} functions, bound {bound} bytes")
print("largest frames:")
for size, loc, qual in rows[:15]:
    print(f"  {size:7d}  {loc}  [{qual}]")

worst = rows[0][0] if rows else 0
if worst > bound:
    print(f"FAIL: a frame of {worst} bytes exceeds the {bound}-byte bound")
    sys.exit(1)
print(f"OK: largest frame {worst} bytes is within the {bound}-byte bound")
