# Working on modem-core

This repository is a teaching project, so the engineering tooling is part of the
lesson. Every CI job maps onto a claim the articles make about the code; the green
checkmark is the evidence, not the assertion. You can run every gate locally with
the same switches CI uses.

## Build

```bash
cmake -B build -G Ninja      # or your generator of choice
cmake --build build
ctest --test-dir build       # the golden tests
```

C++23 is required (GCC 14+, Clang 19+; Clang 18 cannot use `std::expected`
from libstdc++).

## The quality gates

Each is a CMake switch (off by default so a plain build stays fast and portable)
or a standalone tool. CI turns them on.

| Claim in the articles | Gate | Run it yourself |
|---|---|---|
| Builds clean, no warnings | `-Wall -Wextra`, warnings-as-errors | `cmake -B build -DMODEM_WERROR=ON && cmake --build build` |
| Produces the exact symbol stream and waveform | the `tx_golden_symbols` and `tx_golden` CTests | `ctest --test-dir build` |
| Consistent style, not argued in review | clang-format 19 (versions format differently, so CI pins one) | `clang-format --dry-run --Werror $(find modem tests -name '*.?pp')` |
| Follows the C++ Core Guidelines | clang-tidy 19 (`.clang-tidy`) | `cmake -B build -DMODEM_CLANG_TIDY=ON && cmake --build build` |
| Stays simple (avg complexity under six) | lizard | `lizard modem -l cpp --CCN 25` |
| Is tested | gcov / gcovr | `cmake -B build -DMODEM_COVERAGE=ON && cmake --build build && ctest --test-dir build && gcovr --root . --exclude tests` |
| Bounded, known stack | `-fstack-usage` | `cmake -B build -DMODEM_STACK_USAGE=ON && cmake --build build && python tools/check_stack_usage.py build 4096` |

Formatting and warnings are hard gates: fix them before you push. clang-tidy and
coverage are reported today and will ratchet into hard gates as the tree and the
test suite grow.

## A note on cycles

The articles are about code that runs on a microcontroller, and the obvious
question is cycle budget. Honest answer: the CI runners are x86 Linux, so they
cannot measure Cortex-M cycles. What they *can* prove is the part that matters for
determinism and safety: the stack is bounded (`-fstack-usage` above), there is no
heap on the signal path, and the output is bit-for-bit reproducible. Real
target-cycle profiling belongs on the hardware and arrives with the full modem;
the hooks are set up here so it is not a retrofit later.

## Before you push

- `clang-format` clean, `-DMODEM_WERROR=ON` build clean, `ctest` green.
- New functions should keep clear of the complexity ceiling; the few that run high
  are the genuinely hard kernels, not a license to add more.
- The receiver functions declared in the headers are intentionally unimplemented;
  they arrive with the receiver series. Do not stub them here.
