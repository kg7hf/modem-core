# Working on modem-core

This repository is a teaching project, and its tooling is part of what it teaches.
Every CI job checks a claim the articles make about the code, and you can run every
gate locally with the same switches CI uses.

## Build

```bash
cmake --preset gcc-debug           # or gcc-release, clang-debug, clang-release
cmake --build --preset gcc-debug
ctest --preset gcc-debug           # the golden tests and the unit tests
```

The presets in `CMakePresets.json` build with Ninja into `build/<preset>/`, with
warnings as errors, using the `g++` or `clang++` on your PATH. C++23 is required
(GCC 14+, Clang 19+; Clang 18 cannot use `std::expected` from libstdc++). If your
default compiler is older, add a `CMakeUserPresets.json` (git ignores it) that
inherits a preset and sets `CMAKE_CXX_COMPILER`, for example to `g++-14`. Without
presets: `cmake -B build -G Ninja`, `cmake --build build`, `ctest --test-dir build`.

## VS Code

Open the repository folder and install the two recommended extensions (CMake Tools
and C/C++). Pick a configure preset when CMake Tools asks (a `-debug` one to step
through the code), then:

- **Build:** F7, or Ctrl+Shift+B.
- **Run:** the *Run tx_demo* task runs `tx_demo Hi` and writes `tx_out.wav` into the
  build folder. *Run front_end_demo* prints every number in Part 3, step by step, and
  *Run the transmit-chain walkthrough* every number in Part 2, each in its article's
  order.
- **Debug:** F5 with *tx_demo (gdb)* stops at the top of `main()` in `tests/tx_demo.cpp`;
  step down into the transmit chain from there. *front_end_demo (gdb)* does the same for
  Part 3: it stops at the top of `main()`, and each step it calls is a function in
  `tests/walkthrough/front_end_steps.cpp` (`front_end_tests` checks the same functions).
  F5 with *transmit_chain_tests (gdb)* asks which section of Part 2 to run. On a Mac, use
  the *(lldb, macOS)* versions.
- **Test:** the *Tests* task runs them all. The Testing panel (the flask icon) lists
  every unit test; run one, or debug it and step through the code it checks.

The debugger has to be on your PATH: gdb comes with MinGW-w64 toolchains such as
WinLibs, and with most Linux distributions.

## Unit tests

The unit tests use Catch2 v3.16.0, a git submodule in `external/catch2`, fixed to its
release commit. It is the tests' one external tool: it builds only for the tests, never
into the modem, and `-DMODEM_BUILD_TESTS=OFF` builds without it. If a configure stops
asking for it, run `git submodule update --init`. Catch2 is built without exceptions, like
the modem: a failed `REQUIRE` ends that test's process, and each test case runs as its own
CTest test. Each case checks one idea and prints what it measured. Name a test case
with a plain sentence and no commas: Catch2 reads a comma in a test spec as "or", so a
name with one cannot be run on its own. Cases tagged `[.long]` or `[.report]` are
hidden; `ctest -L long` and `ctest -L report` run them.

## The quality gates

Each is a CMake switch (off by default so a plain build stays fast and portable)
or a standalone tool. CI turns them on.

| Claim in the articles | Gate | Run it yourself |
|---|---|---|
| Builds clean, no warnings | `-Wall -Wextra`, warnings-as-errors | `cmake -B build -DMODEM_WERROR=ON && cmake --build build` |
| Produces the exact symbol stream and waveform | the `tx_golden_symbols` and `tx_golden` CTests | `ctest --test-dir build` |
| Each stage does what its article says | the Catch2 unit tests (`tests/transmit_chain_tests.cpp`, `tests/front_end_tests.cpp`) | `ctest --test-dir build` |
| Consistent style, not argued in review | clang-format 19 (versions format differently, so CI uses one fixed version) | `clang-format --dry-run --Werror $(find modem tests -name '*.?pp')` |
| Follows the C++ Core Guidelines | clang-tidy 19 (`.clang-tidy`) | `cmake -B build -DMODEM_CLANG_TIDY=ON && cmake --build build` |
| Stays simple (avg complexity under six) | lizard | `lizard modem -l cpp --CCN 25` |
| Is tested | gcov / gcovr | `cmake -B build -DMODEM_COVERAGE=ON && cmake --build build && ctest --test-dir build && gcovr --root . --exclude tests --exclude external` |
| Bounded, known stack | `-fstack-usage` | `cmake -B build -DMODEM_STACK_USAGE=ON && cmake --build build && python tools/check_stack_usage.py build 4096` |

Formatting and warnings are hard gates: fix them before you push. clang-tidy and
coverage are reported today and will ratchet into hard gates as the tree and the
test suite grow.

## A note on cycles

The articles are about code that runs on a microcontroller, so the obvious
question is the cycle budget. The CI runners are x86 Linux and cannot measure
Cortex-M cycles. What they *can* prove is the part that matters for
determinism and safety: the stack is bounded (`-fstack-usage` above), there is no
heap on the signal path, and the transmitted symbol stream is bit-for-bit identical
on every compiler and platform (the golden test checks it). The carrier comes from
a table and the build forbids fused multiply-add, so the float audio does not depend
on the optimizer; its last bit can still differ across math libraries, because the
filter taps are computed with the library's cosine when the modem starts. Real
target-cycle profiling belongs on the hardware and arrives with
the full modem; the hooks are set up here so it is not a retrofit later.

## Before you push

- `clang-format` clean, `-DMODEM_WERROR=ON` build clean, `ctest` green.
- New functions should keep clear of the complexity ceiling; the few that run high
  are the genuinely hard kernels, not a license to add more.
- The receiver functions that are only declared in the headers are intentionally
  unimplemented; they arrive with the receiver series. Do not stub them here.
