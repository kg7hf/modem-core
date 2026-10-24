# Contributor testing

Run these commands from a clone of public `modem-core`, with Catch2 initialized:

```sh
git submodule update --init --recursive
cmake -P tools/test.cmake
```

The shared command configures a Release build with warnings as errors, builds,
and runs CTest. It stops on configuration, build or test failure, including an
empty test selection. It needs CMake 3.25+, Ninja and GCC 14+ or Clang 19+ with
a suitable C++23 standard library. No Python is needed for these commands.

## Native commands

```sh
cmake -DCXX=g++-14 -DJOBS=2 -P tools/test.cmake
cmake -DSUITE=article -DCXX=g++-14 -P tools/test.cmake
cmake -DSUITE=full -DCXX=g++-14 -P tools/test.cmake
```

PowerShell accepts the same commands. Set `CXX` to your installed compiler's name
or full path (quote arguments containing spaces). Alternatively use the `CXX`
environment variable on a fresh build. Use different build directories when
switching compilers, for example `-DBUILD_DIR=build/contributor-clang`.
All `-D` arguments must precede `-P`.

| Suite | Selection | Downloads |
| --- | --- | --- |
| `quick` (default) | Registered unit/golden tests, excluding long/report labels; includes package-validation negative tests | None |
| `article` | `first-path` recording tests and demo | Exact committed release pack, if absent |
| `full` | All registered public tests, including long/report and recording cases | Exact committed release pack, if absent |

`full` describes this public revision's suite. It is not the private compliance
battery, a hardware test or a standards-conformance verdict. A quick pass does
not imply recording tests passed. The driver sets article data OFF for quick
and ON for article/full, even if the build directory was previously configured
otherwise. Default build folders are `build/contributor-<suite>`; workers default
to two. Use `-DJOBS=N` to change that budget.

For one test, configure/build using the shared command, then use CTest directly:
`ctest --test-dir build/contributor-quick -R tx_golden --output-on-failure`.

## Docker on Linux or Windows

The Dockerfile installs GCC 14, Clang 19, CMake and Ninja on Ubuntu 24.04.
It contains tools only. Your mounted working copy, including uncommitted edits,
is what gets tested. It does not include a private compiler, credentials or data.
Package security updates can change tool patch versions; this is a repeatable
toolchain recipe, not a promise of bit-identical container builds.

On Linux (or a WSL shell):

```sh
docker build -t modem-core-dev .
docker run --rm --user "$(id -u):$(id -g)" --cpus=2 --memory=4g --mount "type=bind,source=$PWD,target=/src" modem-core-dev
docker run --rm --user "$(id -u):$(id -g)" --cpus=2 --memory=4g --mount "type=bind,source=$PWD,target=/src" modem-core-dev -DSUITE=article -P tools/test.cmake
```

On Windows, start Docker Desktop with Linux containers, then in PowerShell:

```powershell
docker build -t modem-core-dev .
docker run --rm --cpus=2 --memory=4g --mount "type=bind,source=$($PWD.Path),target=/src" modem-core-dev
docker run --rm --cpus=2 --memory=4g --mount "type=bind,source=$($PWD.Path),target=/src" modem-core-dev -DSUITE=article -P tools/test.cmake
```

Use `-DSUITE=full` for the full public suite, or add `-DCXX=clang++-19` with a
distinct `-DBUILD_DIR=build/docker-clang`. Never reuse a native Windows CMake
build directory in Linux Docker: give one environment a distinct BUILD_DIR.
The limits above are a starting budget; increase memory for larger future suites.
No privileged container, Docker socket mount or hardware access is needed.

## Versioned test data

Fetch and verify the existing article pack without building a compiler target:

```sh
cmake -P tools/fetch_test_data.cmake
```

The repository's `tests/data/first-path/pack.json` records the public HTTPS URL,
archive size and SHA-256, package identity and case-manifest SHA-256. CMake checks
these plus VERSION and every inventory entry. Missing/corrupt required data is
a setup failure, never a successful skipped recording suite.

Downloaded data and its ZIP remain in Git-ignored `test-data/`, outside `build/`.
Deleting a build does not delete the recordings. Existing verified data works
offline; a retained verified ZIP can repopulate the package. Populate one checkout
from only one configure/fetch at a time. For offline transfer, download the exact
release ZIP, verify its size/hash against pack.json and extract it under
`test-data/`. See [the pack instructions](../tests/data/first-path/README.md).

Do not regenerate or alter recordings to make a failing test pass. Data changes
require a reviewed new immutable release version and updated hashes/expectations.
This contributor tooling neither generates recordings nor publishes releases.

## Public CI and maintainer qualification

PRs run quick tests with GCC and Clang, the Docker article suite, and the existing
quality checks. Pushes to main run full tests with both compilers. Actions' CI
workflow also has a **Run workflow** suite selector for manual runs on a branch.
The Docker article check still runs independently of that selector.
All jobs use standard GitHub-hosted Ubuntu runners with a read-only token.
There are no personal runners, private dependencies, uploaded build/WAV artifacts
or persistent Actions caches in this workflow. Results appear in job logs and
CTest output. Local CTest details remain in the build's `Testing/Temporary` folder.

Contributors are not expected to own an RT1170 or RT1064. Before accepting a change
that needs target qualification, the maintainer can bring the exact public PR
revision into a private integration branch and run the hardware suite there.
Record the public commit, private integration commit, firmware, rig, test/data
versions and outcome. Review the code before executing it on a private bench.
Do not automatically dispatch unreviewed fork code to a personal runner.

After software review and applicable hardware qualification, merge the original
public PR to preserve its authorship and review history, and retain its changes
in private development. If private testing requires a public fix, update the PR
and rerun affected checks. Later private-to-public releases must preserve accepted
public changes; do not replace public main with an older private snapshot.
Hardware evidence qualifies the tested integration, not every future revision.
