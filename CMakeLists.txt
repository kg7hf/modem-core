# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Paul R. Decker (KG7HF)

cmake_minimum_required(VERSION 3.20)
project(modem_core_tx LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Quality-tooling switches (all OFF by default; CI turns each one on in its
#     own job, so a normal local build stays fast and portable). ---
option(MODEM_WERROR      "Treat warnings as errors"                       OFF)
option(MODEM_CLANG_TIDY  "Run clang-tidy during the build (.clang-tidy)"  OFF)
option(MODEM_COVERAGE    "Instrument for gcov / llvm-cov coverage"        OFF)
option(MODEM_STACK_USAGE "Emit -fstack-usage .su reports (bounded stack)" OFF)

if(MODEM_CLANG_TIDY)
    # Prefer clang-tidy 19: version 18 cannot parse std::expected from libstdc++.
    find_program(CLANG_TIDY NAMES clang-tidy-19 clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY}")
endif()

add_library(modem_tx STATIC
    modem/common/status.cpp
    modem/common/types.cpp
    modem/common/dsp/timing_recovery.cpp
    modem/common/fec/convolutional_k7.cpp
    modem/common/waveform/waveform.cpp
    modem/m110a/transmitter.cpp
    modem/m110a/body_waveform.cpp
    modem/m110a/body_audio_stream_frontend.cpp
    modem/m110a/scrambler.cpp
    modem/m110a/demapper.cpp
    modem/m110a/interleaver.cpp
)
target_include_directories(modem_tx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# The runtime rules from "The rules I build by" travel with the library: no exceptions
# and no RTTI. They are PUBLIC, so tx_demo and any other consumer build the same way.
if(MSVC)
    target_compile_options(modem_tx PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/GR-> $<$<COMPILE_LANGUAGE:CXX>:/EHs-> $<$<COMPILE_LANGUAGE:CXX>:/EHc->)
    target_compile_definitions(modem_tx PUBLIC $<$<COMPILE_LANGUAGE:CXX>:_HAS_EXCEPTIONS=0>)
else()
    target_compile_options(modem_tx PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions> $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
    # No fused multiply-add: a*b+c rounds twice on every target, so the float results do not
    # depend on the compiler or the optimization level (Part 3, "The compiler changed the math").
    target_compile_options(modem_tx PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-ffp-contract=off>)
endif()

add_executable(tx_demo tests/tx_demo.cpp)
target_link_libraries(tx_demo PRIVATE modem_tx)

# The demo prints with C++23 std::print. GCC 14's libstdc++ keeps the console
# support std::print needs in the separate libstdc++exp library (later releases
# fold it into libstdc++), so link it only when the toolchain needs it.
include(CheckCXXSourceCompiles)
set(MODEM_PRINT_PROBE "#include <print>\nint main() { std::println(\"{}\", 1); }")
check_cxx_source_compiles("${MODEM_PRINT_PROBE}" MODEM_PRINT_LINKS)
if(NOT MODEM_PRINT_LINKS)
    set(CMAKE_REQUIRED_LIBRARIES stdc++exp)
    check_cxx_source_compiles("${MODEM_PRINT_PROBE}" MODEM_PRINT_LINKS_WITH_STDCXXEXP)
    unset(CMAKE_REQUIRED_LIBRARIES)
    if(NOT MODEM_PRINT_LINKS_WITH_STDCXXEXP)
        message(FATAL_ERROR "C++23 std::print does not link with this toolchain (GCC 14+ or Clang 19+ required)")
    endif()
    target_link_libraries(tx_demo PRIVATE stdc++exp)
endif()

# --- The test bench (from Part 3): Catch2 tests over the real transmitter. ---
# Catch2 is fetched at configure time, fixed to one release by its commit. The modem
# builds without exceptions, and so does Catch2: a failed REQUIRE ends that test's
# process, and each test case runs as its own CTest test.
include(FetchContent)
set(CATCH_CONFIG_DISABLE_EXCEPTIONS ON CACHE BOOL "Build Catch2 without exceptions, like the modem" FORCE)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG 317ac1ed4c0bb6e6b91eafc817e05c488feffcb3 # v3.16.0
    SYSTEM)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

add_executable(front_end_tests
    tests/front_end_tests.cpp
    tests/support/front_end_harness.cpp
    tests/support/impairments.cpp
    tests/support/test_channel.cpp
    tests/support/test_waveform.cpp)
target_link_libraries(front_end_tests PRIVATE modem_tx Catch2::Catch2WithMain)
if(NOT MODEM_PRINT_LINKS)
    target_link_libraries(front_end_tests PRIVATE stdc++exp)
endif()
# A fully static MinGW build keeps another toolchain's runtime DLLs on the PATH (Git for
# Windows carries an older libstdc++) from being loaded by mistake.
if(MINGW)
    target_link_options(front_end_tests PRIVATE -static)
endif()

# --- Apply the warning/tooling flags to our own targets (GCC/Clang). ---
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    foreach(target modem_tx tx_demo front_end_tests)
        target_compile_options(${target} PRIVATE -Wall -Wextra)
        if(MODEM_WERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
        if(MODEM_COVERAGE)
            target_compile_options(${target} PRIVATE --coverage -O0 -g)
            target_link_options(${target} PRIVATE --coverage)
        endif()
        if(MODEM_STACK_USAGE)
            target_compile_options(${target} PRIVATE -fstack-usage)
        endif()
    endforeach()
endif()

# --- Golden regression tests for the 600L "Hi" burst. tx_golden_symbols checks
#     every transmitted tribit (an integer hash, the same on every platform), so
#     one changed bit in framing, coding, interleaving, mapping, probes or
#     scrambling fails it. tx_golden checks the rendered audio's length and first
#     samples, printed to five decimals so that last-bit differences between
#     math libraries cannot fail it (the filter taps are still computed with the
#     library's cosine when the modulator starts). ---
enable_testing()
add_test(NAME tx_golden COMMAND tx_demo Hi)
set_tests_properties(tx_golden PROPERTIES
    PASS_REGULAR_EXPRESSION "rendered 460800 samples.*first: 0\\.39122 0\\.45497 0\\.49206")
add_test(NAME tx_golden_symbols COMMAND tx_demo Hi)
set_tests_properties(tx_golden_symbols PROPERTIES
    PASS_REGULAR_EXPRESSION "23040 transmitted tribits, FNV-1a 6097e9da9fbb6255")

# Every Catch2 test case is its own CTest test, so each shows up in VS Code's Testing
# panel. Hidden cases run only when asked for: ctest -L long, or ctest -L report.
catch_discover_tests(front_end_tests TEST_PREFIX "front_end: ")
catch_discover_tests(front_end_tests TEST_SPEC "[long]" TEST_PREFIX "long: " PROPERTIES LABELS long)
catch_discover_tests(front_end_tests TEST_SPEC "[report]" TEST_PREFIX "report: " PROPERTIES LABELS report)
