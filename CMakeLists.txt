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
    modem/m110a/scrambler.cpp
    modem/m110a/demapper.cpp
    modem/m110a/interleaver.cpp
)
target_include_directories(modem_tx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(tx_demo tests/tx_demo.cpp)
target_link_libraries(tx_demo PRIVATE modem_tx)

# --- Apply the warning/tooling flags to our own targets (GCC/Clang). ---
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    foreach(target modem_tx tx_demo)
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

# --- Golden regression tests for the 600L "Hi" burst. tx_golden_symbols pins
#     every transmitted tribit (an integer hash, the same on every platform), so
#     one changed bit in framing, coding, interleaving, mapping, probes or
#     scrambling fails it. tx_golden pins the rendered audio's length and first
#     samples, printed to five decimals so that last-bit differences between
#     platform math libraries cannot fail it. ---
enable_testing()
add_test(NAME tx_golden COMMAND tx_demo Hi)
set_tests_properties(tx_golden PROPERTIES
    PASS_REGULAR_EXPRESSION "rendered 460800 samples.*first: 0\\.39122 0\\.45497 0\\.49206")
add_test(NAME tx_golden_symbols COMMAND tx_demo Hi)
set_tests_properties(tx_golden_symbols PROPERTIES
    PASS_REGULAR_EXPRESSION "23040 transmitted tribits, FNV-1a 6097e9da9fbb6255")
