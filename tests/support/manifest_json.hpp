// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// =============================================================================
// manifest_json.hpp - a reader for the bench's data manifests, not a JSON library
// =============================================================================
// The article recordings come with a manifest (cases.json) that says what each
// recording is and what the receiver must do with it. This reads the restricted
// form of JSON those manifests are written in, and rejects anything else by name:
//
//   - ASCII only (bytes 0x20 to 0x7E, and tab, CR and LF between tokens); no byte
//     order mark.
//   - Objects with unique keys; arrays; strings; numbers; true, false and null.
//   - String escapes \" \\ \/ \b \f \n \r \t. No \u escapes: write the character.
//   - Numbers as JSON writes them, and finite.
//   - Nesting at most 16 deep; at most 1 MiB; nothing after the top-level value.
//
// A manifest outside that form is a setup error, never a default: parse() returns
// the failure, and `diagnostic` says where (line and column) and what.
// =============================================================================

namespace modem_test::manifest
{

enum class Kind : std::uint8_t
{
    null,
    boolean,
    number,
    string,
    array,
    object,
};

struct Member;

struct Value
{
    Kind kind{Kind::null};
    bool boolean{};
    double number{};
    std::string text{};          // a string's characters, escapes resolved
    std::vector<Value> items{};  // an array's elements
    std::vector<Member> members; // an object's members, in file order

    // The member called `key`, or nothing.
    [[nodiscard]] const Value* find(std::string_view key) const noexcept;
};

// One member of an object. (Not a std::pair, which needs Value complete where Value
// declares its members; and `members` has no {} initializer, which Clang checks where
// Value ends, before Member is complete.)
struct Member
{
    std::string key;
    Value value;
};

inline constexpr std::size_t maximum_depth = 16U;
inline constexpr std::size_t maximum_bytes = std::size_t{1024U} * 1024U;

[[nodiscard]] modem::common::Result<Value> parse(std::string_view text, std::string& diagnostic);

// The file's bytes, or the failure (diagnostic names the file).
[[nodiscard]] modem::common::Result<std::string> read_file(const std::string& path, std::string& diagnostic);

} // namespace modem_test::manifest
