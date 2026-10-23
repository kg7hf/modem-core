// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/manifest_json.hpp"

#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

namespace modem_test::manifest
{

namespace
{

using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;

constexpr std::uint8_t first_printable = 0x20U; // a space
constexpr std::uint8_t delete_character = 0x7FU;

class Parser
{
public:
    explicit Parser(std::string_view text) : mText(text) {}

    [[nodiscard]] const std::string& diagnostic() const noexcept { return mDiagnostic; }

    Result<Value> document()
    {
        if (mText.size() > maximum_bytes)
        {
            return fail("the manifest is larger than 1 MiB");
        }
        if (mText.starts_with("\xEF\xBB\xBF"))
        {
            return fail("a byte order mark is not accepted");
        }
        for (const char c : mText)
        {
            const auto byte = std::bit_cast<std::uint8_t>(c);
            if (byte >= delete_character || (byte < first_printable && c != '\t' && c != '\n' && c != '\r'))
            {
                return fail("only printable ASCII, tab, CR and LF are accepted");
            }
        }
        skip_space();
        auto value = parse_value(0U);
        if (!value)
        {
            return value;
        }
        skip_space();
        if (mAt != mText.size())
        {
            return fail("content after the top-level value");
        }
        return value;
    }

private:
    Status fail(std::string_view what)
    {
        // Report the first failure only, where it happened.
        if (mDiagnostic.empty())
        {
            std::size_t line = 1U;
            std::size_t column = 1U;
            for (std::size_t i = 0U; i < mAt && i < mText.size(); ++i)
            {
                if (mText[i] == '\n')
                {
                    ++line;
                    column = 1U;
                }
                else
                {
                    ++column;
                }
            }
            mDiagnostic = std::format("line {}, column {}: {}", line, column, what);
        }
        return Status{StatusCode::invalid_argument, "the manifest is not in the accepted form"};
    }

    void skip_space() noexcept
    {
        while (mAt < mText.size() && (mText[mAt] == ' ' || mText[mAt] == '\t' || mText[mAt] == '\n' || mText[mAt] == '\r'))
        {
            ++mAt;
        }
    }

    bool literal(std::string_view word) noexcept
    {
        if (mText.substr(mAt, word.size()) == word)
        {
            mAt += word.size();
            return true;
        }
        return false;
    }

    Result<Value> parse_value(std::size_t depth)
    {
        if (depth > maximum_depth)
        {
            return fail("nested more than 16 deep");
        }
        if (mAt >= mText.size())
        {
            return fail("the manifest ends where a value should be");
        }
        Value value;
        const char c = mText[mAt];
        if (c == '{')
        {
            return parse_object(depth);
        }
        if (c == '[')
        {
            return parse_array(depth);
        }
        if (c == '"')
        {
            auto text = parse_string();
            if (!text)
            {
                return text.error();
            }
            value.kind = Kind::string;
            value.text = std::move(text.value());
            return value;
        }
        if (literal("true"))
        {
            value.kind = Kind::boolean;
            value.boolean = true;
            return value;
        }
        if (literal("false"))
        {
            value.kind = Kind::boolean;
            return value;
        }
        if (literal("null"))
        {
            return value;
        }
        if (c == '-' || (c >= '0' && c <= '9'))
        {
            return parse_number();
        }
        return fail("not a value");
    }

    Result<Value> parse_object(std::size_t depth)
    {
        Value value;
        value.kind = Kind::object;
        ++mAt; // {
        skip_space();
        if (mAt < mText.size() && mText[mAt] == '}')
        {
            ++mAt;
            return value;
        }
        while (true)
        {
            skip_space();
            if (mAt >= mText.size() || mText[mAt] != '"')
            {
                return fail("an object member must start with a string key");
            }
            auto key = parse_string();
            if (!key)
            {
                return key.error();
            }
            if (value.find(key.value()) != nullptr)
            {
                return fail(std::format("duplicate key \"{}\"", key.value()));
            }
            skip_space();
            if (mAt >= mText.size() || mText[mAt] != ':')
            {
                return fail("':' expected after a key");
            }
            ++mAt;
            skip_space();
            auto member = parse_value(depth + 1U);
            if (!member)
            {
                return member;
            }
            value.members.push_back(Member{.key = std::move(key.value()), .value = std::move(member.value())});
            skip_space();
            if (mAt < mText.size() && mText[mAt] == ',')
            {
                ++mAt;
                continue;
            }
            if (mAt < mText.size() && mText[mAt] == '}')
            {
                ++mAt;
                return value;
            }
            return fail("',' or '}' expected in an object");
        }
    }

    Result<Value> parse_array(std::size_t depth)
    {
        Value value;
        value.kind = Kind::array;
        ++mAt; // [
        skip_space();
        if (mAt < mText.size() && mText[mAt] == ']')
        {
            ++mAt;
            return value;
        }
        while (true)
        {
            skip_space();
            auto item = parse_value(depth + 1U);
            if (!item)
            {
                return item;
            }
            value.items.push_back(std::move(item.value()));
            skip_space();
            if (mAt < mText.size() && mText[mAt] == ',')
            {
                ++mAt;
                continue;
            }
            if (mAt < mText.size() && mText[mAt] == ']')
            {
                ++mAt;
                return value;
            }
            return fail("',' or ']' expected in an array");
        }
    }

    Result<std::string> parse_string()
    {
        std::string text;
        ++mAt; // "
        while (mAt < mText.size())
        {
            const char c = mText[mAt++];
            if (c == '"')
            {
                return text;
            }
            if (c == '\t' || c == '\n' || c == '\r')
            {
                return fail("a control character inside a string");
            }
            if (c != '\\')
            {
                text.push_back(c);
                continue;
            }
            if (mAt >= mText.size())
            {
                break;
            }
            switch (const char escaped = mText[mAt++]; escaped)
            {
            case '"':
            case '\\':
            case '/':
                text.push_back(escaped);
                break;
            case 'b':
                text.push_back('\b');
                break;
            case 'f':
                text.push_back('\f');
                break;
            case 'n':
                text.push_back('\n');
                break;
            case 'r':
                text.push_back('\r');
                break;
            case 't':
                text.push_back('\t');
                break;
            case 'u':
                return fail("\\u escapes are not accepted: write the character");
            default:
                return fail("an unknown escape");
            }
        }
        return fail("the manifest ends inside a string");
    }

    Result<Value> parse_number()
    {
        // JSON's grammar: -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?
        const auto start = mAt;
        const auto digits = [this]
        {
            const auto from = mAt;
            while (mAt < mText.size() && mText[mAt] >= '0' && mText[mAt] <= '9')
            {
                ++mAt;
            }
            return mAt - from;
        };
        if (mText[mAt] == '-')
        {
            ++mAt;
        }
        if (mAt < mText.size() && mText[mAt] == '0')
        {
            ++mAt;
        }
        else if (digits() == 0U)
        {
            return fail("a number needs a digit");
        }
        if (mAt < mText.size() && mText[mAt] == '.')
        {
            ++mAt;
            if (digits() == 0U)
            {
                return fail("a fraction needs a digit");
            }
        }
        if (mAt < mText.size() && (mText[mAt] == 'e' || mText[mAt] == 'E'))
        {
            ++mAt;
            if (mAt < mText.size() && (mText[mAt] == '+' || mText[mAt] == '-'))
            {
                ++mAt;
            }
            if (digits() == 0U)
            {
                return fail("an exponent needs a digit");
            }
        }
        if (mAt < mText.size() && mText[mAt] >= '0' && mText[mAt] <= '9')
        {
            return fail("a leading zero");
        }
        Value value;
        value.kind = Kind::number;
        const auto token = mText.substr(start, mAt - start);
        const auto [stop, error] = std::from_chars(token.data(), token.data() + token.size(), value.number);
        if (error != std::errc{} || stop != token.data() + token.size() || !std::isfinite(value.number))
        {
            return fail("a number out of range");
        }
        return value;
    }

    std::string_view mText;
    std::string mDiagnostic;
    std::size_t mAt{};
};

} // namespace

const Value* Value::find(std::string_view key) const noexcept
{
    for (const auto& [name, member] : members)
    {
        if (name == key)
        {
            return &member;
        }
    }
    return nullptr;
}

Result<Value> parse(std::string_view text, std::string& diagnostic)
{
    Parser parser(text);
    auto value = parser.document();
    diagnostic = parser.diagnostic();
    return value;
}

Result<std::string> read_file(const std::string& path, std::string& diagnostic)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        diagnostic = std::format("{}: cannot be opened", path);
        return Status{StatusCode::io_error, "the manifest cannot be read"};
    }
    std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad())
    {
        diagnostic = std::format("{}: read failed", path);
        return Status{StatusCode::io_error, "the manifest cannot be read"};
    }
    return bytes;
}

} // namespace modem_test::manifest
