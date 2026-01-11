# constexpr_eval v1.5 — Community Edition (Final Release)

Single-header, zero-dependency constexpr arithmetic expression evaluator for C++17+ (C++23 recommended).

## Features

- Full operators with standard C++ precedence and associativity: `+ - * / % ^ & | ~ << >> ( )`
- Unary `+ - ~` (bitwise `~` only with `int_tag`)
- Advanced literals: decimal/int/float, scientific (`1e3`, `1.23e-4`), hex (`0xFF`), binary (`0b1010`), underscores (`1_000_000`)
- Constants: `pi`, `e`
- Math functions: `sin cos tan asin acos atan sinh cosh tanh sqrt log log10 log2 exp pow abs floor ceil round fmod hypot` (float modes only)
- Return types:
  - `double` (default) — floating-point math
  - `int64_t` via `int_tag` — enables bitwise operators and integer arithmetic
  - `long double` via `long_double_tag` — higher precision
- Configurable token buffer (`MaxTokens` default 1024)
- Compile-time error detection via `static_assert` (syntax, div-by-zero, unsupported, NaN/inf/overflow)

## Usage

```cpp
#include "constexpr_eval.hpp"

constexpr double d = ce::eval<"sinh(pi/2) + round(1.23e-4 + 0b1010)">();
static_assert(d > 11.0);

constexpr int64_t i = ce::eval<"0xFF & ~0b1010 | (0x10 << 2)", ce::int_tag>();
static_assert(i == 244);

constexpr long double ld = ce::eval<"hypot(3,4)", ce::long_double_tag>();
static_assert(ld == 5.0L);

## Limitations

Bitwise operators and hex/binary literals only with int_tag
Math functions and floating-point literals only in float modes
Functions/constants lowercase only
No variables or user-defined functions
Max ~1000 tokens (configurable)

## Bitcoin Blockchain Timestamp (Proof of Existence)

This header has been **fully timestamped and attested** on the Bitcoin blockchain using OpenTimestamps.

- Initial submission: January 10, 2026
- Full attestation completed: January 11, 2026

File: `constexpr_eval.hpp.ots` (included in repo)

Verify anytime with:
```bash
ots verify constexpr_eval.hpp.ots

MIT License — see LICENSE
Copyright © 2026 zenithcpp

[![Release](https://img.shields.io/github/v/release/zenithcpp/constexpr-eval?label=Latest%20Release)](https://github.com/zenithcpp/constexpr-eval/releases/latest)
[![License](https://img.shields.io/github/license/zenithcpp/constexpr-eval)](LICENSE)
[![Bitcoin Timestamp](https://img.shields.io/badge/Bitcoin-Timestamped-Jan%2010%2C%202026-orange)](constexpr_eval.hpp.ots)
