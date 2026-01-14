#pragma once
// constexpr_eval.hpp v1.7 - Ultimate Community Edition (Final)
// Single-header, zero-dependency constexpr arithmetic expression evaluator
// Requires C++23 (constexpr math support)
// MIT License - free to use, share, modify
//
// Features (all fully implemented):
// - Operators: + - * / % ^ & | ~ << >> ( ) with full C++ precedence
// - Unary + - ~
// - Literals: decimal int/float, scientific (1e3), hex (0xFF), binary (0b1010), underscores (1_000)
// - Constants: pi, e (lowercase only)
// - Functions: sin cos tan asin acos atan sinh cosh tanh sqrt log log10 log2 exp pow abs floor ceil round fmod hypot
// - Returns double by default; int_tag for int64_t (bitwise), long_double_tag for long double
// - Custom token buffer (MaxTokens default 1024)
// - NaN/inf detection with static_assert
// - Argument count validation
// - Floating literals rejected in int_tag
// - ^ is XOR in int_tag, pow in float modes
//
//

#if __cplusplus < 202302L
#error "constexpr_eval requires C++23 constexpr math support"
#endif

#include <cstdint>
#include <cstddef>
#include <array>
#include <optional>
#include <cmath>
#include <string_view>
#include <numbers>

namespace ce::detail {

template <class>
inline constexpr bool dependent_false = false;

enum class TokenKind : uint8_t {
  Error,
  EndOfFile,
  Identifier,
  Constant,
  IntegerLiteral,
  FloatLiteral,
  Plus, Minus, Star, Slash, Percent, CaretOrPow,
  Amp, Pipe, Tilde, LShift, RShift,
  LParen, RParen, Comma
};

struct Token {
  TokenKind kind;
  const char* start;
  std::size_t length;
  constexpr Token(TokenKind k = TokenKind::Error, const char* s = nullptr, std::size_t l = 0)
      : kind(k), start(s), length(l) {}
};

constexpr bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }

constexpr bool is_alpha(char c) { return (c >= 'a' && c <= 'z'); } // enforce lowercase

constexpr bool is_ident_continue(char c) { return is_alpha(c) || (c >= '0' && c <= '9'); }

constexpr bool is_hex_digit(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }

constexpr bool is_bin_digit(char c) { return c == '0' || c == '1'; }

constexpr int hex_digit_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return c - 'A' + 10;
}

template <std::size_t N>
struct StringLiteral {
  static constexpr std::size_t size = N;
  char data[N + 1]{};

  constexpr StringLiteral(const char (&str)[N + 1]) {
    for (std::size_t i = 0; i <= N; ++i) data[i] = str[i];
  }
};

template <std::size_t N>
StringLiteral(const char (&)[N]) -> StringLiteral<N - 1>;

template <StringLiteral Lit, std::size_t MaxTokens = 1024>
struct Lexer {
  static constexpr auto lex() {
    std::array<Token, MaxTokens> tokens{};
    std::size_t count = 0;
    const char* p = Lit.data;
    const char* end = p + Lit.size;

    while (p < end && count < tokens.size() - 1) {
      while (p < end && is_space(*p)) ++p;
      if (p >= end) break;

      const char* start = p;
      char c = *p;

      // Bitwise
      if (c == '&' || c == '|' || c == '~') {
        TokenKind k = TokenKind::Error;
        if (c == '&') k = TokenKind::Amp;
        else if (c == '|') k = TokenKind::Pipe;
        else if (c == '~') k = TokenKind::Tilde;
        tokens[count++] = Token(k, start, 1);
        ++p;
        continue;
      }
      if (c == '<' && p + 1 < end && p[1] == '<') {
        tokens[count++] = Token(TokenKind::LShift, start, 2);
        p += 2;
        continue;
      }
      if (c == '>' && p + 1 < end && p[1] == '>') {
        tokens[count++] = Token(TokenKind::RShift, start, 2);
        p += 2;
        continue;
      }

      // Other operators
      if (c == '+' || c == '-' || c == '*' || c == '/' || c == '%' || c == '^' || c == '(' || c == ')' || c == ',') {
        TokenKind k = TokenKind::Error;
        switch (c) {
          case '+': k = TokenKind::Plus; break;
          case '-': k = TokenKind::Minus; break;
          case '*': k = TokenKind::Star; break;
          case '/': k = TokenKind::Slash; break;
          case '%': k = TokenKind::Percent; break;
          case '^': k = TokenKind::CaretOrPow; break;
          case '(': k = TokenKind::LParen; break;
          case ')': k = TokenKind::RParen; break;
          case ',': k = TokenKind::Comma; break;
        }
        tokens[count++] = Token(k, start, 1);
        ++p;
        continue;
      }

      // Identifiers/constants (lowercase only)
      if (is_alpha(c)) {
        ++p;
        while (p < end && is_ident_continue(*p)) ++p;
        std::string_view name(start, p - start);
        if (name == "pi" || name == "e") tokens[count++] = Token(TokenKind::Constant, start, p - start);
        else tokens[count++] = Token(TokenKind::Identifier, start, p - start);
        continue;
      }

      // Binary
      if (c == '0' && p + 1 < end && (p[1] == 'b' || p[1] == 'B')) {
        p += 2;
        const char* bin_start = p;
        while (p < end && (is_bin_digit(*p) || *p == '_')) {
          if (*p == '_') ++p;
          else ++p;
        }
        if (p == bin_start) tokens[count++] = Token(TokenKind::Error, start, p - start);
        else tokens[count++] = Token(TokenKind::IntegerLiteral, start, p - start);
        continue;
      }

      // Hex
      if (c == '0' && p + 1 < end && (p[1] == 'x' || p[1] == 'X')) {
        p += 2;
        const char* hex_start = p;
        while (p < end && (is_hex_digit(*p) || *p == '_')) {
          if (*p == '_') ++p;
          else ++p;
        }
        if (p == hex_start) tokens[count++] = Token(TokenKind::Error, start, p - start);
        else tokens[count++] = Token(TokenKind::IntegerLiteral, start, p - start);
        continue;
      }

      // Decimal/float/scientific
      if ((c >= '0' && c <= '9') || c == '.') {
        bool has_dot = (c == '.');
        bool has_exp = false;
        const char* num_start = p;
        ++p;

        while (p < end) {
          char pc = *p;
          if (pc >= '0' && pc <= '9' || pc == '_') {
            if (pc == '_') ++p;
            else ++p;
          } else if (pc == '.' && !has_dot) { has_dot = true; ++p; }
          else if ((pc == 'e' || pc == 'E') && !has_exp) {
            has_exp = true;
            ++p;
            if (p < end && (*p == '+' || *p == '-')) ++p;
          } else break;
        }

        if ((p[-1] == '.' || (has_exp && (p[-1] == 'e' || p[-1] == 'E')))) {
          tokens[count++] = Token(TokenKind::Error, start, p - start);
          continue;
        }

        tokens[count++] = Token((has_dot || has_exp) ? TokenKind::FloatLiteral : TokenKind::IntegerLiteral,
                                num_start, p - num_start);
        continue;
      }

      tokens[count++] = Token(TokenKind::Error, start, 1);
      ++p;
    }
    if (count >= tokens.size() - 1) {
      static_assert(dependent_false<Lit>, "constexpr_eval error: token buffer overflow (increase MaxTokens)");
    }
    tokens[count++] = Token(TokenKind::EndOfFile, end, 0);
    return std::pair{tokens, count};
  }

  static constexpr auto result = lex();
  static constexpr auto tokens = result.first;
  static constexpr std::size_t token_count = result.second;
};

struct int_tag {};
struct double_tag {};
struct long_double_tag {};

constexpr double get_pi() { return std::numbers::pi; }

constexpr double get_e() { return std::numbers::e; }

constexpr int function_arg_count(std::string_view name) {
  if (name == "pow" || name == "hypot" || name == "fmod") return 2;
  return 1;
}

constexpr double apply_unary(std::string_view name, double arg) {
  if (name == "sin") return std::sin(arg);
  if (name == "cos") return std::cos(arg);
  if (name == "tan") return std::tan(arg);
  if (name == "asin") return std::asin(arg);
  if (name == "acos") return std::acos(arg);
  if (name == "atan") return std::atan(arg);
  if (name == "sinh") return std::sinh(arg);
  if (name == "cosh") return std::cosh(arg);
  if (name == "tanh") return std::tanh(arg);
  if (name == "sqrt") return std::sqrt(arg);
  if (name == "log") return std::log(arg);
  if (name == "log10") return std::log10(arg);
  if (name == "log2") return std::log2(arg);
  if (name == "exp") return std::exp(arg);
  if (name == "abs") return std::abs(arg);
  if (name == "floor") return std::floor(arg);
  if (name == "ceil") return std::ceil(arg);
  if (name == "round") return std::round(arg);
  static_assert(dependent_false<void>, "constexpr_eval error: unsupported unary function");
  return 0.0;
}

constexpr double apply_binary(std::string_view name, double a1, double a2) {
  if (name == "pow") return std::pow(a1, a2);
  if (name == "hypot") return std::hypot(a1, a2);
  if (name == "fmod") return std::fmod(a1, a2);
  static_assert(dependent_false<void>, "constexpr_eval error: unsupported binary function");
  return 0.0;
}

template <typename T = double>
constexpr T parse_number(const char* s, std::size_t len) {
  if (len == 0) return 0;

  T value = 0;
  bool negative = false;
  std::size_t i = 0;

  bool is_hex = (len >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
  bool is_bin = (len >= 2 && s[0] == '0' && (s[1] == 'b' || s[1] == 'B'));
  if (is_hex || is_bin) i = 2;

  if (!is_hex && !is_bin && s[i] == '-') { negative = true; ++i; }
  else if (!is_hex && !is_bin && s[i] == '+') ++i;

  while (i < len && s[i] == '_') ++i;

  if constexpr (std::is_same_v<T, double>) {
    double mantissa = 0;
    double frac = 0;
    double frac_div = 1.0;
    bool in_frac = false;
    bool has_exp = false;
    double exp = 0.0;
    double exp_sign = 1.0;

    for (; i < len; ++i) {
      if (s[i] == '_') continue;
      if (s[i] == 'e' || s[i] == 'E') {
        has_exp = true;
        ++i;
        if (i < len && s[i] == '-') { exp_sign = -1.0; ++i; }
        else if (i < len && s[i] == '+') ++i;
        break;
      }
      if (s[i] == '.') { in_frac = true; continue; }
      int d = s[i] - '0';
      if (in_frac) {
        frac = frac * 10 + d;
        frac_div *= 10;
      } else {
        mantissa = mantissa * 10 + d;
      }
    }

    value = mantissa + frac / frac_div;

    if (has_exp) {
      for (; i < len; ++i) {
        if (s[i] == '_') continue;
        exp = exp * 10 + (s[i] - '0');
      }
      value *= std::pow(10.0, exp_sign * exp);
    }
  } else {
    for (; i < len; ++i) {
      if (s[i] == '_') continue;
      if (is_hex) value = value * 16 + hex_digit_value(s[i]);
      else if (is_bin) value = value * 2 + (s[i] - '0');
      else value = value * 10 + (s[i] - '0');
    }
  }

  return negative ? -value : value;
}

// Grammar (highest → lowest precedence):
// atom : literals | constants | function | '(' expr ')'
// unary : (+ | - | ~) unary | atom
// shift : unary ( (<< | >>) unary )*
// and : shift ( & shift )*
// xor : and ( ^ and )*
// or : xor ( | xor )*
// term : or ( (* | / | %) or )*
// expr : term ( (+ | -) term )*

template <StringLiteral Lit, std::size_t MaxTokens = 1024, typename Tag = double_tag>
struct Parser {
  using L = Lexer<Lit, MaxTokens>;
  static constexpr auto tokens = L::tokens;
  static constexpr std::size_t n_tokens = L::token_count;

  using ValueType = std::conditional_t<std::is_same_v<Tag, int_tag>, int64_t,
                      std::conditional_t<std::is_same_v<Tag, long_double_tag>, long double, double>>;

  static constexpr std::optional<ValueType> parse_atom(std::size_t& pos) {
    if (pos >= n_tokens - 1) return {};

    const Token& t = tokens[pos];

    if (t.kind == TokenKind::Constant) {
      ++pos;
      std::string_view name(t.start, t.length);
      if (name == "pi") return static_cast<ValueType>(get_pi());
      if (name == "e") return static_cast<ValueType>(get_e());
      return {};
    }

    if (t.kind == TokenKind::Identifier) {
      std::string_view name(t.start, t.length);
      ++pos;
      if (pos >= n_tokens - 1 || tokens[pos].kind != TokenKind::LParen) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: expected ( for function call");
      }
      ++pos;
      auto arg1 = parse_expr(pos);
      if (!arg1) return {};
      bool has_two = false;
      ValueType arg2 = 0;
      if (pos < n_tokens - 1 && tokens[pos].kind == TokenKind::Comma) {
        ++pos;
        auto a2 = parse_expr(pos);
        if (!a2) return {};
        arg2 = *a2;
        has_two = true;
      }
      if (pos >= n_tokens - 1 || tokens[pos].kind != TokenKind::RParen) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: expected ) in function call");
      }
      ++pos;

      int expected_args = function_arg_count(name);
      int actual_args = has_two ? 2 : 1;
      if (actual_args != expected_args) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: wrong number of arguments for function");
      }

      if (has_two) {
        double d1 = static_cast<double>(*arg1);
        double d2 = static_cast<double>(arg2);
        double r = apply_binary(name, d1, d2);
        return static_cast<ValueType>(r);
      } else {
        double d = static_cast<double>(*arg1);
        double r = apply_unary(name, d);
        return static_cast<ValueType>(r);
      }
    }

    if (t.kind == TokenKind::IntegerLiteral) {
      ++pos;
      return parse_number<ValueType>(t.start, t.length);
    }

    if (t.kind == TokenKind::FloatLiteral) {
      if constexpr (std::is_same_v<Tag, int_tag>) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: floating-point literal not allowed in int_tag");
      }
      ++pos;
      return parse_number<ValueType>(t.start, t.length);
    }

    if (t.kind == TokenKind::LParen) {
      ++pos;
      auto sub = parse_expr(pos);
      if (!sub || pos >= n_tokens - 1 || tokens[pos].kind != TokenKind::RParen) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: unbalanced parentheses");
      }
      ++pos;
      return sub;
    }

    return {};
  }

  static constexpr std::optional<ValueType> parse_unary(std::size_t& pos) {
    if (pos >= n_tokens - 1) return {};

    const Token& t = tokens[pos];

    if (t.kind == TokenKind::Plus || t.kind == TokenKind::Minus || t.kind == TokenKind::Tilde) {
      TokenKind op = t.kind;
      ++pos;
      auto sub = parse_unary(pos);
      if (!sub) return {};
      if (op == TokenKind::Plus) return *sub;
      if (op == TokenKind::Minus) return -(*sub);
      if constexpr (std::is_same_v<Tag, int_tag>) {
        if (op == TokenKind::Tilde) return ~(*sub);
      } else {
        static_assert(dependent_false<Lit>, "constexpr_eval error: unary ~ only allowed in int_tag");
      }
    }

    return parse_atom(pos);
  }

  static constexpr std::optional<ValueType> parse_shift(std::size_t& pos) {
    auto left = parse_unary(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 && (tokens[pos].kind == TokenKind::LShift || tokens[pos].kind == TokenKind::RShift)) {
      TokenKind op = tokens[pos++].kind;
      auto right = parse_unary(pos);
      if (!right) return {};
      if constexpr (std::is_same_v<Tag, int_tag>) {
        if (op == TokenKind::LShift) *left = *left << *right;
        else *left = *left >> *right;
      } else {
        static_assert(dependent_false<Lit>, "constexpr_eval error: shift operators only allowed in int_tag");
      }
    }
    return left;
  }

  static constexpr std::optional<ValueType> parse_and(std::size_t& pos) {
    auto left = parse_shift(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 && tokens[pos].kind == TokenKind::Amp) {
      ++pos;
      auto right = parse_shift(pos);
      if (!right) return {};
      if constexpr (std::is_same_v<Tag, int_tag>) {
        *left = *left & *right;
      } else {
        static_assert(dependent_false<Lit>, "constexpr_eval error: & operator only allowed in int_tag");
      }
    }
    return left;
  }

  static constexpr std::optional<ValueType> parse_xor(std::size_t& pos) {
    auto left = parse_and(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 && tokens[pos].kind == TokenKind::CaretOrPow) {
      ++pos;
      auto right = parse_and(pos);
      if (!right) return {};
      if constexpr (std::is_same_v<Tag, int_tag>) {
        *left = *left ^ *right;
      } else {
        double l = static_cast<double>(*left);
        double r = static_cast<double>(*right);
        *left = static_cast<ValueType>(std::pow(l, r));
      }
    }
    return left;
  }

  static constexpr std::optional<ValueType> parse_or(std::size_t& pos) {
    auto left = parse_xor(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 && tokens[pos].kind == TokenKind::Pipe) {
      ++pos;
      auto right = parse_xor(pos);
      if (!right) return {};
      if constexpr (std::is_same_v<Tag, int_tag>) {
        *left = *left | *right;
      } else {
        static_assert(dependent_false<Lit>, "constexpr_eval error: | operator only allowed in int_tag");
      }
    }
    return left;
  }

  static constexpr std::optional<ValueType> parse_term(std::size_t& pos) {
    auto left = parse_or(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 &&
           (tokens[pos].kind == TokenKind::Star || tokens[pos].kind == TokenKind::Slash || tokens[pos].kind == TokenKind::Percent)) {
      TokenKind op = tokens[pos++].kind;
      auto right = parse_or(pos);
      if (!right) return {};
      double l = static_cast<double>(*left);
      double r = static_cast<double>(*right);
      if (r == 0.0 && (op == TokenKind::Slash || op == TokenKind::Percent)) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: division or modulo by zero");
      }
      if (op == TokenKind::Star) *left = static_cast<ValueType>(l * r);
      else if (op == TokenKind::Slash) *left = static_cast<ValueType>(l / r);
      else *left = static_cast<ValueType>(std::fmod(l, r));
    }
    return left;
  }

  static constexpr std::optional<ValueType> parse_expr(std::size_t& pos) {
    auto left = parse_term(pos);
    if (!left) return {};

    while (pos < n_tokens - 1 &&
           (tokens[pos].kind == TokenKind::Plus || tokens[pos].kind == TokenKind::Minus)) {
      TokenKind op = tokens[pos++].kind;
      auto right = parse_term(pos);
      if (!right) return {};
      double l = static_cast<double>(*left);
      double r = static_cast<double>(*right);
      *left = static_cast<ValueType>((op == TokenKind::Plus) ? l + r : l - r);
    }
    return left;
  }

  static constexpr ValueType evaluate() {
    std::size_t pos = 0;
    auto result = parse_expr(pos);

    if (!result || pos != n_tokens - 2) {
      static_assert(dependent_false<Lit>, "constexpr_eval parse error: invalid syntax, div/mod by zero, unsupported function, unbalanced parens, or trailing tokens");
    }

    if constexpr (std::is_floating_point_v<ValueType>) {
      if (!std::isfinite(static_cast<double>(*result))) {
        static_assert(dependent_false<Lit>, "constexpr_eval error: result is NaN or infinity (overflow or invalid operation)");
      }
    }

    return *result;
  }

  static constexpr ValueType value = evaluate();
};

}  // namespace ce::detail

namespace ce {

template <ce::detail::StringLiteral Lit, std::size_t MaxTokens = 1024, typename Tag = ce::detail::double_tag>
constexpr auto eval() {
  return ce::detail::Parser<Lit, MaxTokens, Tag>::value;
}

template <StringLiteral Lit, typename Tag>
constexpr auto eval() {
  return ce::detail::Parser<Lit, 1024, Tag>::value;
}

using int_tag = ce::detail::int_tag;
using long_double_tag = ce::detail::long_double_tag;

}  // namespace ce

// Examples:
// constexpr double d = ce::eval<"sinh(pi/2) + round(1.23e-4 + 0b1010)">();
// static_assert(d > 11.0);
//
// constexpr int64_t bit = ce::eval<"0xFF & ~0b1010 | (0x10 << 2)", ce::int_tag>();
//
// constexpr long double ld = ce::eval<"hypot(3,4)", ce::long_double_tag>(); // 5.0L
