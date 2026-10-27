#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

struct TestCase {
  const char* name;
  std::function<void()> fn;
};
std::vector<TestCase>& registry();
extern int g_failures;

struct TestReg {
  TestReg(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};
#define TEST(name)                                   \
  static void name();                                \
  static TestReg reg_##name(#name, name);            \
  static void name()

#define CHECK(c)                                                                  \
  do {                                                                            \
    if (!(c)) { std::printf("  CHECK failed %s:%d: %s\n", __FILE__, __LINE__, #c); ++g_failures; } \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
  do {                                                                                                 \
    const double va = (a), vb = (b);                                                                   \
    if (!(std::abs(va - vb) <= (tol))) {                                                               \
      std::printf("  CHECK_NEAR failed %s:%d: %s=%.6g %s=%.6g tol %.3g\n", __FILE__, __LINE__, #a, va, #b, vb, double(tol)); \
      ++g_failures;                                                                                    \
    }                                                                                                  \
  } while (0)
