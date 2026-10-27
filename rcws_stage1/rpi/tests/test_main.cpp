#include "test_util.hpp"

#include <cstdio>

std::vector<TestCase>& registry() {
  static std::vector<TestCase> r;
  return r;
}
int g_failures = 0;

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int run = 0;
  for (auto& t : registry()) {
    if (filter && std::string(t.name).find(filter) == std::string::npos) continue;
    const int before = g_failures;
    std::printf("[ RUN  ] %s\n", t.name);
    t.fn();
    std::printf("[ %s ] %s\n", g_failures == before ? " OK " : "FAIL", t.name);
    ++run;
  }
  std::printf("%d tests, %d failed checks\n", run, g_failures);
  return g_failures ? 1 : 0;
}
