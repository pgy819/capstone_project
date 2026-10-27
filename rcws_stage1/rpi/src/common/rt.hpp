// Thread placement helpers (Sec 4.1). Best effort: failures are reported, not fatal,
// so the same binary runs on a development PC without privileges.
#pragma once
#include <cstdio>
#include <pthread.h>
#include <sched.h>
#include <cstring>

namespace rcws {

// prio > 0 -> SCHED_FIFO with that priority; prio == 0 -> keep SCHED_OTHER.
inline bool set_thread_rt(int cpu, int prio, const char* name) {
  bool ok = true;
  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (int e = pthread_setaffinity_np(pthread_self(), sizeof(set), &set)) {
      std::fprintf(stderr, "[rt] %s: affinity cpu%d failed: %s\n", name, cpu, std::strerror(e));
      ok = false;
    }
  }
  if (prio > 0) {
    sched_param sp{};
    sp.sched_priority = prio;
    if (int e = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
      std::fprintf(stderr, "[rt] %s: SCHED_FIFO %d failed: %s (run with CAP_SYS_NICE)\n", name, prio,
                   std::strerror(e));
      ok = false;
    }
  }
#ifdef __linux__
  pthread_setname_np(pthread_self(), name);
#endif
  return ok;
}

}  // namespace rcws
