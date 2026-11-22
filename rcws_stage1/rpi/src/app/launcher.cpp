// rcws_launcher - tiny always-on web page to start / stop the tracking program without SSH.
//
//   rcws_launcher --port 8000 --cmd "$HOME/rcws/go.sh" --log $HOME/rcws/logs/launcher.txt
//
// Page http://<pi>:8000/ : start / stop buttons, running time, last lines of the program output and a
// link to the operator page (rcws_vision --preview 8080). Stop = SIGINT to the program's process group
// (rcws_vision saves its log and clip as with Ctrl-C), SIGTERM after 8 s, SIGKILL after 12 s.
// Runs as a systemd user service (tools/pi/rcws-launcher.service). Uses no CPU while idle.
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>

#include "app/web_util.hpp"
#include "common/clock.hpp"

namespace {

volatile sig_atomic_t g_quit = 0;
void on_term(int) { g_quit = 1; }

const char* kPage = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>RCWS Launcher</title>
<style>
:root{--bg:#0f1216;--panel:#171b21;--line:#262c35;--text:#e6e9ee;--muted:#8b95a3;--ok:#3fb96b;--bad:#e5484d;--accent:#4c8dff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,"DejaVu Sans",sans-serif}
main{max-width:720px;margin:0 auto;padding:16px}h1{font-size:18px;margin:4px 0 14px}
section{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:12px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
button{font:inherit;color:#fff;border:0;border-radius:8px;padding:12px 20px;font-weight:650;cursor:pointer}
#start{background:var(--ok)}#stop{background:var(--bad)}button:disabled{opacity:.4;cursor:default}
#state{font-size:17px;font-weight:650}.muted{color:var(--muted)}a{color:var(--accent);font-weight:600}
pre{background:#0c0f13;border:1px solid var(--line);border-radius:8px;padding:8px;font:12.5px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,"DejaVu Sans Mono",monospace;max-height:340px;overflow:auto;white-space:pre-wrap;word-break:break-word;margin:0}
</style></head><body><main>
<h1>RCWS Stage 1 Launcher</h1>
<section><div class="row" style="justify-content:space-between"><span id="state">-</span><span class="muted" id="up"></span></div>
<div class="row" style="margin-top:12px"><button id="start">Start Tracking</button><button id="stop">Stop</button>
<a id="open" href="#" style="display:none">Open Control UI -&gt;</a></div></section>
<section><div class="muted" style="margin-bottom:6px">Program Output (tail)</div><pre id="out"></pre></section>
<p class="muted">Stopping saves logs and clips. STM32 holds servos after 0.5 s and turns torque off after 10 s.</p>
</main><script>
const $=(i)=>document.getElementById(i);$("open").href=`${location.protocol}//${location.hostname}:8080/`;
async function poll(){try{const r=await fetch("/state");const s=await r.json();
$("state").textContent=s.running?"Running":(s.stopping?"Stopping...":"Stopped"+(s.exit!==null?` (exit ${s.exit})`:""));
$("state").style.color=s.running?"var(--ok)":"";$("up").textContent=s.running?`Uptime ${Math.floor(s.uptime)}s`:"";
$("start").disabled=s.running||s.stopping;$("stop").disabled=!s.running||s.stopping;$("open").style.display=s.running?"":"none";
const o=$("out"),b=o.scrollTop+o.clientHeight>=o.scrollHeight-4;o.textContent=s.tail;if(b)o.scrollTop=o.scrollHeight;}catch(e){$("state").textContent="Launcher offline";}}
$("start").onclick=()=>fetch("/start",{method:"POST"}).then(poll);
$("stop").onclick=()=>{if(confirm("Stop the tracking program?"))fetch("/stop",{method:"POST"}).then(poll)};
poll();setInterval(poll,1000);
</script></body></html>)HTML";

bool send_all(int fd, const std::string& s) {
  size_t o = 0;
  while (o < s.size()) {
    const ssize_t k = ::send(fd, s.data() + o, s.size() - o, MSG_NOSIGNAL);
    if (k <= 0) return false;
    o += size_t(k);
  }
  return true;
}
void reply(int fd, const char* status, const char* type, const std::string& body) {
  char h[200];
  std::snprintf(h, sizeof h, "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
                status, type, body.size());
  send_all(fd, h + body);
}
std::string tail(const std::string& path, size_t lines) {
  std::ifstream f(path);
  std::deque<std::string> q;
  std::string l;
  while (std::getline(f, l)) {
    // libcamera / progress lines with \r: keep the last segment
    const size_t r = l.rfind('\r');
    q.push_back(r == std::string::npos ? l : l.substr(r + 1));
    if (q.size() > lines) q.pop_front();
  }
  std::string o;
  for (const auto& s : q) o += s + "\n";
  return o;
}

struct Child {
  pid_t pid = -1;
  int64_t t_start = 0, t_stop = 0;
  bool stopping = false;
  int last_exit = -999;
};

}  // namespace

int main(int argc, char** argv) {
  int port = 8000;
  std::string cmd, logp = "launcher.txt";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--port") port = std::atoi(argv[i + 1]);
    else if (k == "--cmd") cmd = argv[i + 1];
    else if (k == "--log") logp = argv[i + 1];
  }
  if (cmd.empty()) { std::cerr << "usage: rcws_launcher --cmd SCRIPT [--port 8000] [--log FILE]\n"; return 2; }
  std::signal(SIGTERM, on_term);
  std::signal(SIGINT, on_term);
  const int ls = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons(uint16_t(port));
  if (::bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
    std::cerr << "bind " << port << ": " << std::strerror(errno) << "\n";
    return 1;
  }
  ::listen(ls, 8);
  std::cerr << "rcws_launcher: http://<pi>:" << port << "/  runs " << cmd << "\n";
  Child ch;
  while (!g_quit) {
    // reap / escalate
    if (ch.pid > 0) {
      int st = 0;
      if (waitpid(ch.pid, &st, WNOHANG) == ch.pid) {
        ch.last_exit = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + WTERMSIG(st);
        ch.pid = -1;
        ch.stopping = false;
      } else if (ch.stopping) {
        const double s = (rcws::now_ns() - ch.t_stop) * 1e-9;
        if (s > 12) kill(-ch.pid, SIGKILL);
        else if (s > 8) kill(-ch.pid, SIGTERM);
      }
    }
    pollfd p{ls, POLLIN, 0};
    if (::poll(&p, 1, 200) <= 0) continue;
    const int fd = ::accept(ls, nullptr, nullptr);
    if (fd < 0) continue;
    std::string req;
    char buf[2048];
    for (int k = 0; k < 20 && req.find("\r\n\r\n") == std::string::npos; ++k) {
      pollfd q{fd, POLLIN, 0};
      if (::poll(&q, 1, 50) <= 0) continue;
      const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
      if (n <= 0) break;
      req.append(buf, size_t(n));
    }
    const std::string path = rcws::web::http_path(req);
    const bool post = req.rfind("POST", 0) == 0;
    if (path == "/" || path == "/index.html") {
      reply(fd, "200 OK", "text/html; charset=utf-8", kPage);
    } else if (path == "/state") {
      char j[160];
      const bool run = ch.pid > 0 && !ch.stopping;
      std::snprintf(j, sizeof j, "{\"running\":%s,\"stopping\":%s,\"uptime\":%.1f,\"exit\":%s,\"tail\":\"", run ? "true" : "false",
                    ch.stopping ? "true" : "false", run ? (rcws::now_ns() - ch.t_start) * 1e-9 : 0.0,
                    ch.last_exit == -999 ? "null" : std::to_string(ch.last_exit).c_str());
      reply(fd, "200 OK", "application/json; charset=utf-8", std::string(j) + rcws::web::json_escape(tail(logp, 40)) + "\"}");
    } else if (path == "/start" && post) {
      if (ch.pid <= 0) {
        const pid_t pid = fork();
        if (pid == 0) {
          setsid();   // own process group: stop reaches go.sh and rcws_vision together
          const int out = ::open(logp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
          if (out >= 0) { dup2(out, 1); dup2(out, 2); ::close(out); }
          ::close(ls);
          execl("/bin/bash", "bash", cmd.c_str(), static_cast<char*>(nullptr));
          _exit(127);
        }
        if (pid > 0) { ch.pid = pid; ch.t_start = rcws::now_ns(); ch.stopping = false; }
      }
      reply(fd, "200 OK", "application/json", "{}");
    } else if (path == "/stop" && post) {
      if (ch.pid > 0 && !ch.stopping) { kill(-ch.pid, SIGINT); ch.stopping = true; ch.t_stop = rcws::now_ns(); }
      reply(fd, "200 OK", "application/json", "{}");
    } else {
      reply(fd, "404 Not Found", "text/plain", "not found");
    }
    ::close(fd);
  }
  if (ch.pid > 0) { kill(-ch.pid, SIGINT); sleep(3); kill(-ch.pid, SIGKILL); }
  return 0;
}
