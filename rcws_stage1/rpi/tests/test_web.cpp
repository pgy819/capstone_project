// Operator web page: WebSocket handshake / framing and JSON helpers, the operator layer (modes,
// dead-man jog, home, software ESTOP), and the perception operator commands (stop tracking a
// captured object, select another one) on the two-phone synthetic scene.
#include <cmath>
#include <set>

#include "app/operator.hpp"
#include "app/web_util.hpp"
#include "estimation/camera_model.hpp"
#include "perception/perception.hpp"
#include "sim/synthetic.hpp"
#include "test_util.hpp"

using namespace rcws;

namespace {
std::string hex(const std::string& raw) {
  static const char* d = "0123456789abcdef";
  std::string o;
  for (unsigned char c : raw) { o += d[c >> 4]; o += d[c & 15]; }
  return o;
}
std::string client_frame(const std::string& payload, uint8_t opcode = 1) {   // masked, as a browser sends
  std::string f;
  f.push_back(char(0x80 | opcode));
  const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
  if (payload.size() < 126) f.push_back(char(0x80 | payload.size()));
  else { f.push_back(char(0x80 | 126)); f.push_back(char(payload.size() >> 8)); f.push_back(char(payload.size() & 0xFF)); }
  for (uint8_t m : mask) f.push_back(char(m));
  for (size_t i = 0; i < payload.size(); ++i) f.push_back(char(uint8_t(payload[i]) ^ mask[i & 3]));
  return f;
}
constexpr int64_t kMs = 1000000;
}  // namespace

TEST(web_sha1_base64_handshake) {
  CHECK(hex(web::sha1_raw("abc")) == "a9993e364706816aba3e25717850c26c9cd0d89d");
  CHECK(hex(web::sha1_raw("")) == "da39a3ee5e6b4b0d3255bfef95601890afd80709");
  CHECK(hex(web::sha1_raw(std::string(1000, 'a'))) == "291e9a6c66994949b57ba5e650361e98fc36b1ba");
  CHECK(web::base64("Man") == "TWFu" && web::base64("Ma") == "TWE=" && web::base64("M") == "TQ==");
  CHECK(web::ws_accept("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");   // RFC 6455 example
  const std::string head = "GET /ws?x=1 HTTP/1.1\r\nHost: pi\r\nsec-websocket-key: abc==\r\nUpgrade: websocket\r\n\r\n";
  CHECK(web::http_path(head) == "/ws" && web::http_header(head, "Sec-WebSocket-Key") == "abc==");
}

TEST(web_frames_and_json) {
  std::string buf = client_frame("{\"cmd\":\"jog\",\"pan\":-0.5,\"tilt\":1}") + client_frame(std::string(300, 'x'));
  buf += client_frame("{\"cmd\":\"sel").substr(0, 5);   // incomplete frame stays in the buffer
  std::vector<web::WsMsg> m;
  CHECK(web::ws_parse(&buf, &m));
  CHECK(m.size() == 2 && m[0].opcode == 1 && m[1].payload.size() == 300);
  CHECK(buf.size() == 5);
  CHECK(web::json_str(m[0].payload, "cmd").value_or("") == "jog");
  CHECK_NEAR(web::json_num(m[0].payload, "pan").value_or(9), -0.5, 1e-12);
  CHECK_NEAR(web::json_num(m[0].payload, "tilt").value_or(9), 1.0, 1e-12);
  CHECK(!web::json_num(m[0].payload, "id").has_value());
  std::string unmasked = web::ws_frame("hi");
  CHECK(unmasked.size() == 4 && uint8_t(unmasked[0]) == 0x81 && unmasked[1] == 2);
  std::vector<web::WsMsg> bad;
  CHECK(!web::ws_parse(&unmasked, &bad));        // server-style (unmasked) frame from a client: refused
  CHECK(web::ws_frame(std::string(70000, 'a')).size() == 70000 + 10);
  CHECK(web::json_escape("a\"b\\c\n") == "a\\\"b\\\\c\\n");
}

TEST(web_operator_modes) {
  Operator op;
  AimOut a;
  a.mode = AimMode::FOLLOW;
  a.psi = 0.05;
  int64_t t = 1000 * kMs;
  LinkAim_t f = op.frame(a, 1, 0, t, nullptr);
  CHECK(f.mode == LINK_MODE_FOLLOW && f.flags == 0 && f.err_pan > 0);           // AUTO: aim passes
  CHECK(op.set_mode(OpMode::MANUAL, t));
  f = op.frame(a, 2, 0, t, nullptr);
  CHECK(f.mode == LINK_MODE_HOLD && (f.flags & LINK_F_OPERATOR_HOLD) && f.err_pan == 0);
  // dead-man jog: alive for 0.3 s after the last message
  op.jog(1.0, -0.5, t);
  f = op.frame(a, 3, 0, t + 100 * kMs, nullptr);
  CHECK(f.mode == LINK_MODE_JOG && f.rate_pan == int16_t(std::lround(20.0 / 360 * 4096)) && f.rate_tilt < 0);
  f = op.frame(a, 4, 0, t + 400 * kMs, nullptr);
  CHECK(f.mode == LINK_MODE_HOLD && (f.flags & LINK_F_OPERATOR_HOLD));
  // home until the STM32 reports both goals at the centre
  op.home(t);
  LinkStatusView st;
  st.valid = true;
  st.t_ns = t + 10 * kMs;
  st.s.pan_goal = 2300;
  st.s.tilt_goal = 2048;
  CHECK(op.frame(a, 5, 0, t + 20 * kMs, &st).mode == LINK_MODE_HOME);
  st.s.pan_goal = 2050;
  CHECK(op.frame(a, 6, 0, t + 30 * kMs, &st).mode == LINK_MODE_HOLD && !op.homing());
  // software ESTOP: every frame asks for it, mode changes refused until cleared
  op.estop(t);
  f = op.frame(a, 7, 0, t, nullptr);
  CHECK((f.flags & LINK_F_ESTOP_REQ) && f.mode == LINK_MODE_HOLD);
  CHECK(!op.set_mode(OpMode::AUTO, t));
  op.jog(1, 0, t);
  CHECK(op.frame(a, 8, 0, t, nullptr).mode == LINK_MODE_HOLD);
  op.estop_clear(t);
  f = op.frame(a, 9, 0, t + 100 * kMs, nullptr);
  CHECK(!(f.flags & LINK_F_ESTOP_REQ) && (f.flags & LINK_F_ESTOP_CLEAR) && op.mode() == OpMode::MANUAL);
  CHECK(!(op.frame(a, 10, 0, t + 600 * kMs, nullptr).flags & LINK_F_ESTOP_CLEAR));
  CHECK(op.set_mode(OpMode::AUTO, t));
  CHECK(op.frame(a, 11, 0, t + 700 * kMs, nullptr).mode == LINK_MODE_FOLLOW);
}

TEST(web_stop_and_select_target) {
  Config cfg;
  int frames = 0;
  bool ok = false;
  SynthTraj tr = make_scenario("parallel", cfg.cam, &frames, &ok);
  CHECK(ok);
  SyntheticSource src(cfg.cam, tr, std::max(frames, 900), 1);
  OracleDetector det(src, {640, 640}, OracleCfg{}, 2);
  CameraModel cam(cfg.intr);
  Perception per(cfg, cam, &det, nullptr, [](int64_t) { return GimbalPose{}; });
  std::string err;
  src.start(&err);
  Frame fr;
  int first = -1, second = -1, phase = 0, n = 0, relock_first = 0, reselected = 0;
  while (src.grab(&fr, 0)) {
    const auto c = per.on_frame(fr.view);
    src.release(fr, c);
    ++n;
    const bool tracking = per.state() == TrackState::TRACK && per.lock_id() >= 0;
    if (phase == 0 && tracking) {                 // first capture: operator answers "stop tracking"
      first = per.lock_id();
      per.op_stop_target(-1);
      phase = 1;
    } else if (phase == 1) {
      if (per.lock_id() == first) relock_first++;
      if (tracking && per.lock_id() != first) {   // the other phone is acquired instead
        second = per.lock_id();
        phase = 2;
        per.op_select(first);                     // operator picks the first one again
      }
    } else if (phase == 2 && tracking && per.lock_id() == first) {
      reselected = n;
      phase = 3;
    }
  }
  std::printf("  frames %d: first lock #%d, after stop -> #%d, reselected at frame %d\n", n, first, second, reselected);
  CHECK(first >= 0);
  CHECK(second >= 0 && second != first);
  CHECK(relock_first == 0);
  CHECK(phase == 3);
}
