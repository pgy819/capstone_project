// Small, dependency-free helpers for the web page served by rcws_vision: SHA-1 + base64 for the
// WebSocket handshake (RFC 6455), WebSocket frame encode/decode, and minimal JSON field access for
// the short command messages the page sends. Pure functions, unit-tested (tests/test_web.cpp).
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rcws::web {

std::string sha1_raw(const std::string& data);   // 20 bytes
std::string base64(const std::string& raw);
// Sec-WebSocket-Accept value for a client key
std::string ws_accept(const std::string& client_key);

// Server -> client frame (unmasked, FIN). opcode 1 = text, 9/10 = ping/pong, 8 = close.
std::string ws_frame(const std::string& payload, uint8_t opcode = 1);

struct WsMsg {
  uint8_t opcode = 0;
  std::string payload;
};
// Pops complete client frames (masked, FIN; fragments are not used by browsers for short text)
// from the front of `buf`. Returns false on a protocol error (close the connection).
bool ws_parse(std::string* buf, std::vector<WsMsg>* out);

// HTTP request head helpers
std::string http_path(const std::string& head);                           // "GET /x HTTP/1.1" -> "/x"
std::string http_header(const std::string& head, const std::string& name);  // case-insensitive

// Minimal JSON: value of a top-level string / number field in a flat object, e.g. {"cmd":"jog","pan":0.5}
std::optional<std::string> json_str(const std::string& j, const std::string& key);
std::optional<double> json_num(const std::string& j, const std::string& key);
std::string json_escape(const std::string& s);

}  // namespace rcws::web
