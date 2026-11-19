#include "app/web_util.hpp"

#include <cctype>
#include <cstdlib>
#include <cstring>

namespace rcws::web {

namespace {
uint32_t rol(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }
std::string lower(std::string s) {
  for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
// position just after "key": in a flat JSON object, or npos
size_t value_pos(const std::string& j, const std::string& key) {
  const std::string k = "\"" + key + "\"";
  size_t p = 0;
  while ((p = j.find(k, p)) != std::string::npos) {
    size_t q = p + k.size();
    while (q < j.size() && std::isspace(static_cast<unsigned char>(j[q]))) ++q;
    if (q < j.size() && j[q] == ':') {
      ++q;
      while (q < j.size() && std::isspace(static_cast<unsigned char>(j[q]))) ++q;
      return q;
    }
    p = q;
  }
  return std::string::npos;
}
}  // namespace

std::string sha1_raw(const std::string& data) {
  uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
  std::string m = data;
  const uint64_t bits = uint64_t(data.size()) * 8;
  m.push_back(char(0x80));
  while (m.size() % 64 != 56) m.push_back('\0');
  for (int i = 7; i >= 0; --i) m.push_back(char((bits >> (i * 8)) & 0xFF));
  for (size_t off = 0; off < m.size(); off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      const auto* b = reinterpret_cast<const unsigned char*>(m.data() + off + 4 * i);
      w[i] = (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[2]) << 8) | b[3];
    }
    for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
      uint32_t f, k;
      if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
      else { f = b ^ c ^ d; k = 0xCA62C1D6; }
      const uint32_t t = rol(a, 5) + f + e + k + w[i];
      e = d; d = c; c = rol(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
  }
  std::string out(20, '\0');
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 4; ++j) out[size_t(4 * i + j)] = char((h[i] >> (24 - 8 * j)) & 0xFF);
  return out;
}

std::string base64(const std::string& raw) {
  static const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < raw.size(); i += 3) {
    const uint32_t v = (uint32_t(uint8_t(raw[i])) << 16) | (uint32_t(uint8_t(raw[i + 1])) << 8) | uint8_t(raw[i + 2]);
    out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63]; out += t[(v >> 6) & 63]; out += t[v & 63];
  }
  if (i + 1 == raw.size()) {
    const uint32_t v = uint32_t(uint8_t(raw[i])) << 16;
    out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63]; out += "==";
  } else if (i + 2 == raw.size()) {
    const uint32_t v = (uint32_t(uint8_t(raw[i])) << 16) | (uint32_t(uint8_t(raw[i + 1])) << 8);
    out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63]; out += t[(v >> 6) & 63]; out += '=';
  }
  return out;
}

std::string ws_accept(const std::string& client_key) {
  return base64(sha1_raw(client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
}

std::string ws_frame(const std::string& payload, uint8_t opcode) {
  std::string f;
  f.push_back(char(0x80 | (opcode & 0x0F)));
  const size_t n = payload.size();
  if (n < 126) {
    f.push_back(char(n));
  } else if (n < 65536) {
    f.push_back(char(126));
    f.push_back(char((n >> 8) & 0xFF));
    f.push_back(char(n & 0xFF));
  } else {
    f.push_back(char(127));
    for (int i = 7; i >= 0; --i) f.push_back(char((uint64_t(n) >> (8 * i)) & 0xFF));
  }
  return f + payload;
}

bool ws_parse(std::string* buf, std::vector<WsMsg>* out) {
  for (;;) {
    const std::string& b = *buf;
    if (b.size() < 2) return true;
    const uint8_t b0 = uint8_t(b[0]), b1 = uint8_t(b[1]);
    if (!(b1 & 0x80)) return false;            // client frames must be masked
    uint64_t n = b1 & 0x7F;
    size_t p = 2;
    if (n == 126) {
      if (b.size() < 4) return true;
      n = (uint64_t(uint8_t(b[2])) << 8) | uint8_t(b[3]);
      p = 4;
    } else if (n == 127) {
      if (b.size() < 10) return true;
      n = 0;
      for (int i = 0; i < 8; ++i) n = (n << 8) | uint8_t(b[size_t(2 + i)]);
      p = 10;
    }
    if (n > (1u << 16)) return false;          // commands are tiny
    if (b.size() < p + 4 + n) return true;
    const uint8_t* mask = reinterpret_cast<const uint8_t*>(b.data() + p);
    WsMsg m;
    m.opcode = b0 & 0x0F;
    m.payload.resize(size_t(n));
    for (size_t i = 0; i < n; ++i) m.payload[i] = char(uint8_t(b[p + 4 + i]) ^ mask[i & 3]);
    buf->erase(0, p + 4 + size_t(n));
    out->push_back(std::move(m));
  }
}

std::string http_path(const std::string& head) {
  const size_t a = head.find(' ');
  if (a == std::string::npos) return "";
  const size_t b = head.find(' ', a + 1);
  if (b == std::string::npos) return "";
  std::string p = head.substr(a + 1, b - a - 1);
  const size_t q = p.find('?');
  return q == std::string::npos ? p : p.substr(0, q);
}

std::string http_header(const std::string& head, const std::string& name) {
  const std::string h = lower(head), k = "\n" + lower(name) + ":";
  const size_t p = h.find(k);
  if (p == std::string::npos) return "";
  size_t s = p + k.size();
  while (s < head.size() && head[s] == ' ') ++s;
  size_t e = head.find('\r', s);
  if (e == std::string::npos) e = head.find('\n', s);
  return head.substr(s, e == std::string::npos ? std::string::npos : e - s);
}

std::optional<std::string> json_str(const std::string& j, const std::string& key) {
  size_t p = value_pos(j, key);
  if (p == std::string::npos || p >= j.size() || j[p] != '"') return std::nullopt;
  std::string v;
  for (++p; p < j.size() && j[p] != '"'; ++p) {
    if (j[p] == '\\' && p + 1 < j.size()) ++p;
    v.push_back(j[p]);
  }
  if (p >= j.size()) return std::nullopt;
  return v;
}

std::optional<double> json_num(const std::string& j, const std::string& key) {
  const size_t p = value_pos(j, key);
  if (p == std::string::npos) return std::nullopt;
  char* end = nullptr;
  const double v = std::strtod(j.c_str() + p, &end);
  if (end == j.c_str() + p) return std::nullopt;
  return v;
}

std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': break;
      case '\t': o += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else o += c;
    }
  }
  return o;
}

}  // namespace rcws::web
