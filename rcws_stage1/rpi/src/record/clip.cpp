#include "record/clip.hpp"

#include <cstring>

#include "record/binlog.hpp"

namespace rcws {
namespace {

#pragma pack(push, 1)
struct ClipHeader {
  char magic[8];
  uint32_t version, mw, mh, lw, lh, n;
};
struct ClipMeta {
  uint32_t seq;
  int64_t t_sof;
  int32_t exp;
  float gain, lens;
};
#pragma pack(pop)

void copy_plane(uint8_t* dst, const uint8_t* src, int w, int h, int stride) {
  for (int j = 0; j < h; ++j) std::memcpy(dst + size_t(j) * w, src + size_t(j) * stride, w);
}

}  // namespace

bool ClipRecorder::init(const FrameView& f, double seconds, int fps) {
  const size_t n = size_t(seconds * fps);
  if (n == 0 || !f.main_y) return false;
  mw_ = f.main_w; mh_ = f.main_h;
  lw_ = f.lo_y ? f.lo_w : 0; lh_ = f.lo_y ? f.lo_h : 0;
  frame_bytes_ = size_t(mw_) * mh_ * 3 / 2 + size_t(lw_) * lh_ * 3 / 2;
  ring_.resize(n);
  for (auto& s : ring_) s.data.resize(frame_bytes_);  // allocate up front: no allocation in the loop
  return true;
}

void ClipRecorder::push(const FrameView& f) {
  if (ring_.empty()) return;
  Slot& s = ring_[head_];
  s.seq = f.seq; s.t_sof = f.t_sof_ns; s.exp = f.exposure_us; s.gain = f.analog_gain; s.lens = f.lens_position;
  uint8_t* p = s.data.data();
  copy_plane(p, f.main_y, mw_, mh_, f.main_stride); p += size_t(mw_) * mh_;
  const size_t c = size_t(mw_ / 2) * (mh_ / 2);
  if (f.main_u) { copy_plane(p, f.main_u, mw_ / 2, mh_ / 2, f.main_uv_stride); copy_plane(p + c, f.main_v, mw_ / 2, mh_ / 2, f.main_uv_stride); }
  else std::memset(p, 128, 2 * c);
  p += 2 * c;
  if (lw_) {
    copy_plane(p, f.lo_y, lw_, lh_, f.lo_stride); p += size_t(lw_) * lh_;
    const size_t lc = size_t(lw_ / 2) * (lh_ / 2);
    if (f.lo_u) { copy_plane(p, f.lo_u, lw_ / 2, lh_ / 2, f.lo_uv_stride); copy_plane(p + lc, f.lo_v, lw_ / 2, lh_ / 2, f.lo_uv_stride); }
    else std::memset(p, 128, 2 * lc);
  }
  head_ = (head_ + 1) % ring_.size();
  count_ = std::min(count_ + 1, ring_.size());
}

bool ClipRecorder::save(const std::string& path) const {
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  ClipHeader h{{'R', 'C', 'W', 'S', 'C', 'L', 'I', 'P'}, 1, uint32_t(mw_), uint32_t(mh_), uint32_t(lw_), uint32_t(lh_), uint32_t(count_)};
  std::fwrite(&h, sizeof h, 1, f);
  const size_t start = (head_ + ring_.size() - count_) % ring_.size();
  for (size_t i = 0; i < count_; ++i) {
    const Slot& s = ring_[(start + i) % ring_.size()];
    ClipMeta m{s.seq, s.t_sof, s.exp, s.gain, s.lens};
    std::fwrite(&m, sizeof m, 1, f);
    std::fwrite(s.data.data(), 1, frame_bytes_, f);
  }
  std::fclose(f);
  return true;
}

bool ReplaySource::start(std::string* err) {
  f_ = std::fopen(path_.c_str(), "rb");
  if (!f_) { *err = "cannot open " + path_; return false; }
  ClipHeader h{};
  if (std::fread(&h, sizeof h, 1, f_) != 1 || std::memcmp(h.magic, "RCWSCLIP", 8) != 0) { *err = "not a clip"; return false; }
  mw_ = int(h.mw); mh_ = int(h.mh); lw_ = int(h.lw); lh_ = int(h.lh); n_ = h.n;
  buf_.resize(size_t(mw_) * mh_ * 3 / 2 + size_t(lw_) * lh_ * 3 / 2);
  return true;
}

bool ReplaySource::grab(Frame* fr, int) {
  if (!f_ || read_ >= n_) { eos_ = true; return false; }
  ClipMeta m{};
  if (std::fread(&m, sizeof m, 1, f_) != 1 || std::fread(buf_.data(), 1, buf_.size(), f_) != buf_.size()) {
    eos_ = true;
    return false;
  }
  ++read_;
  cur_seq_ = m.seq;
  FrameView& v = fr->view;
  v = FrameView{};
  v.seq = m.seq; v.t_sof_ns = m.t_sof; v.exposure_us = m.exp; v.analog_gain = m.gain; v.lens_position = m.lens;
  uint8_t* p = buf_.data();
  v.main_y = p; v.main_w = mw_; v.main_h = mh_; v.main_stride = mw_; p += size_t(mw_) * mh_;
  v.main_u = p; v.main_v = p + size_t(mw_ / 2) * (mh_ / 2); v.main_uv_stride = mw_ / 2; p += size_t(mw_) * mh_ / 2;
  if (lw_) {
    v.lo_y = p; v.lo_w = lw_; v.lo_h = lh_; v.lo_stride = lw_; p += size_t(lw_) * lh_;
    v.lo_u = p; v.lo_v = p + size_t(lw_ / 2) * (lh_ / 2); v.lo_uv_stride = lw_ / 2;
  }
  fr->handle = nullptr;
  fr->dropped = 0;
  return true;
}

void ReplaySource::stop() {
  if (f_) std::fclose(f_);
  f_ = nullptr;
}

ReplayDetector::ReplayDetector(const std::string& path, cv::Size input, std::function<uint32_t()> cur)
    : in_(input), cur_(std::move(cur)) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return;
  char magic[8];
  uint32_t ver = 0, n = 0;
  if (std::fread(magic, 1, 8, f) != 8 || std::memcmp(magic, "RCWSLOG", 7) != 0) { std::fclose(f); return; }
  if (std::fread(&ver, 4, 1, f) != 1 || std::fread(&n, 4, 1, f) != 1) { std::fclose(f); return; }
  std::fseek(f, long(n), SEEK_CUR);
  std::vector<uint8_t> payload;
  uint16_t ty = 0, len = 0;
  while (std::fread(&ty, 2, 1, f) == 1 && std::fread(&len, 2, 1, f) == 1) {
    payload.resize(len);
    if (std::fread(payload.data(), 1, len, f) != len) break;
    if (ty != uint16_t(RecType::DETBATCH) || len < sizeof(RecDetHead)) continue;
    RecDetHead hd;
    std::memcpy(&hd, payload.data(), sizeof hd);
    DetectionBatch b;
    b.frame_seq = hd.seq; b.t_sof_ns = hd.t_sof; b.t_submit_ns = hd.t_submit; b.t_done_ns = hd.t_done;
    b.kind = DetKind(hd.kind);
    b.crop = {hd.crop[0], hd.crop[1], hd.crop[2], hd.crop[3]};
    for (int i = 0; i < hd.n && sizeof hd + (i + 1) * sizeof(RecDet) <= len; ++i) {
      RecDet d;
      std::memcpy(&d, payload.data() + sizeof hd + i * sizeof d, sizeof d);
      b.dets.push_back({{d.box[0], d.box[1], d.box[2], d.box[3]}, d.score, ObjClass(d.cls)});
    }
    by_consumed_.emplace(hd.consumed_seq, std::move(b));
  }
  std::fclose(f);
  ok_ = true;
}

std::optional<DetectionBatch> ReplayDetector::poll() {
  const uint32_t cur = cur_();
  // drop batches that belong to frames before the replay window
  while (!by_consumed_.empty() && by_consumed_.begin()->first < cur) by_consumed_.erase(by_consumed_.begin());
  auto it = by_consumed_.find(cur);
  if (it == by_consumed_.end()) return std::nullopt;
  DetectionBatch b = std::move(it->second);
  by_consumed_.erase(it);
  return b;
}

}  // namespace rcws
