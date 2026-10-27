#include "record/binlog.hpp"

#include <chrono>
#include <cstring>

#include "common/rt.hpp"

namespace rcws {

bool BinLog::open(const std::string& path, const std::string& cfg_text, int cpu) {
  f_ = std::fopen(path.c_str(), "wb");
  if (!f_) return false;
  const char magic[8] = {'R', 'C', 'W', 'S', 'L', 'O', 'G', 0};
  const uint32_t ver = 1, n = uint32_t(cfg_text.size());
  std::fwrite(magic, 1, 8, f_);
  std::fwrite(&ver, 4, 1, f_);
  std::fwrite(&n, 4, 1, f_);
  std::fwrite(cfg_text.data(), 1, n, f_);
  stop_ = false;
  th_ = std::thread([this, cpu] { run(cpu); });
  return true;
}

void BinLog::close() {
  if (!f_) return;
  stop_ = true;
  if (th_.joinable()) th_.join();
  std::fclose(f_);
  f_ = nullptr;
}

void BinLog::write_raw(RecType t, const void* p, size_t n) {
  if (!f_) return;
  std::vector<uint8_t> b(4 + n);
  const uint16_t ty = uint16_t(t), len = uint16_t(n);
  std::memcpy(b.data(), &ty, 2);
  std::memcpy(b.data() + 2, &len, 2);
  std::memcpy(b.data() + 4, p, n);
  if (!q_.push(std::move(b))) dropped_++;
}

void BinLog::write_det(const DetectionBatch& b, uint32_t consumed_seq) {
  const size_t n = std::min<size_t>(b.dets.size(), 255);
  std::vector<uint8_t> buf(sizeof(RecDetHead) + n * sizeof(RecDet));
  RecDetHead h{};
  h.seq = b.frame_seq;
  h.consumed_seq = consumed_seq;
  h.t_sof = b.t_sof_ns;
  h.t_submit = b.t_submit_ns;
  h.t_done = b.t_done_ns;
  h.crop[0] = b.crop.x; h.crop[1] = b.crop.y; h.crop[2] = b.crop.w; h.crop[3] = b.crop.h;
  h.kind = uint8_t(b.kind);
  h.n = uint8_t(n);
  std::memcpy(buf.data(), &h, sizeof h);
  for (size_t i = 0; i < n; ++i) {
    RecDet d{};
    const auto& s = b.dets[i];
    d.box[0] = s.box.x; d.box[1] = s.box.y; d.box[2] = s.box.w; d.box[3] = s.box.h;
    d.score = s.score;
    d.cls = uint8_t(s.cls);
    std::memcpy(buf.data() + sizeof h + i * sizeof d, &d, sizeof d);
  }
  write_raw(RecType::DETBATCH, buf.data(), buf.size());
}

void BinLog::run(int cpu) {
  if (cpu >= 0) set_thread_rt(cpu, 0, "logger");
  while (true) {
    bool any = false;
    while (auto b = q_.pop()) {
      std::fwrite(b->data(), 1, b->size(), f_);
      any = true;
    }
    if (!any) {
      if (stop_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  std::fflush(f_);
}

}  // namespace rcws
