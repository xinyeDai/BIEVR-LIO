#pragma once

#include <bievr_lio/common.h>
#include <algorithm>
#include <array>
#include <bit>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace bievr {

// V1: 8-byte magic, then records of uint64 stamp_ns, uint64 point_count,
// followed by point_count float32 x/y/z/intensity tuples, in native little endian.
// Points are already deskewed in the IMU/body frame at stamp_ns.
inline constexpr char kScanMagic[8] = {'B','I','E','V','R','S','1','\0'};

class ScanArchiveWriter {
 public:
  explicit ScanArchiveWriter(const std::filesystem::path& path) {
    if constexpr (std::endian::native != std::endian::little)
      throw std::runtime_error("Scan archives require little-endian storage.");
    if (std::filesystem::exists(path))
      throw std::runtime_error("Scan archive already exists; move the previous recording: " + path.string());
    std::filesystem::create_directories(path.parent_path());
    output_.open(path, std::ios::binary);
    output_.write(kScanMagic, sizeof(kScanMagic));
    output_.flush();
    if (!output_) throw std::runtime_error("Cannot create scan archive: " + path.string());
    worker_ = std::thread([this] { run(); });
  }
  ~ScanArchiveWriter() { finish(); }
  void add(uint64_t stamp, const IntensityPointcloud& cloud) {
    std::unique_lock<std::mutex> lock(mutex_);
    // Bound memory without silently losing scans when disk is slower than playback.
    condition_.wait(lock, [this] { return queue_.size() < 32 || failed_ || stopping_; });
    if (failed_ || stopping_) throw std::runtime_error("Scan archive writer unavailable.");
    queue_.push_back({stamp, cloud});
    condition_.notify_all();
  }
  bool finish() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    return !failed_;
  }
 private:
  struct Frame { uint64_t stamp; IntensityPointcloud cloud; };
  void run() {
    for (;;) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) return;
        frame = std::move(queue_.front());
        queue_.pop_front();
        condition_.notify_all();
      }
      const uint64_t count = frame.cloud.size();
      output_.write(reinterpret_cast<const char*>(&frame.stamp), sizeof(frame.stamp));
      output_.write(reinterpret_cast<const char*>(&count), sizeof(count));
      for (size_t i = 0; i < count; ++i) {
        const std::array<float, 4> point{static_cast<float>(frame.cloud[i][0]),
            static_cast<float>(frame.cloud[i][1]), static_cast<float>(frame.cloud[i][2]),
            static_cast<float>(frame.cloud[i][3])};
        output_.write(reinterpret_cast<const char*>(point.data()), sizeof(point));
      }
      output_.flush();
      if (!output_) {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true;
        condition_.notify_all();
        return;
      }
    }
  }
  std::ofstream output_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Frame> queue_;
  std::thread worker_;
  bool stopping_ = false, failed_ = false;
};

class ScanArchiveReader {
 public:
  explicit ScanArchiveReader(const std::string& path) : input_(path, std::ios::binary) {
    if constexpr (std::endian::native != std::endian::little)
      throw std::runtime_error("Scan archives require little-endian storage.");
    std::array<char, 8> magic{};
    input_.read(magic.data(), magic.size());
    if (!input_ || !std::equal(magic.begin(), magic.end(), kScanMagic))
      throw std::runtime_error("Invalid or missing scan archive: " + path);
    remaining_ = std::filesystem::file_size(path) - magic.size();
  }
  bool next(uint64_t& stamp, IntensityPointcloud& cloud) {
    if (remaining_ == 0) return false;
    uint64_t count = 0;
    input_.read(reinterpret_cast<char*>(&stamp), sizeof(stamp));
    input_.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!input_ || remaining_ < 16 || count > (remaining_ - 16) / 16)
      throw std::runtime_error("Truncated scan archive record.");
    remaining_ -= 16 + count * 16;
    cloud.resize(count);
    for (size_t i = 0; i < count; ++i) {
      std::array<float, 4> point{};
      input_.read(reinterpret_cast<char*>(point.data()), sizeof(point));
      if (!input_) throw std::runtime_error("Cannot read scan archive points.");
      cloud[i] << point[0], point[1], point[2], point[3];
    }
    return true;
  }
 private:
  std::ifstream input_;
  uint64_t remaining_ = 0;
};
}  // namespace bievr
