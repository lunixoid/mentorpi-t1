#ifndef MENTORPI_VOICE_WAV_WRITER_HPP_
#define MENTORPI_VOICE_WAV_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace mentorpi_voice {

// SD033 I10: PCM16 little-endian WAV writer. RIFF and data sizes are patched on close() and in
// the destructor, so a file closed normally is readable by read_wav_pcm16.
class WavWriter {
 public:
  static constexpr size_t kHeaderBytes = 44;

  WavWriter() = default;
  WavWriter(const WavWriter&) = delete;
  WavWriter& operator=(const WavWriter&) = delete;
  ~WavWriter() { close(); }

  bool open(const std::string& path, uint32_t rate, uint16_t channels) {
    close();
    if (rate == 0 || channels == 0) {
      return false;
    }
    out_.open(path, std::ios::binary | std::ios::trunc);
    if (!out_.is_open()) {
      return false;
    }
    rate_ = rate;
    channels_ = channels;
    data_bytes_ = 0;
    write_header();
    if (!out_) {
      out_.close();
      return false;
    }
    return true;
  }

  bool is_open() const { return out_.is_open(); }

  bool write(const int16_t* samples, size_t count) {
    if (!out_.is_open()) {
      return false;
    }
    if (count > (kMaxDataBytes - data_bytes_) / 2) {
      return false;
    }
    buf_.resize(count * 2);
    for (size_t i = 0; i < count; ++i) {
      const auto u = static_cast<uint16_t>(samples[i]);
      buf_[2 * i] = static_cast<char>(u & 0xFF);
      buf_[2 * i + 1] = static_cast<char>((u >> 8) & 0xFF);
    }
    out_.write(buf_.data(), static_cast<std::streamsize>(buf_.size()));
    if (!out_) {
      return false;
    }
    data_bytes_ += buf_.size();
    return true;
  }

  size_t bytes() const { return kHeaderBytes + data_bytes_; }

  bool close() {
    if (!out_.is_open()) {
      return true;
    }
    out_.seekp(0);
    write_header();
    const bool ok = static_cast<bool>(out_);
    out_.close();
    return ok && !out_.fail();
  }

 private:
  static constexpr size_t kMaxDataBytes = 0xFFFFFFFFu - 36u;

  void put_le16(uint16_t v) {
    const char b[2] = {static_cast<char>(v & 0xFF), static_cast<char>((v >> 8) & 0xFF)};
    out_.write(b, 2);
  }

  void put_le32(uint32_t v) {
    const char b[4] = {static_cast<char>(v & 0xFF), static_cast<char>((v >> 8) & 0xFF),
                       static_cast<char>((v >> 16) & 0xFF), static_cast<char>((v >> 24) & 0xFF)};
    out_.write(b, 4);
  }

  void write_header() {
    const auto data = static_cast<uint32_t>(data_bytes_);
    out_.write("RIFF", 4);
    put_le32(36u + data);
    out_.write("WAVE", 4);
    out_.write("fmt ", 4);
    put_le32(16);
    put_le16(1);
    put_le16(channels_);
    put_le32(rate_);
    put_le32(rate_ * channels_ * 2u);
    put_le16(static_cast<uint16_t>(channels_ * 2u));
    put_le16(16);
    out_.write("data", 4);
    put_le32(data);
  }

  std::ofstream out_;
  uint32_t rate_{0};
  uint16_t channels_{0};
  size_t data_bytes_{0};
  std::vector<char> buf_;
};

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_WAV_WRITER_HPP_
