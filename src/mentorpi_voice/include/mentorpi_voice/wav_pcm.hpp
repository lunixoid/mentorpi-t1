#ifndef MENTORPI_VOICE_WAV_PCM_HPP_
#define MENTORPI_VOICE_WAV_PCM_HPP_

#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace mentorpi_voice {

struct PcmClip {
  uint32_t sample_rate{0};
  uint16_t channels{0};
  std::vector<int16_t> samples;
};

namespace detail {

inline uint16_t read_le16(const char* p) {
  const auto* u = reinterpret_cast<const unsigned char*>(p);
  return static_cast<uint16_t>(u[0] | (static_cast<uint16_t>(u[1]) << 8));
}

inline uint32_t read_le32(const char* p) {
  const auto* u = reinterpret_cast<const unsigned char*>(p);
  return static_cast<uint32_t>(u[0]) | (static_cast<uint32_t>(u[1]) << 8) |
         (static_cast<uint32_t>(u[2]) << 16) | (static_cast<uint32_t>(u[3]) << 24);
}

}  // namespace detail

inline std::optional<PcmClip> read_wav_pcm16(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  in.seekg(0, std::ios::end);
  const std::streamoff file_size = in.tellg();
  if (file_size < 12) {
    return std::nullopt;
  }
  in.seekg(0);
  std::vector<char> buf(static_cast<size_t>(file_size));
  if (!in.read(buf.data(), static_cast<std::streamsize>(file_size))) {
    return std::nullopt;
  }

  if (std::memcmp(buf.data(), "RIFF", 4) != 0 || std::memcmp(buf.data() + 8, "WAVE", 4) != 0) {
    return std::nullopt;
  }

  bool have_fmt = false;
  uint16_t audio_format = 0;
  uint16_t channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits_per_sample = 0;
  const char* data_ptr = nullptr;
  uint32_t data_bytes = 0;

  size_t off = 12;
  while (off + 8 <= buf.size()) {
    const char* id = buf.data() + off;
    const uint32_t chunk_size = detail::read_le32(buf.data() + off + 4);
    off += 8;
    if (chunk_size > buf.size() - off) {
      return std::nullopt;
    }
    if (std::memcmp(id, "fmt ", 4) == 0) {
      if (chunk_size < 16) {
        return std::nullopt;
      }
      audio_format = detail::read_le16(buf.data() + off);
      channels = detail::read_le16(buf.data() + off + 2);
      sample_rate = detail::read_le32(buf.data() + off + 4);
      bits_per_sample = detail::read_le16(buf.data() + off + 14);
      have_fmt = true;
    } else if (std::memcmp(id, "data", 4) == 0 && data_ptr == nullptr) {
      data_ptr = buf.data() + off;
      data_bytes = chunk_size;
    }
    off += chunk_size;
    if ((chunk_size % 2u) == 1u && off < buf.size()) {
      ++off;
    }
  }

  if (!have_fmt || data_ptr == nullptr) {
    return std::nullopt;
  }
  if (audio_format != 1 || bits_per_sample != 16 || channels == 0) {
    return std::nullopt;
  }
  if ((data_bytes % 2u) != 0) {
    return std::nullopt;
  }

  PcmClip clip;
  clip.sample_rate = sample_rate;
  clip.channels = channels;
  const size_t n = data_bytes / 2;
  clip.samples.resize(n);
  for (size_t i = 0; i < n; ++i) {
    clip.samples[i] = static_cast<int16_t>(detail::read_le16(data_ptr + i * 2));
  }
  return clip;
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_WAV_PCM_HPP_
