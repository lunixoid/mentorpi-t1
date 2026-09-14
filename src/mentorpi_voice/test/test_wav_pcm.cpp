#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "mentorpi_voice/wav_pcm.hpp"

using mentorpi_voice::read_wav_pcm16;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void put_le16(std::string& out, uint16_t v) {
  out.push_back(static_cast<char>(v & 0xFF));
  out.push_back(static_cast<char>((v >> 8) & 0xFF));
}

void put_le32(std::string& out, uint32_t v) {
  out.push_back(static_cast<char>(v & 0xFF));
  out.push_back(static_cast<char>((v >> 8) & 0xFF));
  out.push_back(static_cast<char>((v >> 16) & 0xFF));
  out.push_back(static_cast<char>((v >> 24) & 0xFF));
}

std::string make_pcm16_wav(uint32_t sample_rate, uint16_t channels, uint16_t bits,
                           const std::vector<int16_t>& samples) {
  const uint32_t data_bytes = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
  const uint16_t block_align = static_cast<uint16_t>(channels * (bits / 8));
  const uint32_t byte_rate = sample_rate * block_align;
  std::string out;
  out += "RIFF";
  put_le32(out, 36 + data_bytes);
  out += "WAVE";
  out += "fmt ";
  put_le32(out, 16);
  put_le16(out, 1);
  put_le16(out, channels);
  put_le32(out, sample_rate);
  put_le32(out, byte_rate);
  put_le16(out, block_align);
  put_le16(out, bits);
  out += "data";
  put_le32(out, data_bytes);
  for (int16_t s : samples) {
    put_le16(out, static_cast<uint16_t>(s));
  }
  return out;
}

bool write_file(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return static_cast<bool>(out);
}

}  // namespace

int main() {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("mentorpi_voice_wav_" + std::to_string(getpid()));
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::cerr << "FAIL: cannot create temp dir\n";
    return 1;
  }

  const std::vector<int16_t> samples_16k = {0, 1, -1, 32767, -32768};
  const auto wav16_path = dir / "mono16k.wav";
  expect(write_file(wav16_path, make_pcm16_wav(16000, 1, 16, samples_16k)), "write 16 kHz wav");
  const auto clip16 = read_wav_pcm16(wav16_path.string());
  expect(clip16.has_value(), "read 16 kHz mono");
  if (clip16) {
    expect(clip16->sample_rate == 16000, "16 kHz sample_rate");
    expect(clip16->channels == 1, "16 kHz channels");
    expect(clip16->samples == samples_16k, "16 kHz samples");
  }

  const std::vector<int16_t> samples_24k = {10, 20, 30, -40};
  const auto wav24_path = dir / "mono24k.wav";
  expect(write_file(wav24_path, make_pcm16_wav(24000, 1, 16, samples_24k)), "write 24 kHz wav");
  const auto clip24 = read_wav_pcm16(wav24_path.string());
  expect(clip24.has_value(), "read 24 kHz mono");
  if (clip24) {
    expect(clip24->sample_rate == 24000, "24 kHz sample_rate");
    expect(clip24->channels == 1, "24 kHz channels");
    expect(clip24->samples == samples_24k, "24 kHz samples");
  }

  const auto not_riff_path = dir / "not_riff.bin";
  expect(write_file(not_riff_path, std::string("HELLO WORLD!!!!")), "write not-RIFF");
  expect(!read_wav_pcm16(not_riff_path.string()).has_value(), "not RIFF -> none");

  const auto not_pcm_path = dir / "float.wav";
  std::string float_wav = make_pcm16_wav(16000, 1, 16, {1, 2});
  float_wav[20] = 3;  // audio_format at fmt payload offset 0 -> IEEE float
  expect(write_file(not_pcm_path, float_wav), "write non-PCM16");
  expect(!read_wav_pcm16(not_pcm_path.string()).has_value(), "not PCM16 -> none");

  const auto truncated_path = dir / "truncated.wav";
  std::string truncated = make_pcm16_wav(16000, 1, 16, {1, 2, 3, 4});
  truncated.resize(24);  // RIFF/WAVE + start of fmt, no data
  expect(write_file(truncated_path, truncated), "write truncated");
  expect(!read_wav_pcm16(truncated_path.string()).has_value(), "truncated -> none");

  std::filesystem::remove_all(dir, ec);

  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_wav_pcm: ok\n";
  return 0;
}
