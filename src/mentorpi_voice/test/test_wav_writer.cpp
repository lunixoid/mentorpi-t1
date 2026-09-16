#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

#include "mentorpi_voice/wav_pcm.hpp"
#include "mentorpi_voice/wav_writer.hpp"

using mentorpi_voice::read_wav_pcm16;
using mentorpi_voice::WavWriter;

namespace {

int g_fails = 0;

void expect(bool cond, const std::string& what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

std::filesystem::path make_temp_dir() {
  const auto dir = std::filesystem::temp_directory_path() /
                   ("mentorpi_voice_test_wav_writer_" + std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  return dir;
}

void test_roundtrip(const std::filesystem::path& dir, uint32_t rate, uint16_t channels,
                    size_t samples, const std::string& name) {
  std::vector<int16_t> data(samples);
  for (size_t i = 0; i < samples; ++i) {
    data[i] = static_cast<int16_t>(static_cast<int>((i * 37) % 65536) - 32768);
  }
  const std::string path = (dir / name).string();

  WavWriter writer;
  expect(writer.open(path, rate, channels), name + ": open");
  const size_t half = samples / 2;
  expect(writer.write(data.data(), half), name + ": write first half");
  expect(writer.write(data.data() + half, samples - half), name + ": write second half");
  expect(writer.bytes() == WavWriter::kHeaderBytes + samples * 2, name + ": bytes = header + data");
  expect(writer.close(), name + ": close");

  const auto clip = read_wav_pcm16(path);
  expect(clip.has_value(), name + ": read back");
  if (!clip.has_value()) {
    return;
  }
  expect(clip->sample_rate == rate, name + ": sample rate");
  expect(clip->channels == channels, name + ": channels");
  expect(clip->samples == data, name + ": samples identical");
}

void test_open_fails_in_missing_dir(const std::filesystem::path& dir) {
  WavWriter writer;
  expect(!writer.open((dir / "missing_subdir" / "x.wav").string(), 16000, 1),
         "open in a missing dir -> false");
  const int16_t sample = 1;
  expect(!writer.write(&sample, 1), "write without open -> false");
  expect(!writer.open((dir / "bad_rate.wav").string(), 0, 1), "rate 0 -> false");
}

void test_destructor_finalizes(const std::filesystem::path& dir) {
  const std::string path = (dir / "destructor.wav").string();
  {
    WavWriter writer;
    expect(writer.open(path, 16000, 1), "destructor: open");
    const std::vector<int16_t> data(160, 7);
    expect(writer.write(data.data(), data.size()), "destructor: write");
  }
  const auto clip = read_wav_pcm16(path);
  expect(clip.has_value() && clip->samples.size() == 160, "destructor writes final sizes");
}

}  // namespace

int main() {
  const auto dir = make_temp_dir();
  test_roundtrip(dir, 48000, 1, 1000, "mono48k.wav");
  test_roundtrip(dir, 16000, 2, 1000, "stereo16k.wav");
  test_open_fails_in_missing_dir(dir);
  test_destructor_finalizes(dir);
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_wav_writer: ok\n";
  return 0;
}
