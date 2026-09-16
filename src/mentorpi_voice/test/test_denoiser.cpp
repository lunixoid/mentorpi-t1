#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mentorpi_voice/denoiser.hpp"

using mentorpi_voice::DenoiseOptions;
using mentorpi_voice::Denoiser;
using mentorpi_voice::make_denoiser;

namespace {

int g_fails = 0;

void expect(bool cond, const std::string& what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

constexpr unsigned kRate = 48000;
constexpr size_t kFrame = 480;

// Uniform white noise with RMS -20 dBFS (amplitude A has RMS A / sqrt(3)).
std::vector<int16_t> white_noise(size_t frames) {
  const double amplitude = 32768.0 * 0.1 * std::sqrt(3.0);
  std::vector<int16_t> out(frames * kFrame);
  uint32_t state = 2463534242u;
  for (auto& s : out) {
    state = state * 1664525u + 1013904223u;
    const double u = static_cast<double>(state) / 4294967296.0 * 2.0 - 1.0;
    s = static_cast<int16_t>(std::lround(amplitude * u));
  }
  return out;
}

double rms(const std::vector<int16_t>& x, size_t from, size_t to) {
  double sum = 0.0;
  for (size_t i = from; i < to; ++i) {
    sum += static_cast<double>(x[i]) * static_cast<double>(x[i]);
  }
  return to > from ? std::sqrt(sum / static_cast<double>(to - from)) : 0.0;
}

std::vector<int16_t> run(Denoiser& denoiser, std::vector<int16_t> signal) {
  for (size_t off = 0; off + kFrame <= signal.size(); off += kFrame) {
    denoiser.process(signal.data() + off);
  }
  return signal;
}

std::unique_ptr<Denoiser> make_ok(const char* kind) {
  std::string error;
  auto denoiser = make_denoiser(kind, kRate, DenoiseOptions{}, error);
  expect(error.empty(), std::string(kind) + ": no error (" + error + ")");
  expect(std::string(denoiser->name()) == kind, std::string(kind) + ": name");
  return denoiser;
}

void test_none_passthrough() {
  auto denoiser = make_ok("none");
  const auto noise = white_noise(10);
  expect(run(*denoiser, noise) == noise, "none leaves frames unchanged");
}

void test_suppresses_white_noise(const char* kind) {
  auto denoiser = make_ok(kind);
  const auto noise = white_noise(300);
  const auto out = run(*denoiser, noise);
  const size_t from = 100 * kFrame;
  const double in_rms = rms(noise, from, noise.size());
  const double out_rms = rms(out, from, out.size());
  expect(out_rms < in_rms, std::string(kind) + ": RMS after 1 s below input on white noise");
  std::cout << kind << ": white noise -20 dBFS, out/in "
            << 20.0 * std::log10(std::max(out_rms, 1e-3) / in_rms) << " dB\n";
}

void test_edges_and_reset(const char* kind) {
  auto denoiser = make_ok(kind);
  const std::vector<int16_t> zeros(50 * kFrame, 0);
  const auto out_zeros = run(*denoiser, zeros);
  bool quiet = true;
  for (const int16_t s : out_zeros) {
    quiet = quiet && std::abs(static_cast<int>(s)) < 100;
  }
  expect(quiet, std::string(kind) + ": zeros stay near zero");

  std::vector<int16_t> square(50 * kFrame);
  for (size_t i = 0; i < square.size(); ++i) {
    square[i] = (i / 24) % 2 == 0 ? 32767 : -32768;
  }
  run(*denoiser, square);  // must not crash on full scale; output saturates in int16

  const auto noise = white_noise(50);
  denoiser->reset();
  const auto after_reset = run(*denoiser, noise);
  auto fresh = make_ok(kind);
  expect(run(*fresh, noise) == after_reset, std::string(kind) + ": reset == fresh state");
}

void test_factory_failures() {
  std::string error;
  auto wrong_rate = make_denoiser("rnnoise", 16000, DenoiseOptions{}, error);
  expect(std::string(wrong_rate->name()) == "none" && !error.empty(),
         "rnnoise at 16000 -> none + error");

  auto unknown = make_denoiser("bogus", kRate, DenoiseOptions{}, error);
  expect(std::string(unknown->name()) == "none" && !error.empty(), "bogus -> none + error");

  DenoiseOptions positive;
  positive.speex_noise_suppress_db = 5.0;
  auto bad_db = make_denoiser("speexdsp", kRate, positive, error);
  expect(std::string(bad_db->name()) == "none" && !error.empty(), "speexdsp +5 dB -> none + error");

  auto speex = make_denoiser("speexdsp", kRate, DenoiseOptions{}, error);
  expect(std::string(speex->name()) == "speexdsp" && error.empty(), "speexdsp at 48000 -> ok");
}

}  // namespace

int main() {
  test_none_passthrough();
  test_suppresses_white_noise("rnnoise");
  test_suppresses_white_noise("speexdsp");
  test_edges_and_reset("rnnoise");
  test_edges_and_reset("speexdsp");
  test_factory_failures();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_denoiser: ok\n";
  return 0;
}
