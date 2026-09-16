#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mentorpi_voice/dual_channel.hpp"

using mentorpi_voice::DualChannel;
using mentorpi_voice::DualOptions;
using mentorpi_voice::make_dual_channel;

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
constexpr double kPi = 3.14159265358979323846;

struct Lcg {
  uint32_t state;
  float next() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state >> 16) - 32768) / 32768.0f;
  }
};

std::unique_ptr<DualChannel> make_ok(const char* kind, const DualOptions& opt = DualOptions{}) {
  std::string error;
  auto stage = make_dual_channel(kind, kRate, kFrame, 2, opt, error);
  expect(error.empty(), std::string(kind) + ": no error (" + error + ")");
  expect(std::string(stage->name()) == kind, std::string(kind) + ": name");
  return stage;
}

std::vector<int16_t> run(DualChannel& stage, const std::vector<int16_t>& interleaved) {
  const size_t frames = interleaved.size() / 2;
  std::vector<int16_t> out(frames, 0);
  for (size_t off = 0; off + kFrame <= frames; off += kFrame) {
    stage.process(interleaved.data() + off * 2, out.data() + off);
  }
  return out;
}

double rms(const std::vector<int16_t>& x, size_t from) {
  double sum = 0.0;
  size_t n = 0;
  for (size_t i = from; i < x.size(); ++i) {
    sum += static_cast<double>(x[i]) * static_cast<double>(x[i]);
    ++n;
  }
  return n == 0 ? 0.0 : std::sqrt(sum / static_cast<double>(n));
}

// Amplitude of a tone at freq_hz inside x, by projection on sine and cosine.
double tone_amplitude(const std::vector<int16_t>& x, double freq_hz, size_t from) {
  double re = 0.0;
  double im = 0.0;
  size_t n = 0;
  for (size_t i = from; i < x.size(); ++i) {
    const double phase = 2.0 * kPi * freq_hz * static_cast<double>(i) / kRate;
    re += x[i] * std::cos(phase);
    im += x[i] * std::sin(phase);
    ++n;
  }
  return n == 0 ? 0.0 : 2.0 * std::hypot(re, im) / static_cast<double>(n);
}

double tone_residual_rms(const std::vector<int16_t>& x, double freq_hz, size_t from) {
  double re = 0.0;
  double im = 0.0;
  size_t n = 0;
  for (size_t i = from; i < x.size(); ++i) {
    const double phase = 2.0 * kPi * freq_hz * static_cast<double>(i) / kRate;
    re += x[i] * std::cos(phase);
    im += x[i] * std::sin(phase);
    ++n;
  }
  if (n == 0) {
    return 0.0;
  }
  re = 2.0 * re / static_cast<double>(n);
  im = 2.0 * im / static_cast<double>(n);
  double sum = 0.0;
  for (size_t i = from; i < x.size(); ++i) {
    const double phase = 2.0 * kPi * freq_hz * static_cast<double>(i) / kRate;
    const double tone = re * std::cos(phase) + im * std::sin(phase);
    const double residual = x[i] - tone;
    sum += residual * residual;
  }
  return std::sqrt(sum / static_cast<double>(n));
}

void test_none_is_half_sum() {
  auto stage = make_ok("none");
  std::vector<int16_t> interleaved(kFrame * 2 * 4);
  Lcg lcg{7};
  for (auto& s : interleaved) {
    s = static_cast<int16_t>(std::lround(lcg.next() * 10000.0f));
  }
  const auto out = run(*stage, interleaved);
  bool same = true;
  for (size_t i = 0; i < out.size(); ++i) {
    const int32_t expected =
        (static_cast<int32_t>(interleaved[2 * i]) + interleaved[2 * i + 1]) / 2;
    same = same && out[i] == static_cast<int16_t>(expected);
  }
  expect(same, "none is the half sum of the channels");
}

// Reference channel holds the noise, primary holds the same noise delayed and filtered.
void test_nlms_cancels_correlated_noise() {
  auto stage = make_ok("nlms");
  const size_t frames = kRate * 3;
  std::vector<int16_t> interleaved(frames * 2);
  Lcg lcg{99};
  std::vector<float> noise(frames + 16, 0.0f);
  for (auto& v : noise) {
    v = lcg.next() * 6000.0f;
  }
  for (size_t i = 0; i < frames; ++i) {
    const float reference = noise[i + 8];
    const float primary = 0.8f * noise[i + 5] + 0.4f * noise[i];
    interleaved[2 * i] = static_cast<int16_t>(std::lround(primary));
    interleaved[2 * i + 1] = static_cast<int16_t>(std::lround(reference));
  }
  const auto out = run(*stage, interleaved);

  std::vector<int16_t> primary_only(frames);
  for (size_t i = 0; i < frames; ++i) {
    primary_only[i] = interleaved[2 * i];
  }
  const size_t from = kRate;  // skip the first second of adaptation
  const double before = rms(primary_only, from);
  const double after = std::max(rms(out, from), 1e-3);
  const double db = 20.0 * std::log10(after / before);
  expect(db <= -10.0, "nlms cancels the correlated noise by at least 10 dB");
  std::cout << "nlms: correlated noise " << db << " dB\n";
}

// Same tone in both channels, independent noise in each.
void test_coherence_keeps_common_tone() {
  auto stage = make_ok("coherence");
  const size_t frames = kRate * 2;
  const double tone_hz = 1000.0;
  const double tone_amp = 3000.0;
  const double noise_amp = 3000.0;
  std::vector<int16_t> interleaved(frames * 2);
  Lcg left{11};
  Lcg right{22};
  for (size_t i = 0; i < frames; ++i) {
    const double tone =
        tone_amp * std::sin(2.0 * kPi * tone_hz * static_cast<double>(i) / kRate);
    interleaved[2 * i] =
        static_cast<int16_t>(std::lround(std::clamp(tone + left.next() * noise_amp, -32768.0, 32767.0)));
    interleaved[2 * i + 1] =
        static_cast<int16_t>(std::lround(std::clamp(tone + right.next() * noise_amp, -32768.0, 32767.0)));
  }
  const auto out = run(*stage, interleaved);

  const size_t from = kRate;  // second half only
  const double amp = tone_amplitude(out, tone_hz, from);
  const double residual = tone_residual_rms(out, tone_hz, from);
  // Independent noise of amplitude A in both channels gives A / sqrt(3) / sqrt(2) after averaging.
  const double noise_rms_after_average = noise_amp / std::sqrt(3.0) / std::sqrt(2.0);
  const double tone_db = 20.0 * std::log10(std::max(amp, 1e-3) / tone_amp);
  const double noise_db = 20.0 * std::log10(std::max(residual, 1e-3) / noise_rms_after_average);
  expect(std::fabs(tone_db) <= 3.0, "coherence keeps the common tone within 3 dB");
  expect(noise_db <= -6.0, "coherence drops the uncorrelated part by at least 6 dB");
  std::cout << "coherence: tone " << tone_db << " dB, uncorrelated " << noise_db << " dB\n";
}

void test_edges(const char* kind) {
  auto stage = make_ok(kind);
  const std::vector<int16_t> zeros(kFrame * 2 * 20, 0);
  const auto quiet = run(*stage, zeros);
  bool near_zero = true;
  for (const int16_t s : quiet) {
    near_zero = near_zero && std::abs(static_cast<int>(s)) < 100;
  }
  expect(near_zero, std::string(kind) + ": zeros stay near zero");

  std::vector<int16_t> full(kFrame * 2 * 20);
  for (size_t i = 0; i < full.size(); ++i) {
    full[i] = (i / 48) % 2 == 0 ? 32767 : -32768;
  }
  const auto loud = run(*stage, full);
  expect(loud.size() == full.size() / 2, std::string(kind) + ": full scale runs without overflow");
}

void test_factory_failures() {
  std::string error;
  auto mono = make_dual_channel("coherence", kRate, kFrame, 1, DualOptions{}, error);
  expect(std::string(mono->name()) == "none" && !error.empty(), "one channel -> none + error");

  auto unknown = make_dual_channel("bogus", kRate, kFrame, 2, DualOptions{}, error);
  expect(std::string(unknown->name()) == "none" && !error.empty(), "bogus -> none + error");

  DualOptions short_taps;
  short_taps.nlms_taps = 4;
  auto taps = make_dual_channel("nlms", kRate, kFrame, 2, short_taps, error);
  expect(std::string(taps->name()) == "none" && !error.empty(), "nlms_taps 4 -> none + error");

  DualOptions odd_fft;
  odd_fft.coherence_fft = 300;
  auto fft = make_dual_channel("coherence", kRate, kFrame, 2, odd_fft, error);
  expect(std::string(fft->name()) == "none" && !error.empty(), "coherence_fft 300 -> none + error");

  DualOptions positive_floor;
  positive_floor.coherence_floor_db = 5.0;
  auto floor = make_dual_channel("coherence", kRate, kFrame, 2, positive_floor, error);
  expect(std::string(floor->name()) == "none" && !error.empty(),
         "coherence_floor_db +5 -> none + error");
}

}  // namespace

int main() {
  test_none_is_half_sum();
  test_nlms_cancels_correlated_noise();
  test_coherence_keeps_common_tone();
  test_edges("nlms");
  test_edges("coherence");
  test_factory_failures();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_dual_channel: ok\n";
  return 0;
}
