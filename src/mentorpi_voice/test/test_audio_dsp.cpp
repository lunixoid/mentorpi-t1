#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "mentorpi_voice/audio_dsp.hpp"

using mentorpi_voice::Decimator;
using mentorpi_voice::LevelMeter;
using mentorpi_voice::LevelStats;
using mentorpi_voice::mix_to_mono;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

constexpr double kPi = 3.14159265358979323846;

std::vector<int16_t> sine(double freq_hz, double rate_hz, double amplitude, size_t n) {
  std::vector<int16_t> out(n);
  for (size_t i = 0; i < n; ++i) {
    out[i] = static_cast<int16_t>(
        std::lround(amplitude * std::sin(2.0 * kPi * freq_hz * static_cast<double>(i) / rate_hz)));
  }
  return out;
}

double rms(const std::vector<int16_t>& x, size_t skip) {
  double sum = 0.0;
  size_t n = 0;
  for (size_t i = skip; i < x.size(); ++i) {
    sum += static_cast<double>(x[i]) * static_cast<double>(x[i]);
    ++n;
  }
  return n == 0 ? 0.0 : std::sqrt(sum / static_cast<double>(n));
}

void test_decimator_passband() {
  const auto in = sine(1000.0, 48000.0, 10000.0, 48000);
  Decimator dec(3);
  std::vector<int16_t> out;
  dec.process(in.data(), in.size(), out);
  expect(out.size() == in.size() / 3, "48000 samples / 3 -> 16000");
  const double gain_db = 20.0 * std::log10(rms(out, 100) / rms(in, 300));
  expect(std::fabs(gain_db) <= 1.0, "1 kHz passes within +-1 dB");
  if (std::fabs(gain_db) > 1.0) {
    std::cerr << "  gain " << gain_db << " dB\n";
  }
}

void test_decimator_stopband() {
  const auto in = sine(12000.0, 48000.0, 10000.0, 48000);
  Decimator dec(3);
  std::vector<int16_t> out;
  dec.process(in.data(), in.size(), out);
  const double out_rms = std::max(rms(out, 100), 1e-3);
  const double gain_db = 20.0 * std::log10(out_rms / rms(in, 300));
  expect(gain_db <= -40.0, "12 kHz attenuated by >= 40 dB");
  if (gain_db > -40.0) {
    std::cerr << "  gain " << gain_db << " dB\n";
  }
}

void test_decimator_chunking() {
  std::vector<int16_t> in(4800);
  uint32_t state = 12345;
  for (auto& s : in) {
    state = state * 1664525u + 1013904223u;
    s = static_cast<int16_t>(state >> 16);
  }
  Decimator whole(3);
  std::vector<int16_t> expected;
  whole.process(in.data(), in.size(), expected);

  for (const size_t chunk : {size_t{100}, size_t{480}}) {
    Decimator dec(3);
    std::vector<int16_t> got;
    std::vector<int16_t> part;
    for (size_t off = 0; off < in.size(); off += chunk) {
      const size_t n = std::min(chunk, in.size() - off);
      dec.process(in.data() + off, n, part);
      got.insert(got.end(), part.begin(), part.end());
    }
    expect(got == expected,
           chunk == 100 ? "chunks of 100 == one call" : "chunks of 480 == one call");
  }

  whole.reset();
  std::vector<int16_t> again;
  whole.process(in.data(), in.size(), again);
  expect(again == expected, "reset -> same output as a fresh decimator");
}

void test_decimator_factor_one() {
  const std::vector<int16_t> in = {1, -2, 3, 32767, -32768};
  Decimator dec(1);
  std::vector<int16_t> out;
  dec.process(in.data(), in.size(), out);
  expect(out == in, "factor 1 copies");

  bool threw = false;
  try {
    Decimator bad(0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  expect(threw, "factor 0 throws");
}

void test_mix_to_mono() {
  const std::vector<int16_t> stereo = {100, 300, -2, -4, 32767, 32767};
  std::vector<int16_t> out;
  mix_to_mono(stereo.data(), 3, 2, out);
  expect(out == std::vector<int16_t>({200, -3, 32767}), "stereo average");

  const std::vector<int16_t> mono = {5, -6, 7};
  mix_to_mono(mono.data(), mono.size(), 1, out);
  expect(out == mono, "mono copy");
}

void test_level_meter() {
  std::vector<int16_t> square(480);
  for (size_t i = 0; i < square.size(); ++i) {
    square[i] = (i / 24) % 2 == 0 ? 32767 : -32768;
  }
  LevelMeter meter;
  meter.add(square.data(), square.size());
  const LevelStats full = meter.take();
  expect(full.samples == 480, "samples counted");
  expect(full.peak_dbfs > -0.01 && full.peak_dbfs <= 0.0, "full-scale square -> peak 0 dBFS");
  expect(std::fabs(full.clipped_share - 1.0) < 1e-9, "full-scale square -> clipped share 1");

  const std::vector<int16_t> zeros(480, 0);
  meter.add(zeros.data(), zeros.size());
  const LevelStats silent = meter.take();
  expect(silent.peak_dbfs == -120.0 && silent.rms_dbfs == -120.0, "zeros -> -120 dBFS");
  expect(silent.clipped_share == 0.0, "zeros -> no clipping");

  const LevelStats empty = meter.take();
  expect(empty.samples == 0 && empty.peak_dbfs == -120.0, "take resets the meter");
}

}  // namespace

int main() {
  test_decimator_passband();
  test_decimator_stopband();
  test_decimator_chunking();
  test_decimator_factor_one();
  test_mix_to_mono();
  test_level_meter();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_audio_dsp: ok\n";
  return 0;
}
