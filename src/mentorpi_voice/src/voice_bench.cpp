// SD033 D7, I12: offline bench. Runs CommandPipeline over recorded _raw.wav files for every
// combination of parameters and compares detected commands with labels.tsv. No ROS, no ALSA.
#include <time.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mentorpi_voice/bench_score.hpp"
#include "mentorpi_voice/command_pipeline.hpp"
#include "mentorpi_voice/utterance.hpp"
#include "mentorpi_voice/wav_pcm.hpp"
#include "vosk_api.h"

namespace {

using mentorpi_voice::AsrEngine;
using mentorpi_voice::CommandPipeline;
using mentorpi_voice::LabelRow;
using mentorpi_voice::parse_labels;
using mentorpi_voice::parse_vosk_partial;
using mentorpi_voice::parse_vosk_result;
using mentorpi_voice::PcmClip;
using mentorpi_voice::PipelineConfig;
using mentorpi_voice::PipelineEvent;
using mentorpi_voice::read_wav_pcm16;
using mentorpi_voice::Score;
using mentorpi_voice::score_sequence;
using mentorpi_voice::VoskEngine;

constexpr const char* kUsage =
    R"(usage: voice_bench --dir <dir> --labels <labels.tsv> [options]
  --model <dir>               Vosk model (default: $VOSK_MODEL_DIR, else share of mentorpi_voice)
  --denoise <list>            none,rnnoise,speexdsp (default none)
  --partial-trigger <list>    0,1 (default 1)
  --partial-stable-ms <list>  default 200
  --max-utterance-ms <list>   default 5000, 0 disables
  --min-confidence <list>     default 0.6
  --speex-db <list>           default -30, varied only for speexdsp
  --dual <list>               none,nlms,coherence (default none): stage over both channels
  --nlms-taps <list>          default 256, varied only for nlms
  --nlms-mu <list>            default 0.1, varied only for nlms
  --coherence-fft <list>      default 512, varied only for coherence
  --coherence-floor-db <list> default -18, varied only for coherence
  --jobs <n>                  parallel runs (default 1); ms_per_s is thread CPU time
  --events                    print every command: EVENT file config audio_time_ms via command text
  --trace                     print what the recognizer returns:
                              TRACE file config audio_time_ms partial|result|final|reset text
  --free                      diagnostic: recognizer without the grammar (full model vocabulary)
Lists are comma separated, every combination runs. Files in labels.tsv are relative to --dir.
Each file gets 1 s of silence at the end so the last phrase can close.
)";

// The grammar of voice_command.yaml (SD031 I6.1).
const std::vector<std::string> kPhrases = {"режим запрет", "режим следование", "режим ручной"};
const std::vector<std::string> kCommands = {"mode_forbid", "mode_follow", "mode_manual"};

struct Options {
  std::string dir;
  std::string labels;
  std::string model;
  std::vector<std::string> denoise{"none"};
  std::vector<long> partial_trigger{1};
  std::vector<long> partial_stable_ms{200};
  std::vector<long> max_utterance_ms{5000};
  std::vector<double> min_confidence{0.6};
  std::vector<double> speex_db{-30.0};
  std::vector<std::string> dual{"none"};
  std::vector<long> nlms_taps{256};
  std::vector<double> nlms_mu{0.1};
  std::vector<long> coherence_fft{512};
  std::vector<double> coherence_floor_db{-18.0};
  long jobs{1};
  bool events{false};
  bool trace{false};
  bool free_grammar{false};
};

struct BenchConfig {
  std::string name;
  PipelineConfig pipeline;
};

struct RunResult {
  Score score;
  double cpu_ms{0.0};
  double audio_s{0.0};
  std::vector<std::string> events;
  std::string error;
};

// Diagnostic recognizer without the grammar: shows whether speech is intelligible at all.
class FreeVoskEngine : public AsrEngine {
 public:
  FreeVoskEngine(VoskModel* model, float sample_rate)
      : rec_(vosk_recognizer_new(model, sample_rate)) {
    if (rec_ != nullptr) {
      vosk_recognizer_set_words(rec_, 1);
    }
  }
  FreeVoskEngine(const FreeVoskEngine&) = delete;
  FreeVoskEngine& operator=(const FreeVoskEngine&) = delete;
  ~FreeVoskEngine() override {
    if (rec_ != nullptr) {
      vosk_recognizer_free(rec_);
    }
  }
  bool ok() const { return rec_ != nullptr; }
  int accept(const int16_t* data, int n) override {
    return vosk_recognizer_accept_waveform_s(rec_, data, n);
  }
  std::string partial() override { return vosk_recognizer_partial_result(rec_); }
  std::string result() override { return vosk_recognizer_result(rec_); }
  std::string final_result() override { return vosk_recognizer_final_result(rec_); }
  void reset() override { vosk_recognizer_reset(rec_); }

 private:
  VoskRecognizer* rec_{nullptr};
};

// Forwards to the real engine and records every text it returns, on the engine's audio clock.
class TracingEngine : public AsrEngine {
 public:
  TracingEngine(std::unique_ptr<AsrEngine> inner, unsigned rate, std::string prefix,
                std::vector<std::string>* lines)
      : inner_(std::move(inner)), rate_(rate), prefix_(std::move(prefix)), lines_(lines) {}

  int accept(const int16_t* data, int n) override {
    samples_ += static_cast<uint64_t>(n);
    return inner_->accept(data, n);
  }
  std::string partial() override {
    std::string json = inner_->partial();
    const std::string text = parse_vosk_partial(json);
    if (text != last_partial_) {
      last_partial_ = text;
      if (!text.empty()) {
        log("partial", text);
      }
    }
    return json;
  }
  std::string result() override {
    std::string json = inner_->result();
    log("result", parse_vosk_result(json).text);
    last_partial_.clear();
    return json;
  }
  std::string final_result() override {
    std::string json = inner_->final_result();
    log("final", parse_vosk_result(json).text);
    last_partial_.clear();
    return json;
  }
  void reset() override {
    inner_->reset();
    log("reset", "");
    last_partial_.clear();
  }

 private:
  void log(const char* kind, const std::string& text) {
    lines_->push_back("TRACE\t" + prefix_ + "\t" + std::to_string(samples_ * 1000 / rate_) + "\t" +
                      kind + "\t" + text);
  }

  std::unique_ptr<AsrEngine> inner_;
  unsigned rate_;
  std::string prefix_;
  std::vector<std::string>* lines_;
  uint64_t samples_{0};
  std::string last_partial_;
};

[[noreturn]] void fail_usage(const std::string& message) {
  std::cerr << "error: " << message << "\n\n" << kUsage;
  std::exit(2);
}

std::vector<std::string> split_list(const std::string& value, const std::string& flag) {
  std::vector<std::string> out;
  std::istringstream in(value);
  std::string item;
  while (std::getline(in, item, ',')) {
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  if (out.empty()) {
    fail_usage("empty list for " + flag);
  }
  return out;
}

long to_long(const std::string& text, const std::string& flag) {
  char* end = nullptr;
  errno = 0;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (text.empty() || errno != 0 || *end != '\0') {
    fail_usage("bad integer '" + text + "' for " + flag);
  }
  return value;
}

double to_double(const std::string& text, const std::string& flag) {
  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || errno != 0 || *end != '\0') {
    fail_usage("bad number '" + text + "' for " + flag);
  }
  return value;
}

std::vector<long> long_list(const std::string& value, const std::string& flag, long min_value,
                            long max_value) {
  std::vector<long> out;
  for (const auto& item : split_list(value, flag)) {
    const long v = to_long(item, flag);
    if (v < min_value || v > max_value) {
      fail_usage(flag + " value " + item + " out of range");
    }
    out.push_back(v);
  }
  return out;
}

std::vector<double> double_list(const std::string& value, const std::string& flag) {
  std::vector<double> out;
  for (const auto& item : split_list(value, flag)) {
    out.push_back(to_double(item, flag));
  }
  return out;
}

Options parse_args(int argc, char** argv) {
  Options opts;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        fail_usage("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--help" || arg == "-h") {
      std::cout << kUsage;
      std::exit(0);
    } else if (arg == "--dir") {
      opts.dir = value();
    } else if (arg == "--labels") {
      opts.labels = value();
    } else if (arg == "--model") {
      opts.model = value();
    } else if (arg == "--denoise") {
      opts.denoise = split_list(value(), arg);
      for (const auto& kind : opts.denoise) {
        if (kind != "none" && kind != "rnnoise" && kind != "speexdsp") {
          fail_usage("unknown denoise '" + kind + "'");
        }
      }
    } else if (arg == "--partial-trigger") {
      opts.partial_trigger = long_list(value(), arg, 0, 1);
    } else if (arg == "--partial-stable-ms") {
      opts.partial_stable_ms = long_list(value(), arg, 0, 60000);
    } else if (arg == "--max-utterance-ms") {
      opts.max_utterance_ms = long_list(value(), arg, 0, 600000);
    } else if (arg == "--min-confidence") {
      opts.min_confidence = double_list(value(), arg);
    } else if (arg == "--speex-db") {
      opts.speex_db = double_list(value(), arg);
    } else if (arg == "--dual") {
      opts.dual = split_list(value(), arg);
      for (const auto& kind : opts.dual) {
        if (kind != "none" && kind != "nlms" && kind != "coherence") {
          fail_usage("unknown dual '" + kind + "'");
        }
      }
    } else if (arg == "--nlms-taps") {
      opts.nlms_taps = long_list(value(), arg, 16, 2048);
    } else if (arg == "--nlms-mu") {
      opts.nlms_mu = double_list(value(), arg);
    } else if (arg == "--coherence-fft") {
      opts.coherence_fft = long_list(value(), arg, 128, 4096);
    } else if (arg == "--coherence-floor-db") {
      opts.coherence_floor_db = double_list(value(), arg);
    } else if (arg == "--jobs") {
      opts.jobs = to_long(value(), arg);
      if (opts.jobs < 1) {
        fail_usage("--jobs must be >= 1");
      }
    } else if (arg == "--events") {
      opts.events = true;
    } else if (arg == "--trace") {
      opts.trace = true;
    } else if (arg == "--free") {
      opts.free_grammar = true;
    } else {
      fail_usage("unknown argument " + arg);
    }
  }
  if (opts.dir.empty() || opts.labels.empty()) {
    fail_usage("--dir and --labels are required");
  }
  return opts;
}

std::string default_model_dir() {
  if (const char* env = std::getenv("VOSK_MODEL_DIR"); env != nullptr && *env != '\0') {
    return env;
  }
  std::error_code ec;
  const auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (ec) {
    return std::string();
  }
  // install/mentorpi_voice/lib/mentorpi_voice/voice_bench -> install/mentorpi_voice/share/...
  return (exe.parent_path().parent_path().parent_path() /
          "share/mentorpi_voice/models/vosk-model-small-ru-0.22")
      .string();
}

std::vector<BenchConfig> make_configs(const Options& opts) {
  std::vector<BenchConfig> out;
  for (const auto& dual : opts.dual) {
    // Method parameters are swept only for the method they belong to.
    const std::vector<long> taps =
        dual == "nlms" ? opts.nlms_taps : std::vector<long>{opts.nlms_taps.front()};
    const std::vector<double> mus =
        dual == "nlms" ? opts.nlms_mu : std::vector<double>{opts.nlms_mu.front()};
    const std::vector<long> ffts =
        dual == "coherence" ? opts.coherence_fft : std::vector<long>{opts.coherence_fft.front()};
    const std::vector<double> floors = dual == "coherence"
                                           ? opts.coherence_floor_db
                                           : std::vector<double>{opts.coherence_floor_db.front()};
    for (const long tap : taps) {
      for (const double mu : mus) {
        for (const long fft : ffts) {
          for (const double floor_db : floors) {
            for (const auto& denoise : opts.denoise) {
              for (const long partial : opts.partial_trigger) {
                for (const long stable : opts.partial_stable_ms) {
                  for (const long utterance : opts.max_utterance_ms) {
                    for (const double conf : opts.min_confidence) {
                      const std::vector<double> speex =
                          denoise == "speexdsp" ? opts.speex_db
                                                : std::vector<double>{opts.speex_db.front()};
                      for (const double db : speex) {
                        BenchConfig config;
                        PipelineConfig& p = config.pipeline;
                        p.asr_rate = 16000;
                        p.denoise = denoise;
                        p.denoise_options.speex_noise_suppress_db = db;
                        p.partial_trigger = partial != 0;
                        p.partial_stable = std::chrono::milliseconds(stable);
                        p.max_utterance = std::chrono::milliseconds(utterance);
                        p.phrases = kPhrases;
                        p.commands = kCommands;
                        p.min_confidence = conf;
                        p.dual = dual;
                        p.dual_options.nlms_taps = static_cast<size_t>(tap);
                        p.dual_options.nlms_mu = mu;
                        p.dual_options.coherence_fft = static_cast<size_t>(fft);
                        p.dual_options.coherence_floor_db = floor_db;
                        std::ostringstream name;
                        name << denoise << ";p=" << partial << ";s=" << stable << ";u=" << utterance
                             << ";c=" << std::fixed << std::setprecision(2) << conf;
                        name << ";d=" << dual;
                        if (dual == "nlms") {
                          name << ";t=" << tap << ";mu=" << std::setprecision(2) << mu;
                        } else if (dual == "coherence") {
                          name << ";n=" << fft << ";fl=" << std::setprecision(0) << floor_db;
                        }
                        if (denoise == "speexdsp") {
                          name << ";x=" << std::setprecision(0) << db;
                        }
                        if (opts.free_grammar) {
                          name << ";free";
                        }
                        config.name = name.str();
                        out.push_back(std::move(config));
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return out;
}

double thread_cpu_ms() {
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1e6;
}

RunResult run_one(VoskModel* model, const LabelRow& row, const PcmClip& clip,
                  const BenchConfig& bench, const Options& opts) {
  RunResult result;
  try {
    PipelineConfig config = bench.pipeline;
    config.capture_rate = clip.sample_rate;
    config.channels = clip.channels;
    std::unique_ptr<AsrEngine> engine;
    if (opts.free_grammar) {
      auto free_engine =
          std::make_unique<FreeVoskEngine>(model, static_cast<float>(config.asr_rate));
      if (!free_engine->ok()) {
        result.error = "vosk_recognizer_new failed";
        return result;
      }
      engine = std::move(free_engine);
    } else {
      auto grammar_engine =
          std::make_unique<VoskEngine>(model, static_cast<float>(config.asr_rate), config.phrases);
      if (!grammar_engine->ok()) {
        result.error = "vosk_recognizer_new_grm failed";
        return result;
      }
      engine = std::move(grammar_engine);
    }
    if (opts.trace) {
      engine = std::make_unique<TracingEngine>(std::move(engine), config.asr_rate,
                                               row.file + "\t" + bench.name, &result.events);
    }
    CommandPipeline pipeline(config, std::move(engine));
    if (!pipeline.denoise_error().empty()) {
      result.error = "denoise: " + pipeline.denoise_error();
      return result;
    }
    if (!pipeline.dual_error().empty()) {
      result.error = "dual: " + pipeline.dual_error();
      return result;
    }

    const size_t channels = clip.channels;
    const size_t frames = clip.samples.size() / channels;
    const size_t chunk = std::max<size_t>(1, clip.sample_rate / 10);
    const std::vector<int16_t> silence(chunk * channels, 0);
    constexpr size_t kTailChunks = 10;
    std::vector<std::string> detected;
    std::vector<PipelineEvent> events;
    auto collect = [&]() {
      for (const auto& e : events) {
        if (!e.command || !e.phrase.has_value() || *e.phrase >= config.commands.size()) {
          continue;
        }
        const std::string& command = config.commands[*e.phrase];
        detected.push_back(command);
        if (opts.events) {
          result.events.push_back("EVENT\t" + row.file + "\t" + bench.name + "\t" +
                                  std::to_string(e.audio_time.count()) + "\t" + e.via + "\t" +
                                  command + "\t" + e.text);
        }
      }
    };

    const double start = thread_cpu_ms();
    for (size_t off = 0; off < frames; off += chunk) {
      const size_t n = std::min(chunk, frames - off);
      pipeline.feed(clip.samples.data() + off * channels, n, events);
      collect();
    }
    for (size_t i = 0; i < kTailChunks; ++i) {
      pipeline.feed(silence.data(), chunk, events);
      collect();
    }
    result.cpu_ms = thread_cpu_ms() - start;
    result.audio_s = static_cast<double>(frames + kTailChunks * chunk) / clip.sample_rate;
    result.score = score_sequence(row.commands, detected);
  } catch (const std::exception& e) {
    result.error = e.what();
  }
  return result;
}

std::string session_of(const std::string& file) {
  const size_t slash = file.find('/');
  return slash == std::string::npos ? file : file.substr(0, slash);
}

void print_row(const std::string& file, const std::string& config, const Score& s, double cpu_ms,
               double audio_s) {
  const double ms_per_s = audio_s > 0.0 ? cpu_ms / audio_s : 0.0;
  std::cout << file << '\t' << config << '\t' << s.expected << '\t' << s.detected << '\t' << s.hits
            << '\t' << s.missed << '\t' << s.extra << '\t' << std::fixed << std::setprecision(1)
            << ms_per_s << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  const Options opts = parse_args(argc, argv);

  std::ifstream labels_in(opts.labels);
  if (!labels_in) {
    std::cerr << "error: cannot open labels " << opts.labels << '\n';
    return 2;
  }
  std::string error;
  const auto rows = parse_labels(labels_in, error);
  if (!rows.has_value()) {
    std::cerr << "error: " << opts.labels << ": " << error << '\n';
    return 2;
  }
  if (rows->empty()) {
    std::cerr << "error: " << opts.labels << ": no rows\n";
    return 2;
  }

  std::vector<PcmClip> clips;
  for (const auto& row : *rows) {
    const std::string path = (std::filesystem::path(opts.dir) / row.file).string();
    auto clip = read_wav_pcm16(path);
    if (!clip.has_value()) {
      std::cerr << "error: cannot read " << path << '\n';
      return 2;
    }
    clips.push_back(std::move(*clip));
  }

  const std::string model_dir = opts.model.empty() ? default_model_dir() : opts.model;
  vosk_set_log_level(-1);
  VoskModel* model = vosk_model_new(model_dir.c_str());
  if (model == nullptr) {
    std::cerr << "error: cannot load Vosk model " << model_dir << '\n';
    return 2;
  }

  const std::vector<BenchConfig> configs = make_configs(opts);
  const size_t tasks = rows->size() * configs.size();
  std::vector<RunResult> results(tasks);
  std::atomic<size_t> next{0};
  auto worker = [&]() {
    while (true) {
      const size_t i = next++;
      if (i >= tasks) {
        break;
      }
      const size_t r = i / configs.size();
      const size_t c = i % configs.size();
      results[i] = run_one(model, (*rows)[r], clips[r], configs[c], opts);
      std::cerr << "done " << (*rows)[r].file << ' ' << configs[c].name << '\n';
    }
  };
  std::vector<std::thread> pool;
  for (long j = 0; j < opts.jobs; ++j) {
    pool.emplace_back(worker);
  }
  for (auto& t : pool) {
    t.join();
  }
  vosk_model_free(model);

  int status = 0;
  for (const auto& result : results) {
    for (const auto& line : result.events) {
      std::cout << line << '\n';
    }
  }
  std::cout << "file\tconfig\texpected\tdetected\thits\tmissed\textra\tms_per_s\n";
  for (size_t i = 0; i < tasks; ++i) {
    const auto& row = (*rows)[i / configs.size()];
    const auto& config = configs[i % configs.size()];
    if (!results[i].error.empty()) {
      std::cerr << "error: " << row.file << ' ' << config.name << ": " << results[i].error << '\n';
      status = 1;
      continue;
    }
    print_row(row.file, config.name, results[i].score, results[i].cpu_ms, results[i].audio_s);
  }

  std::vector<std::string> sessions;
  for (const auto& row : *rows) {
    const std::string session = session_of(row.file);
    if (std::find(sessions.begin(), sessions.end(), session) == sessions.end()) {
      sessions.push_back(session);
    }
  }
  for (size_t c = 0; c < configs.size(); ++c) {
    Score all;
    double all_cpu = 0.0;
    double all_audio = 0.0;
    for (const auto& session : sessions) {
      Score total;
      double cpu = 0.0;
      double audio = 0.0;
      for (size_t r = 0; r < rows->size(); ++r) {
        const auto& result = results[r * configs.size() + c];
        if (session_of((*rows)[r].file) != session || !result.error.empty()) {
          continue;
        }
        total += result.score;
        cpu += result.cpu_ms;
        audio += result.audio_s;
      }
      print_row("TOTAL " + session, configs[c].name, total, cpu, audio);
      all += total;
      all_cpu += cpu;
      all_audio += audio;
    }
    print_row("TOTAL all", configs[c].name, all, all_cpu, all_audio);
  }
  return status;
}
