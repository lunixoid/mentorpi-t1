#include <alsa/asoundlib.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/named_command.hpp"
#include "mentorpi_voice/announce_policy.hpp"
#include "mentorpi_voice/audio_dsp.hpp"
#include "mentorpi_voice/command_pipeline.hpp"
#include "mentorpi_voice/wav_pcm.hpp"
#include "mentorpi_voice/wav_writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "vosk_api.h"

namespace mentorpi_voice {
namespace {

std::string join_strings(const std::vector<std::string>& values) {
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << ',';
    }
    out << values[i];
  }
  out << ']';
  return out.str();
}

std::string join_ints(const std::vector<int64_t>& values) {
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << ',';
    }
    out << values[i];
  }
  out << ']';
  return out.str();
}

std::string resolve_model_dir(const std::string& model_dir) {
  if (!model_dir.empty()) {
    return model_dir;
  }
  return ament_index_cpp::get_package_share_directory("mentorpi_voice") +
         "/models/vosk-model-small-ru-0.22";
}

std::string alsa_plughw_name(const std::string& device) {
  if (device.rfind("hw:", 0) == 0) {
    return "plughw:" + device.substr(3);
  }
  return device;
}

// SD033 D5: capture gain of an ALSA mixer control, in percent of its range. percent < 0 only
// reads the current value. Returns false and fills error when the mixer or the control is missing.
bool capture_gain_percent(const std::string& mixer_device, const std::string& control, long percent,
                          long* current_percent, std::string* error) {
  snd_mixer_t* handle = nullptr;
  int err = snd_mixer_open(&handle, 0);
  if (err < 0) {
    *error = snd_strerror(err);
    return false;
  }
  auto close_handle = [&handle]() { snd_mixer_close(handle); };
  err = snd_mixer_attach(handle, mixer_device.c_str());
  if (err < 0) {
    *error = snd_strerror(err);
    close_handle();
    return false;
  }
  err = snd_mixer_selem_register(handle, nullptr, nullptr);
  if (err >= 0) {
    err = snd_mixer_load(handle);
  }
  if (err < 0) {
    *error = snd_strerror(err);
    close_handle();
    return false;
  }

  snd_mixer_selem_id_t* sid = nullptr;
  snd_mixer_selem_id_alloca(&sid);
  snd_mixer_selem_id_set_index(sid, 0);
  snd_mixer_selem_id_set_name(sid, control.c_str());
  snd_mixer_elem_t* elem = snd_mixer_find_selem(handle, sid);
  if (elem == nullptr) {
    *error = "no capture control '" + control + "'";
    close_handle();
    return false;
  }

  long min = 0;
  long max = 0;
  err = snd_mixer_selem_get_capture_volume_range(elem, &min, &max);
  if (err < 0 || max <= min) {
    *error = "no capture volume range";
    close_handle();
    return false;
  }
  if (percent >= 0) {
    const long value = min + (max - min) * percent / 100;
    err = snd_mixer_selem_set_capture_volume_all(elem, value);
    if (err < 0) {
      *error = snd_strerror(err);
      close_handle();
      return false;
    }
  }
  long value = min;
  err = snd_mixer_selem_get_capture_volume(elem, SND_MIXER_SCHN_MONO, &value);
  if (err < 0) {
    err = snd_mixer_selem_get_capture_volume(elem, SND_MIXER_SCHN_FRONT_LEFT, &value);
  }
  if (err < 0) {
    *error = snd_strerror(err);
    close_handle();
    return false;
  }
  *current_percent = (value - min) * 100 / (max - min);
  close_handle();
  return true;
}

bool waitpid_with_timeout(pid_t pid, int* status, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const pid_t waited = waitpid(pid, status, WNOHANG);
    if (waited == pid) {
      return true;
    }
    if (waited < 0) {
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      kill(pid, SIGKILL);
      waitpid(pid, status, 0);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

std::string run_tts_synthesis(const std::string& tts_command, const std::vector<std::string>& args,
                              const std::string& text) {
  std::vector<std::string> argv_store;
  argv_store.reserve(args.size() + 1);
  argv_store.push_back(tts_command);
  argv_store.insert(argv_store.end(), args.begin(), args.end());
  std::vector<char*> argv;
  argv.reserve(argv_store.size() + 1);
  for (auto& token : argv_store) {
    argv.push_back(token.data());
  }
  argv.push_back(nullptr);

  signal(SIGPIPE, SIG_IGN);

  int pipefd[2];
  if (pipe(pipefd) != 0) {
    return "pipe failed";
  }

  const pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    return "fork failed";
  }

  if (pid == 0) {
    close(pipefd[1]);
    if (dup2(pipefd[0], STDIN_FILENO) < 0) {
      _exit(127);
    }
    close(pipefd[0]);
    execvp(argv_store[0].c_str(), argv.data());
    _exit(127);
  }

  close(pipefd[0]);
  if (!text.empty()) {
    size_t offset = 0;
    while (offset < text.size()) {
      const ssize_t written = write(pipefd[1], text.data() + offset, text.size() - offset);
      if (written < 0) {
        close(pipefd[1]);
        kill(pid, SIGKILL);
        int ignored = 0;
        waitpid(pid, &ignored, 0);
        return "stdin write failed";
      }
      offset += static_cast<size_t>(written);
    }
  }
  close(pipefd[1]);

  int status = 0;
  if (!waitpid_with_timeout(pid, &status, std::chrono::seconds(10))) {
    return "timeout";
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    if (WIFEXITED(status)) {
      return "exit code " + std::to_string(WEXITSTATUS(status));
    }
    return "terminated abnormally";
  }
  return "";
}

std::string local_timestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm tm_local{};
  localtime_r(&now, &tm_local);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm_local);
  return buf;
}

// SD033 D4.1: WAV segments of the raw capture and of the block fed to Vosk. Files are written by
// a separate thread so the capture thread never waits on the disk. Any failure (size limit, file
// error, queue behind by more than 2 s of raw audio) stops recording until restart; recognition
// is not affected.
class AudioRecorder {
 public:
  struct Config {
    std::string dir;
    uint32_t raw_rate{48000};
    uint16_t raw_channels{1};
    uint32_t asr_rate{16000};
    int64_t segment_s{300};
    uint64_t max_bytes{0};
  };
  using Log = std::function<void(bool warn, const std::string& text)>;

  AudioRecorder(Config config, Log log) : config_(std::move(config)), log_(std::move(log)) {
    max_queued_raw_ = static_cast<size_t>(config_.raw_rate) * config_.raw_channels * 2;
    segment_raw_limit_ = static_cast<uint64_t>(config_.raw_rate) * config_.raw_channels *
                         static_cast<uint64_t>(config_.segment_s);
    thread_ = std::thread([this] { writer_loop(); });
  }

  AudioRecorder(const AudioRecorder&) = delete;
  AudioRecorder& operator=(const AudioRecorder&) = delete;

  ~AudioRecorder() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void push_raw(const int16_t* data, size_t samples) { push(false, data, samples); }
  void push_asr(const int16_t* data, size_t samples) { push(true, data, samples); }

 private:
  struct Block {
    bool asr{false};
    std::vector<int16_t> samples;
  };

  void push(bool asr, const int16_t* data, size_t samples) {
    if (stopped_ || samples == 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (overflow_) {
        return;
      }
      if (!asr) {
        if (queued_raw_ + samples > max_queued_raw_) {
          overflow_ = true;
        } else {
          queued_raw_ += samples;
        }
      }
      if (!overflow_) {
        queue_.push_back(Block{asr, std::vector<int16_t>(data, data + samples)});
      }
    }
    cv_.notify_one();
  }

  void writer_loop() {
    while (true) {
      Block block;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return shutdown_ || overflow_ || !queue_.empty(); });
        if (overflow_) {
          queue_.clear();
          queued_raw_ = 0;
          overflow_ = false;
          lock.unlock();
          stop("queue overflow");
          continue;
        }
        if (queue_.empty()) {
          break;
        }
        block = std::move(queue_.front());
        queue_.pop_front();
        if (!block.asr) {
          queued_raw_ -= block.samples.size();
        }
      }
      if (!stopped_) {
        write_block(block);
      }
    }
    close_files();
  }

  void write_block(const Block& block) {
    if (!block.asr && (!raw_.is_open() || segment_raw_samples_ >= segment_raw_limit_)) {
      if (!open_segment()) {
        return;
      }
    }
    if (!raw_.is_open()) {
      return;
    }
    const uint64_t block_bytes = static_cast<uint64_t>(block.samples.size()) * 2;
    if (written_bytes_ + block_bytes > config_.max_bytes) {
      stop("limit");
      return;
    }
    WavWriter& writer = block.asr ? asr_ : raw_;
    if (!writer.write(block.samples.data(), block.samples.size())) {
      stop("write failed " + (block.asr ? asr_path_ : raw_path_));
      return;
    }
    written_bytes_ += block_bytes;
    if (!block.asr) {
      segment_raw_samples_ += block.samples.size();
    }
  }

  bool open_segment() {
    close_files();
    if (written_bytes_ + 2 * WavWriter::kHeaderBytes > config_.max_bytes) {
      stop("limit");
      return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(config_.dir, ec);
    if (ec) {
      stop("cannot create " + config_.dir + ": " + ec.message());
      return false;
    }
    const std::string base = config_.dir + "/" + local_timestamp();
    raw_path_ = base + "_raw.wav";
    asr_path_ = base + "_asr.wav";
    if (!raw_.open(raw_path_, config_.raw_rate, config_.raw_channels)) {
      stop("cannot open " + raw_path_);
      return false;
    }
    if (!asr_.open(asr_path_, config_.asr_rate, 1)) {
      stop("cannot open " + asr_path_);
      return false;
    }
    written_bytes_ += 2 * WavWriter::kHeaderBytes;
    segment_raw_samples_ = 0;
    log_(false, "record open " + raw_path_);
    return true;
  }

  void close_files() {
    raw_.close();
    asr_.close();
  }

  void stop(const std::string& reason) {
    if (stopped_.exchange(true)) {
      return;
    }
    close_files();
    log_(true, "record stopped: " + reason);
  }

  Config config_;
  Log log_;
  size_t max_queued_raw_{0};
  uint64_t segment_raw_limit_{0};

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Block> queue_;
  size_t queued_raw_{0};
  bool overflow_{false};
  bool shutdown_{false};
  std::atomic<bool> stopped_{false};

  WavWriter raw_;
  WavWriter asr_;
  std::string raw_path_;
  std::string asr_path_;
  uint64_t written_bytes_{0};
  uint64_t segment_raw_samples_{0};

  std::thread thread_;
};

}  // namespace

class VoiceCommandNode : public rclcpp::Node {
 public:
  VoiceCommandNode() : Node("voice_command") {
    declare_parameter<std::string>("capture_device", "default");
    declare_parameter<int64_t>("capture_rate", 48000);
    declare_parameter<int64_t>("sample_rate", 16000);
    declare_parameter<int64_t>("capture_channels", 1);
    declare_parameter<std::string>("model_dir", "");
    declare_parameter<std::vector<std::string>>(
        "phrases", {"режим запрет", "режим следование", "режим ручной"});
    declare_parameter<std::vector<std::string>>("commands",
                                                {"mode_forbid", "mode_follow", "mode_manual"});
    declare_parameter<std::vector<int64_t>>("states", {0, 2, 1});
    declare_parameter<double>("min_confidence", 0.6);
    declare_parameter<int64_t>("retry_ms", 1000);
    declare_parameter<std::string>("denoise", "speexdsp");
    declare_parameter<double>("speex_noise_suppress_db", -30.0);
    declare_parameter<bool>("partial_trigger", true);
    declare_parameter<int64_t>("partial_stable_ms", 200);
    declare_parameter<int64_t>("max_utterance_ms", 5000);
    declare_parameter<std::string>("mixer_device", "hw:CARD=Device");
    declare_parameter<std::string>("capture_mixer_control", "Mic");
    declare_parameter<int64_t>("capture_gain_percent", -1);
    declare_parameter<std::string>("record_dir", "");
    declare_parameter<int64_t>("record_segment_s", 300);
    declare_parameter<int64_t>("record_max_mb", 500);
    declare_parameter<bool>("publish_commands", true);
    declare_parameter<int64_t>("stats_log_ms", 10000);
    declare_parameter<std::string>("playback_device", "default");
    declare_parameter<std::string>("tts_command", "RHVoice-test");
    declare_parameter<std::string>("tts_voice", "anna");
    declare_parameter<std::string>("tts_cache_dir", "/tmp/mentorpi_voice");
    declare_parameter<int64_t>("announce_timeout_ms", 1000);
    declare_parameter<int64_t>("pad_announce_window_ms", 1000);
    declare_parameter<int64_t>("mute_tail_ms", 300);

    capture_device_ = get_parameter("capture_device").as_string();
    if (capture_device_.empty()) {
      throw std::invalid_argument("capture_device must not be empty");
    }
    sample_rate_ = get_parameter("sample_rate").as_int();
    if (sample_rate_ <= 0) {
      throw std::invalid_argument("sample_rate must be > 0");
    }
    capture_rate_ = get_parameter("capture_rate").as_int();
    if (capture_rate_ <= 0) {
      throw std::invalid_argument("capture_rate must be > 0");
    }
    if (capture_rate_ % sample_rate_ != 0) {
      throw std::invalid_argument(
          "capture_rate must be a multiple of sample_rate (got capture_rate=" +
          std::to_string(capture_rate_) + " sample_rate=" + std::to_string(sample_rate_) + ")");
    }
    capture_channels_ = get_parameter("capture_channels").as_int();
    if (capture_channels_ != 1 && capture_channels_ != 2) {
      throw std::invalid_argument("capture_channels must be 1 or 2");
    }
    const std::string model_dir_param = get_parameter("model_dir").as_string();
    phrases_ = get_parameter("phrases").as_string_array();
    commands_ = get_parameter("commands").as_string_array();
    states_ = get_parameter("states").as_integer_array();
    min_confidence_ = get_parameter("min_confidence").as_double();
    if (min_confidence_ < 0.0 || min_confidence_ > 1.0) {
      throw std::invalid_argument("min_confidence must be in [0, 1]");
    }
    const int64_t retry_ms = get_parameter("retry_ms").as_int();
    if (retry_ms <= 0) {
      throw std::invalid_argument("retry_ms must be > 0");
    }
    retry_ = std::chrono::milliseconds(retry_ms);

    denoise_ = get_parameter("denoise").as_string();
    speex_noise_suppress_db_ = get_parameter("speex_noise_suppress_db").as_double();
    if (speex_noise_suppress_db_ > 0.0) {
      throw std::invalid_argument("speex_noise_suppress_db must be <= 0");
    }
    partial_trigger_ = get_parameter("partial_trigger").as_bool();
    partial_stable_ms_ = get_parameter("partial_stable_ms").as_int();
    if (partial_stable_ms_ < 0) {
      throw std::invalid_argument("partial_stable_ms must be >= 0");
    }
    max_utterance_ms_ = get_parameter("max_utterance_ms").as_int();
    if (max_utterance_ms_ < 0) {
      throw std::invalid_argument("max_utterance_ms must be >= 0");
    }
    mixer_device_ = get_parameter("mixer_device").as_string();
    capture_mixer_control_ = get_parameter("capture_mixer_control").as_string();
    capture_gain_percent_ = get_parameter("capture_gain_percent").as_int();
    if (capture_gain_percent_ < -1 || capture_gain_percent_ > 100) {
      throw std::invalid_argument("capture_gain_percent must be -1 or 0-100");
    }

    record_dir_ = get_parameter("record_dir").as_string();
    record_segment_s_ = get_parameter("record_segment_s").as_int();
    if (record_segment_s_ <= 0) {
      throw std::invalid_argument("record_segment_s must be > 0");
    }
    record_max_mb_ = get_parameter("record_max_mb").as_int();
    if (record_max_mb_ <= 0) {
      throw std::invalid_argument("record_max_mb must be > 0");
    }
    publish_commands_ = get_parameter("publish_commands").as_bool();
    stats_log_ms_ = get_parameter("stats_log_ms").as_int();
    if (stats_log_ms_ <= 0) {
      throw std::invalid_argument("stats_log_ms must be > 0");
    }

    playback_device_ = get_parameter("playback_device").as_string();
    if (playback_device_.empty()) {
      throw std::invalid_argument("playback_device must not be empty");
    }
    tts_command_ = get_parameter("tts_command").as_string();
    if (tts_command_.empty()) {
      throw std::invalid_argument("tts_command must not be empty");
    }
    tts_voice_ = get_parameter("tts_voice").as_string();
    if (tts_voice_.empty()) {
      throw std::invalid_argument("tts_voice must not be empty");
    }
    tts_cache_dir_ = get_parameter("tts_cache_dir").as_string();
    if (tts_cache_dir_.empty()) {
      throw std::invalid_argument("tts_cache_dir must not be empty");
    }
    announce_timeout_ms_ = get_parameter("announce_timeout_ms").as_int();
    if (announce_timeout_ms_ <= 0) {
      throw std::invalid_argument("announce_timeout_ms must be > 0");
    }
    pad_announce_window_ms_ = get_parameter("pad_announce_window_ms").as_int();
    if (pad_announce_window_ms_ <= 0) {
      throw std::invalid_argument("pad_announce_window_ms must be > 0");
    }
    mute_tail_ms_ = get_parameter("mute_tail_ms").as_int();
    if (mute_tail_ms_ <= 0) {
      throw std::invalid_argument("mute_tail_ms must be > 0");
    }

    if (phrases_.size() != commands_.size() || phrases_.size() != states_.size()) {
      throw std::invalid_argument(
          "phrases, commands, states must have the same length (got phrases=" +
          std::to_string(phrases_.size()) + " commands=" + std::to_string(commands_.size()) +
          " states=" + std::to_string(states_.size()) + ")");
    }
    bool seen[3] = {false, false, false};
    for (const int64_t mode : states_) {
      if (mode < 0 || mode > 2) {
        throw std::invalid_argument("states values must be 0, 1 or 2");
      }
      const auto idx = static_cast<size_t>(mode);
      if (seen[idx]) {
        throw std::invalid_argument("states must contain each mode 0-2 exactly once");
      }
      seen[idx] = true;
    }
    if (!seen[0] || !seen[1] || !seen[2]) {
      throw std::invalid_argument("states must contain each mode 0-2 exactly once");
    }

    capture_device_open_ = alsa_plughw_name(capture_device_);
    playback_device_open_ = alsa_plughw_name(playback_device_);
    announce_policy_ = AnnouncePolicy(std::chrono::milliseconds(announce_timeout_ms_),
                                      std::chrono::milliseconds(pad_announce_window_ms_));
    mute_window_ = MuteWindow(std::chrono::milliseconds(mute_tail_ms_));

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_pub_ =
        create_publisher<mentorpi_msgs::msg::NamedCommand>("/commands/named", command_qos);

    RCLCPP_INFO(get_logger(),
                "voice_command started (capture_device=%s capture_rate=%ld sample_rate=%ld "
                "capture_channels=%ld model_dir=%s phrases=%s commands=%s states=%s "
                "min_confidence=%.2f retry_ms=%ld record_dir=%s record_segment_s=%ld "
                "record_max_mb=%ld publish_commands=%s stats_log_ms=%ld)",
                capture_device_.c_str(), capture_rate_, sample_rate_, capture_channels_,
                model_dir_param.c_str(), join_strings(phrases_).c_str(),
                join_strings(commands_).c_str(), join_ints(states_).c_str(), min_confidence_,
                retry_ms, record_dir_.c_str(), record_segment_s_, record_max_mb_,
                publish_commands_ ? "true" : "false", stats_log_ms_);
    RCLCPP_INFO(get_logger(),
                "voice_command pipeline (denoise=%s speex_noise_suppress_db=%.1f "
                "partial_trigger=%s partial_stable_ms=%ld max_utterance_ms=%ld mixer_device=%s "
                "capture_mixer_control=%s capture_gain_percent=%ld)",
                denoise_.c_str(), speex_noise_suppress_db_, partial_trigger_ ? "true" : "false",
                partial_stable_ms_, max_utterance_ms_, mixer_device_.c_str(),
                capture_mixer_control_.c_str(), capture_gain_percent_);
    RCLCPP_INFO(get_logger(),
                "voice_command announce (playback_device=%s tts_command=%s tts_voice=%s "
                "tts_cache_dir=%s announce_timeout_ms=%ld pad_announce_window_ms=%ld "
                "mute_tail_ms=%ld)",
                playback_device_.c_str(), tts_command_.c_str(), tts_voice_.c_str(),
                tts_cache_dir_.c_str(), announce_timeout_ms_, pad_announce_window_ms_,
                mute_tail_ms_);

    if (!record_dir_.empty()) {
      AudioRecorder::Config record_config;
      record_config.dir = record_dir_;
      record_config.raw_rate = static_cast<uint32_t>(capture_rate_);
      record_config.raw_channels = static_cast<uint16_t>(capture_channels_);
      record_config.asr_rate = static_cast<uint32_t>(sample_rate_);
      record_config.segment_s = record_segment_s_;
      record_config.max_bytes = static_cast<uint64_t>(record_max_mb_) * 1024u * 1024u;
      const rclcpp::Logger logger = get_logger();
      recorder_ = std::make_unique<AudioRecorder>(record_config,
                                                  [logger](bool warn, const std::string& text) {
                                                    if (warn) {
                                                      RCLCPP_WARN(logger, "%s", text.c_str());
                                                    } else {
                                                      RCLCPP_INFO(logger, "%s", text.c_str());
                                                    }
                                                  });
    }

    synthesize_tts_cache();

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&VoiceCommandNode::on_control_state, this, std::placeholders::_1));
    toggle_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/control/mode_toggle", last_qos,
        std::bind(&VoiceCommandNode::on_mode_toggle, this, std::placeholders::_1));

    expire_timer_ = create_wall_timer(std::chrono::milliseconds(75),
                                      std::bind(&VoiceCommandNode::on_expire_timer, this));

    running_ = true;
    playback_thread_ = std::thread([this] { playback_loop(); });

    const std::string model_dir = resolve_model_dir(model_dir_param);
    vosk_set_log_level(0);
    model_ = vosk_model_new(model_dir.c_str());
    if (model_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "model load failed dir=%s", model_dir.c_str());
      return;
    }

    auto engine = std::make_unique<VoskEngine>(model_, static_cast<float>(sample_rate_), phrases_);
    if (!engine->ok()) {
      RCLCPP_ERROR(get_logger(), "model load failed dir=%s", model_dir.c_str());
      vosk_model_free(model_);
      model_ = nullptr;
      return;
    }

    PipelineConfig pipeline_config;
    pipeline_config.capture_rate = static_cast<unsigned>(capture_rate_);
    pipeline_config.asr_rate = static_cast<unsigned>(sample_rate_);
    pipeline_config.channels = static_cast<unsigned>(capture_channels_);
    pipeline_config.denoise = denoise_;
    pipeline_config.denoise_options.speex_noise_suppress_db = speex_noise_suppress_db_;
    pipeline_config.partial_trigger = partial_trigger_;
    pipeline_config.partial_stable = std::chrono::milliseconds(partial_stable_ms_);
    pipeline_config.max_utterance = std::chrono::milliseconds(max_utterance_ms_);
    pipeline_config.phrases = phrases_;
    pipeline_config.commands = commands_;
    pipeline_config.min_confidence = min_confidence_;
    pipeline_ = std::make_unique<CommandPipeline>(pipeline_config, std::move(engine));
    if (!pipeline_->denoise_error().empty()) {
      RCLCPP_ERROR(get_logger(), "denoise init failed kind=%s: %s, using none", denoise_.c_str(),
                   pipeline_->denoise_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "denoise ready kind=%s rate=%ld", pipeline_->denoise_name(),
                  capture_rate_);
    }

    capture_thread_ = std::thread([this] { capture_loop(); });
  }

  ~VoiceCommandNode() override {
    running_ = false;
    playback_cv_.notify_all();
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    close_capture();
    recorder_.reset();
    pipeline_.reset();
    if (model_ != nullptr) {
      vosk_model_free(model_);
      model_ = nullptr;
    }
  }

 private:
  enum class AnnounceTrigger { None, Voice, Pad };

  void synthesize_tts_cache() {
    std::error_code ec;
    std::filesystem::create_directories(tts_cache_dir_, ec);
    tts_clips_.resize(phrases_.size());
    for (size_t i = 0; i < phrases_.size(); ++i) {
      const std::string wav_path = tts_cache_dir_ + "/" + std::to_string(i) + ".wav";
      const std::vector<std::string> args = {"-p", tts_voice_, "-o", wav_path};
      const std::string err = run_tts_synthesis(tts_command_, args, phrases_[i]);
      if (!err.empty()) {
        RCLCPP_ERROR(get_logger(), "tts failed \"%s\": %s", phrases_[i].c_str(), err.c_str());
        tts_clips_[i] = std::nullopt;
        continue;
      }
      const auto clip = read_wav_pcm16(wav_path);
      if (!clip.has_value()) {
        RCLCPP_ERROR(get_logger(), "tts failed \"%s\": %s", phrases_[i].c_str(), "invalid wav");
        tts_clips_[i] = std::nullopt;
        continue;
      }
      tts_clips_[i] = *clip;
      RCLCPP_INFO(get_logger(), "tts ready \"%s\" -> %s (%u Hz)", phrases_[i].c_str(),
                  wav_path.c_str(), clip->sample_rate);
    }
  }

  std::optional<size_t> phrase_index_for_state(uint8_t state) const {
    for (size_t i = 0; i < states_.size(); ++i) {
      if (static_cast<uint8_t>(states_[i]) == state) {
        return i;
      }
    }
    return std::nullopt;
  }

  void announce_mode(uint8_t mode, const char* trigger) {
    const auto idx = phrase_index_for_state(mode);
    if (!idx.has_value() || !tts_clips_[*idx].has_value()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "announce \"%s\" trigger=%s", phrases_[*idx].c_str(), trigger);
    enqueue_playback(*idx);
  }

  void enqueue_playback(size_t phrase_index) {
    {
      std::lock_guard<std::mutex> lock(playback_mutex_);
      pending_playback_ = phrase_index;
    }
    playback_cv_.notify_one();
  }

  void on_control_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(policy_mutex_);
    current_state_ = msg->state;
    have_current_state_ = true;
    if (const auto mode = announce_policy_.on_state(msg->state, now)) {
      const char* trigger = "pad";
      if (pending_announce_trigger_ == AnnounceTrigger::Voice) {
        trigger = "voice";
      }
      pending_announce_trigger_ = AnnounceTrigger::None;
      announce_mode(*mode, trigger);
    }
  }

  void on_mode_toggle(const std_msgs::msg::Empty::SharedPtr) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(policy_mutex_);
    pending_announce_trigger_ = AnnounceTrigger::Pad;
    announce_policy_.on_pad_toggle(now);
  }

  void on_expire_timer() {
    if (!running_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(policy_mutex_);
    if (const auto missed = announce_policy_.expire(now)) {
      RCLCPP_WARN(get_logger(), "announce skipped: state %u not reached in %ld ms", *missed,
                  announce_timeout_ms_);
    }
  }

  bool play_pcm_clip(const PcmClip& clip) {
    snd_pcm_t* pcm = nullptr;
    int err = snd_pcm_open(&pcm, playback_device_open_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
      RCLCPP_WARN(get_logger(), "playback failed device=%s: %s", playback_device_.c_str(),
                  snd_strerror(err));
      return false;
    }
    err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                             clip.channels, clip.sample_rate, 1, 500000);
    if (err < 0) {
      RCLCPP_WARN(get_logger(), "playback failed device=%s: %s", playback_device_.c_str(),
                  snd_strerror(err));
      snd_pcm_close(pcm);
      return false;
    }

    const size_t frame_count = clip.samples.size() / clip.channels;
    const auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(static_cast<double>(frame_count) / clip.sample_rate));
    const auto start = std::chrono::steady_clock::now();
    const auto end = start + duration;
    {
      std::lock_guard<std::mutex> lock(mute_mutex_);
      mute_window_.on_playback(start, end);
    }

    const snd_pcm_uframes_t chunk = 1024;
    size_t offset = 0;
    while (offset < frame_count && running_) {
      const snd_pcm_uframes_t frames =
          std::min<snd_pcm_uframes_t>(chunk, static_cast<snd_pcm_uframes_t>(frame_count - offset));
      const int16_t* data = clip.samples.data() + offset * clip.channels;
      snd_pcm_sframes_t written = snd_pcm_writei(pcm, data, frames);
      if (written < 0) {
        written = snd_pcm_recover(pcm, static_cast<int>(written), 1);
        if (written < 0) {
          RCLCPP_WARN(get_logger(), "playback failed device=%s: %s", playback_device_.c_str(),
                      snd_strerror(static_cast<int>(written)));
          snd_pcm_close(pcm);
          return false;
        }
        continue;
      }
      offset += static_cast<size_t>(written);
    }
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
    return true;
  }

  void playback_loop() {
    while (running_) {
      std::optional<size_t> phrase_index;
      {
        std::unique_lock<std::mutex> lock(playback_mutex_);
        playback_cv_.wait(lock, [this] { return !running_ || pending_playback_.has_value(); });
        if (!running_) {
          break;
        }
        phrase_index = pending_playback_;
        pending_playback_.reset();
      }
      if (!phrase_index.has_value() || phrase_index >= tts_clips_.size() ||
          !tts_clips_[*phrase_index].has_value()) {
        continue;
      }
      play_pcm_clip(*tts_clips_[*phrase_index]);
    }
  }

  void close_capture() {
    if (pcm_ != nullptr) {
      snd_pcm_close(pcm_);
      pcm_ = nullptr;
    }
  }

  // SD033 D5: set the gain on every capture open, then check it on every stats line.
  void apply_capture_gain(bool announce) {
    if (capture_gain_percent_ < 0) {
      return;
    }
    long current = -1;
    std::string error;
    if (!capture_gain_percent(mixer_device_, capture_mixer_control_, capture_gain_percent_,
                              &current, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "mixer failed device=%s: %s",
                           mixer_device_.c_str(), error.c_str());
      gain_now_ = -1;
      return;
    }
    gain_now_ = current;
    if (announce) {
      RCLCPP_INFO(get_logger(), "mixer gain set control=%s percent=%ld",
                  capture_mixer_control_.c_str(), current);
    }
  }

  void check_capture_gain() {
    if (capture_gain_percent_ < 0) {
      return;
    }
    long current = -1;
    std::string error;
    if (!capture_gain_percent(mixer_device_, capture_mixer_control_, -1, &current, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "mixer failed device=%s: %s",
                           mixer_device_.c_str(), error.c_str());
      gain_now_ = -1;
      return;
    }
    gain_now_ = current;
    if (std::labs(current - capture_gain_percent_) > 1) {
      RCLCPP_WARN(get_logger(), "mixer gain drift control=%s now=%ld%% target=%ld%%, reapply",
                  capture_mixer_control_.c_str(), current, capture_gain_percent_);
      apply_capture_gain(false);
    }
  }

  bool open_capture() {
    snd_pcm_t* pcm = nullptr;
    int err = snd_pcm_open(&pcm, capture_device_open_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "capture open failed device=%s: %s",
                           capture_device_.c_str(), snd_strerror(err));
      return false;
    }
    err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                             static_cast<unsigned>(capture_channels_),
                             static_cast<unsigned>(capture_rate_), 1, 100000);
    if (err < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "capture open failed device=%s: %s",
                           capture_device_.c_str(), snd_strerror(err));
      snd_pcm_close(pcm);
      return false;
    }
    pcm_ = pcm;
    apply_capture_gain(true);
    return true;
  }

  bool is_muted() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mute_mutex_);
    return mute_window_.muted(now);
  }

  void handle_event(const PipelineEvent& event) {
    if (!event.command) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ignored \"%s\" conf_min=%.2f via=%s reason=%s", event.text.c_str(),
                           event.conf_min, event.via, event.reason);
      return;
    }
    if (!event.phrase.has_value() || *event.phrase >= commands_.size()) {
      return;
    }
    const size_t idx = *event.phrase;
    if (!publish_commands_) {
      RCLCPP_INFO(get_logger(), "heard \"%s\" conf_min=%.2f via=%s -> %s (dry run)",
                  event.text.c_str(), event.conf_min, event.via, commands_[idx].c_str());
      return;
    }

    mentorpi_msgs::msg::NamedCommand msg;
    msg.name = commands_[idx];
    msg.source = "voice";
    command_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "heard \"%s\" conf_min=%.2f via=%s -> %s", event.text.c_str(),
                event.conf_min, event.via, commands_[idx].c_str());

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(policy_mutex_);
    pending_announce_trigger_ = AnnounceTrigger::Voice;
    announce_policy_.on_voice_command(static_cast<uint8_t>(states_[idx]), now);
    if (have_current_state_) {
      if (const auto mode = announce_policy_.on_state(current_state_, now)) {
        pending_announce_trigger_ = AnnounceTrigger::None;
        announce_mode(*mode, "voice");
      }
    }
  }

  void log_stats_if_due(std::chrono::steady_clock::time_point& last_stats) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats < std::chrono::milliseconds(stats_log_ms_)) {
      return;
    }
    last_stats = now;
    check_capture_gain();
    const LevelStats level = level_meter_.take();
    const PipelineTiming timing = pipeline_->take_timing();
    const double denoise_ms_per_s = timing.audio_s > 0.0 ? timing.denoise_ms / timing.audio_s : 0.0;
    const double asr_ms_per_s = timing.audio_s > 0.0 ? timing.asr_ms / timing.audio_s : 0.0;
    RCLCPP_INFO(get_logger(),
                "audio stats peak_dbfs=%.1f rms_dbfs=%.1f clipped=%.3f%% denoise_ms_per_s=%.1f "
                "asr_ms_per_s=%.1f gain=%ld%%",
                level.peak_dbfs, level.rms_dbfs, level.clipped_share * 100.0, denoise_ms_per_s,
                asr_ms_per_s, gain_now_);
  }

  void capture_loop() {
    auto last_retry = std::chrono::steady_clock::now() - retry_;
    auto last_stats = std::chrono::steady_clock::now();
    const auto channels = static_cast<size_t>(capture_channels_);
    const snd_pcm_uframes_t period =
        static_cast<snd_pcm_uframes_t>(std::max<int64_t>(1, capture_rate_ / 10));
    std::vector<int16_t> interleaved(static_cast<size_t>(period) * channels);
    std::vector<int16_t> mono;
    std::vector<PipelineEvent> events;
    bool was_muted = false;

    while (running_ && rclcpp::ok()) {
      if (pcm_ == nullptr) {
        const auto now_st = std::chrono::steady_clock::now();
        if (now_st - last_retry < retry_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        last_retry = now_st;
        if (!open_capture()) {
          continue;
        }
      }

      const snd_pcm_sframes_t nread = snd_pcm_readi(pcm_, interleaved.data(), period);
      if (!running_) {
        break;
      }
      if (nread == -EAGAIN) {
        continue;
      }
      if (nread < 0) {
        const int recovered = snd_pcm_recover(pcm_, static_cast<int>(nread), 1);
        if (recovered < 0) {
          close_capture();
        }
        continue;
      }
      if (nread == 0) {
        continue;
      }

      const auto frames = static_cast<size_t>(nread);
      if (recorder_) {
        recorder_->push_raw(interleaved.data(), frames * channels);
      }
      mix_to_mono(interleaved.data(), frames, static_cast<unsigned>(channels), mono);
      level_meter_.add(mono.data(), mono.size());
      log_stats_if_due(last_stats);

      if (is_muted()) {
        if (!was_muted) {
          pipeline_->reset();
        }
        was_muted = true;
        continue;
      }
      was_muted = false;

      pipeline_->feed(interleaved.data(), frames, events);
      const std::vector<int16_t>& asr_block = pipeline_->last_asr_block();
      if (recorder_ && !asr_block.empty()) {
        recorder_->push_asr(asr_block.data(), asr_block.size());
      }
      for (const auto& event : events) {
        handle_event(event);
      }
    }
    close_capture();
  }

  std::string capture_device_;
  std::string capture_device_open_;
  int64_t capture_rate_{48000};
  int64_t sample_rate_{16000};
  int64_t capture_channels_{1};
  std::vector<std::string> phrases_;
  std::vector<std::string> commands_;
  std::vector<int64_t> states_;
  double min_confidence_{0.6};
  std::chrono::milliseconds retry_{1000};

  std::string denoise_{"speexdsp"};
  double speex_noise_suppress_db_{-30.0};
  bool partial_trigger_{true};
  int64_t partial_stable_ms_{200};
  int64_t max_utterance_ms_{5000};
  std::string mixer_device_;
  std::string capture_mixer_control_;
  int64_t capture_gain_percent_{-1};

  std::string record_dir_;
  int64_t record_segment_s_{300};
  int64_t record_max_mb_{500};
  bool publish_commands_{true};
  int64_t stats_log_ms_{10000};

  std::string playback_device_;
  std::string playback_device_open_;
  std::string tts_command_;
  std::string tts_voice_;
  std::string tts_cache_dir_;
  int64_t announce_timeout_ms_{1000};
  int64_t pad_announce_window_ms_{1000};
  int64_t mute_tail_ms_{300};

  std::vector<std::optional<PcmClip>> tts_clips_;
  AnnouncePolicy announce_policy_{std::chrono::milliseconds(0), std::chrono::milliseconds(0)};
  MuteWindow mute_window_{std::chrono::milliseconds(0)};

  std::mutex policy_mutex_;
  std::mutex mute_mutex_;
  std::mutex playback_mutex_;
  std::condition_variable playback_cv_;
  std::optional<size_t> pending_playback_;
  AnnounceTrigger pending_announce_trigger_{AnnounceTrigger::None};
  uint8_t current_state_{0};
  bool have_current_state_{false};

  // Capture thread only.
  LevelMeter level_meter_;
  std::unique_ptr<AudioRecorder> recorder_;
  std::unique_ptr<CommandPipeline> pipeline_;
  long gain_now_{-1};

  VoskModel* model_{nullptr};
  snd_pcm_t* pcm_{nullptr};

  rclcpp::Publisher<mentorpi_msgs::msg::NamedCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr toggle_sub_;
  rclcpp::TimerBase::SharedPtr expire_timer_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread playback_thread_;
};

}  // namespace mentorpi_voice

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_voice::VoiceCommandNode>());
  rclcpp::shutdown();
  return 0;
}
