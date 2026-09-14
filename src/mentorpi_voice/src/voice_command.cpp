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
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/named_command.hpp"
#include "mentorpi_voice/announce_policy.hpp"
#include "mentorpi_voice/utterance.hpp"
#include "mentorpi_voice/wav_pcm.hpp"
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

}  // namespace

class VoiceCommandNode : public rclcpp::Node {
 public:
  VoiceCommandNode() : Node("voice_command") {
    declare_parameter<std::string>("capture_device", "default");
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
                "voice_command started (capture_device=%s sample_rate=%ld capture_channels=%ld "
                "model_dir=%s phrases=%s commands=%s states=%s min_confidence=%.2f retry_ms=%ld)",
                capture_device_.c_str(), sample_rate_, capture_channels_, model_dir_param.c_str(),
                join_strings(phrases_).c_str(), join_strings(commands_).c_str(),
                join_ints(states_).c_str(), min_confidence_, retry_ms);
    RCLCPP_INFO(get_logger(),
                "voice_command announce (playback_device=%s tts_command=%s tts_voice=%s "
                "tts_cache_dir=%s announce_timeout_ms=%ld pad_announce_window_ms=%ld "
                "mute_tail_ms=%ld)",
                playback_device_.c_str(), tts_command_.c_str(), tts_voice_.c_str(),
                tts_cache_dir_.c_str(), announce_timeout_ms_, pad_announce_window_ms_,
                mute_tail_ms_);

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

    const std::string grammar = grammar_json(phrases_);
    rec_ = vosk_recognizer_new_grm(model_, static_cast<float>(sample_rate_), grammar.c_str());
    if (rec_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "model load failed dir=%s", model_dir.c_str());
      vosk_model_free(model_);
      model_ = nullptr;
      return;
    }
    vosk_recognizer_set_words(rec_, 1);

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
    if (rec_ != nullptr) {
      vosk_recognizer_free(rec_);
      rec_ = nullptr;
    }
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
                             static_cast<unsigned>(sample_rate_), 1, 100000);
    if (err < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "capture open failed device=%s: %s",
                           capture_device_.c_str(), snd_strerror(err));
      snd_pcm_close(pcm);
      return false;
    }
    pcm_ = pcm;
    return true;
  }

  bool is_muted() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mute_mutex_);
    return mute_window_.muted(now);
  }

  void handle_result(const char* json_text) {
    const CommandDecision decision =
        decide_command(parse_vosk_result(json_text), phrases_, commands_, min_confidence_);
    if (normalize_phrase(decision.text).empty()) {
      return;
    }
    if (decision.command.has_value()) {
      mentorpi_msgs::msg::NamedCommand msg;
      msg.name = *decision.command;
      msg.source = "voice";
      command_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "heard \"%s\" conf_min=%.2f -> %s", decision.text.c_str(),
                  decision.conf_min, decision.command->c_str());

      size_t cmd_idx = commands_.size();
      for (size_t i = 0; i < commands_.size(); ++i) {
        if (commands_[i] == *decision.command) {
          cmd_idx = i;
          break;
        }
      }
      if (cmd_idx >= commands_.size()) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(policy_mutex_);
      pending_announce_trigger_ = AnnounceTrigger::Voice;
      announce_policy_.on_voice_command(static_cast<uint8_t>(states_[cmd_idx]), now);
      if (have_current_state_) {
        if (const auto mode = announce_policy_.on_state(current_state_, now)) {
          pending_announce_trigger_ = AnnounceTrigger::None;
          announce_mode(*mode, "voice");
        }
      }
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "ignored \"%s\" conf_min=%.2f reason=%s",
                         decision.text.c_str(), decision.conf_min, decision.ignore_reason);
  }

  void capture_loop() {
    auto last_retry = std::chrono::steady_clock::now() - retry_;
    const snd_pcm_uframes_t period =
        static_cast<snd_pcm_uframes_t>(std::max<int64_t>(1, sample_rate_ / 10));
    std::vector<int16_t> interleaved(static_cast<size_t>(period) *
                                     static_cast<size_t>(capture_channels_));
    std::vector<int16_t> mono(static_cast<size_t>(period));
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

      const bool muted = is_muted();
      if (muted) {
        if (!was_muted && rec_ != nullptr) {
          vosk_recognizer_reset(rec_);
        }
        was_muted = true;
        continue;
      }
      was_muted = false;

      const size_t frames = static_cast<size_t>(nread);
      if (capture_channels_ == 1) {
        mono.assign(interleaved.begin(), interleaved.begin() + static_cast<std::ptrdiff_t>(frames));
      } else {
        mono.resize(frames);
        for (size_t i = 0; i < frames; ++i) {
          int sum = 0;
          for (int64_t ch = 0; ch < capture_channels_; ++ch) {
            sum +=
                interleaved[i * static_cast<size_t>(capture_channels_) + static_cast<size_t>(ch)];
          }
          mono[i] = static_cast<int16_t>(sum / capture_channels_);
        }
      }

      const int rc = vosk_recognizer_accept_waveform_s(rec_, mono.data(), static_cast<int>(frames));
      if (rc == 1) {
        handle_result(vosk_recognizer_result(rec_));
      } else if (rc < 0) {
        vosk_recognizer_reset(rec_);
      }
    }
    close_capture();
  }

  std::string capture_device_;
  std::string capture_device_open_;
  int64_t sample_rate_{16000};
  int64_t capture_channels_{1};
  std::vector<std::string> phrases_;
  std::vector<std::string> commands_;
  std::vector<int64_t> states_;
  double min_confidence_{0.6};
  std::chrono::milliseconds retry_{1000};

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

  VoskModel* model_{nullptr};
  VoskRecognizer* rec_{nullptr};
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
