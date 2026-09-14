#include <CLI/CLI.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "calib.hpp"
#include "ui.hpp"
#include "units.hpp"

namespace {

int run_status() {
  ui::print_status(units::query());
  return 0;
}

int run_start() {
  if (!units::start()) {
    ui::print_action_error();
    return 1;
  }
  ui::print_started();
  return 0;
}

int run_restart() {
  if (!units::restart()) {
    ui::print_action_error();
    return 1;
  }
  ui::print_restarted();
  return 0;
}

int run_stock() {
  if (!units::stock()) {
    ui::print_action_error();
    return 1;
  }
  ui::print_stock();
  return 0;
}

int run_mode(units::ModeCommand command) {
  units::ModeChange change;
  units::Status known;
  if (!units::set_mode(units::process_runner(), command, &change, &known)) {
    ui::print_mode_error(known, change.detail);
    return 1;
  }
  ui::print_mode_changed(change);
  return 0;
}

int run_calib_show() {
  calib::Show show;
  if (!calib::query(units::process_runner(), &show)) {
    ui::print_calib_error(show);
    return 1;
  }
  ui::print_calib(show);
  return 0;
}

bool result_is_proposal(const calib::Result& result) {
  return !result.fields.empty() || (result.have_residual_before && result.have_residual_after) ||
         (result.have_imu_residual_before && result.have_imu_residual_after);
}

bool result_is_hold(const calib::Result& result) {
  return result.have_frames || result.have_cloud_points || result.have_scan_rays ||
         result.have_imu_samples || result.have_travel || result.have_turn || !result.hints.empty();
}

int run_calib_stage(const std::vector<std::string>& subcommand) {
  calib::Result result;
  calib::Result hold;
  auto on_line = [&](const std::string& line) {
    calib::Result line_result;
    if (!calib::parse_result_line(line, &line_result)) {
      return;
    }
    if (!line_result.stage.empty()) {
      hold.stage = line_result.stage;
    }
    if (line_result.clear_hints) {
      hold.hints.clear();
      hold.clear_hints = true;
    }
    if (!line_result.hints.empty()) {
      for (const auto& hint : line_result.hints) {
        if (hold.hints.empty() || hold.hints.back() != hint) {
          hold.hints.push_back(hint);
        }
      }
    }
    if (line_result.have_frames) {
      hold.have_frames = true;
      hold.frames = line_result.frames;
    }
    if (line_result.have_cloud_points) {
      hold.have_cloud_points = true;
      hold.cloud_points = line_result.cloud_points;
    }
    if (line_result.have_scan_rays) {
      hold.have_scan_rays = true;
      hold.scan_rays = line_result.scan_rays;
    }
    if (line_result.have_imu_samples) {
      hold.have_imu_samples = true;
      hold.imu_samples = line_result.imu_samples;
    }
    if (line_result.have_travel) {
      hold.have_travel = true;
      hold.travel_m = line_result.travel_m;
    }
    if (line_result.have_turn) {
      hold.have_turn = true;
      hold.turn_deg = line_result.turn_deg;
    }
    ui::print_calib_hold_line(hold);
    hold.clear_hints = false;
  };
  const bool ok = calib::run_command(units::process_runner(), subcommand, &result, on_line);
  ui::print_calib_hold_finish();
  if (!ok) {
    if (result.stage.empty() && !subcommand.empty()) {
      result.stage = subcommand.front();
    }
    ui::print_calib_stage_error(result);
    return 1;
  }
  if (result.stage == "side") {
    ui::print_calib_side(result);
    return 0;
  }
  if (result_is_proposal(result)) {
    ui::print_calib_proposal(result);
    return 0;
  }
  if (result_is_hold(result)) {
    ui::print_calib_hold(result);
  }
  return 0;
}

int run_calib_action(const std::vector<std::string>& subcommand, bool kept) {
  calib::Result result;
  const bool ok = calib::run_command(units::process_runner(), subcommand, &result);
  if (!ok) {
    if (result.stage.empty() && subcommand.size() == 1 && subcommand[0] != "abort") {
      result.stage = subcommand[0];
    }
    ui::print_calib_stage_error(result);
    return 1;
  }
  if (subcommand.size() == 1 && subcommand[0] == "save") {
    ui::print_calib_save(result);
    return 0;
  }
  if (subcommand.size() == 1 && subcommand[0] == "abort") {
    std::cout << "Draft dropped.\n";
    return 0;
  }
  ui::print_calib_stage_phrase(result, kept);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"", "t1ctl"};
  app.set_help_flag("-h,--help", "this help");
  bool show_version = false;
  app.add_flag("-V,--version", show_version, "print version");

  app.add_subcommand("status");
  auto* cmd_start = app.add_subcommand("start");
  auto* cmd_restart = app.add_subcommand("restart");
  auto* cmd_stock = app.add_subcommand("stock");
  auto* cmd_mode = app.add_subcommand("mode", "set control mode");
  auto* mode_forbid = cmd_mode->add_subcommand("forbid", "hold motion (Forbidden)");
  auto* mode_allow = cmd_mode->add_subcommand("allow", "release hold, return to follow");
  auto* mode_manual = cmd_mode->add_subcommand("manual", "select operator pad");
  cmd_mode->require_subcommand(1, 1);
  auto* cmd_calib = app.add_subcommand("calib", "sensor extrinsics");
  cmd_calib->add_subcommand("status", "print live poses and residual");
  auto* calib_corner = cmd_calib->add_subcommand("corner", "camera to lidar at a corner");
  auto* calib_drive = cmd_calib->add_subcommand("drive", "lidar yaw from a pad run");
  auto* calib_side = cmd_calib->add_subcommand("side", "transverse sign from a side object");
  auto* calib_lidar = cmd_calib->add_subcommand("lidar", "enter lidar height and tilt");
  auto* calib_camera = cmd_calib->add_subcommand("camera", "enter camera height");
  cmd_calib->add_subcommand("accept", "keep last stage as draft");
  cmd_calib->add_subcommand("reject", "drop last stage");
  cmd_calib->add_subcommand("save", "write file; live after restart");
  cmd_calib->add_subcommand("abort", "drop draft; file unchanged");
  cmd_calib->require_subcommand(0, 1);

  double corner_timeout = 0;
  double drive_timeout = 0;
  double side_timeout = 0;
  std::string side_side;
  calib_corner->add_option("--timeout", corner_timeout, "sample collection timeout in seconds");
  calib_drive->add_option("--timeout", drive_timeout, "sample collection timeout in seconds");
  calib_side->add_option("--side", side_side, "object side left or right")->required();
  calib_side->add_option("--timeout", side_timeout, "sample collection timeout in seconds");

  double lidar_height = 0;
  double lidar_pitch = 0;
  double lidar_roll = 0;
  calib_lidar->add_option("--height", lidar_height, "lidar height in metres")->required();
  calib_lidar->add_option("--pitch", lidar_pitch, "lidar pitch in degrees")->required();
  calib_lidar->add_option("--roll", lidar_roll, "lidar roll in degrees")->required();

  double camera_height = 0;
  calib_camera->add_option("--height", camera_height, "camera height in metres")->required();

  app.require_subcommand(0, 1);

  try {
    app.parse(argc, argv);
  } catch (const CLI::CallForHelp&) {
    ui::print_help();
    return 0;
  } catch (const CLI::ParseError& e) {
    if (e.get_exit_code() == 0) {
      ui::print_help();
      return 0;
    }
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  if (show_version) {
    ui::print_version();
    return 0;
  }
  if (cmd_start->parsed()) {
    return run_start();
  }
  if (cmd_restart->parsed()) {
    return run_restart();
  }
  if (cmd_stock->parsed()) {
    return run_stock();
  }
  if (cmd_mode->parsed()) {
    if (mode_forbid->parsed()) {
      return run_mode(units::ModeCommand::Forbid);
    }
    if (mode_allow->parsed()) {
      return run_mode(units::ModeCommand::Allow);
    }
    if (mode_manual->parsed()) {
      return run_mode(units::ModeCommand::Manual);
    }
    std::cerr << "error: missing mode subcommand\n";
    return 1;
  }
  if (cmd_calib->parsed()) {
    if (calib_corner->parsed()) {
      std::vector<std::string> args = {"corner"};
      if (corner_timeout > 0) {
        args.push_back("--timeout");
        args.push_back(std::to_string(corner_timeout));
      }
      return run_calib_stage(args);
    }
    if (calib_drive->parsed()) {
      std::vector<std::string> args = {"drive"};
      if (drive_timeout > 0) {
        args.push_back("--timeout");
        args.push_back(std::to_string(drive_timeout));
      }
      return run_calib_stage(args);
    }
    if (calib_side->parsed()) {
      std::vector<std::string> args = {"side", "--side", side_side};
      if (side_timeout > 0) {
        args.push_back("--timeout");
        args.push_back(std::to_string(side_timeout));
      }
      return run_calib_stage(args);
    }
    if (calib_lidar->parsed()) {
      return run_calib_stage({"lidar", "--height", std::to_string(lidar_height), "--pitch",
                              std::to_string(lidar_pitch), "--roll", std::to_string(lidar_roll)});
    }
    if (calib_camera->parsed()) {
      return run_calib_stage({"camera", "--height", std::to_string(camera_height)});
    }
    if (cmd_calib->get_subcommand("accept")->parsed()) {
      return run_calib_action({"accept"}, true);
    }
    if (cmd_calib->get_subcommand("reject")->parsed()) {
      return run_calib_action({"reject"}, false);
    }
    if (cmd_calib->get_subcommand("save")->parsed()) {
      return run_calib_action({"save"}, true);
    }
    if (cmd_calib->get_subcommand("abort")->parsed()) {
      return run_calib_action({"abort"}, false);
    }
    return run_calib_show();
  }
  return run_status();
}
