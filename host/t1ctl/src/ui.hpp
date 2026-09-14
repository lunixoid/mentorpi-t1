#pragma once

#include "calib.hpp"
#include "units.hpp"

namespace ui {

void print_help();
void print_version();
void print_status(const units::Status& status);
void print_started();
void print_restarted();
void print_stock();
void print_action_error();
void print_mode_changed(const units::ModeChange& change);
void print_mode_error(const units::Status& status, const std::string& detail);
void print_calib(const calib::Show& show);
void print_calib_error(const calib::Show& show);
void print_calib_hold_line(const calib::Result& result);
void print_calib_hold(const calib::Result& result);
void print_calib_hold_finish();
void print_calib_proposal(const calib::Result& result);
void print_calib_side(const calib::Result& result);
void print_calib_save(const calib::Result& result);
void print_calib_stage_error(const calib::Result& result);
void print_calib_stage_phrase(const calib::Result& result, bool kept);

}  // namespace ui
