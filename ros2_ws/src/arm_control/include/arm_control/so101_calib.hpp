// Shared SO-101 calibration JSON (from calibrate_so101).
#pragma once

#include <array>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "arm_control/arm_types.hpp"

namespace arm_control {

struct JointCalib {
  int id = 0;
  int zero_ticks = 0;
  int min_ticks = 0;
  int max_ticks = 4095;
  int sign = 1;
};

inline int calib_json_int(const std::string& block, const char* key) {
  const std::string pat = std::string("\"") + key + "\":";
  const auto pos = block.find(pat);
  if (pos == std::string::npos) {
    throw std::runtime_error(std::string("missing ") + key);
  }
  return std::atoi(block.c_str() + pos + pat.size());
}

inline std::array<JointCalib, kDof> load_so101_calib(const std::string& path) {
  static const char* kNames[kDof] = {"shoulder_pan", "shoulder_lift",
                                     "elbow_flex",   "wrist_flex",
                                     "wrist_roll",   "gripper"};
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::stringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();
  std::array<JointCalib, kDof> out;
  for (int i = 0; i < kDof; ++i) {
    const std::string key = std::string("\"") + kNames[i] + "\"";
    const auto start = text.find(key);
    if (start == std::string::npos) {
      throw std::runtime_error(std::string("missing joint ") + kNames[i]);
    }
    const auto brace = text.find('{', start);
    const auto end = text.find('}', brace);
    const std::string block = text.substr(brace, end - brace + 1);
    out[i].id = calib_json_int(block, "id");
    out[i].zero_ticks = calib_json_int(block, "zero_ticks");
    out[i].min_ticks = calib_json_int(block, "min_ticks");
    out[i].max_ticks = calib_json_int(block, "max_ticks");
    try {
      out[i].sign = calib_json_int(block, "sign");
    } catch (const std::runtime_error&) {
      out[i].sign = 1;
    }
    if (out[i].sign != 1 && out[i].sign != -1) {
      throw std::runtime_error("sign must be ±1");
    }
  }
  return out;
}

}  // namespace arm_control
