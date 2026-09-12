// Copied from ros2_ws/src/arm_control/include/arm_control/so101_calib.hpp.
// Looks up joints by name so it reads the same JSON calibrate_so101 writes.
#pragma once

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace so101_hardware {

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

inline std::string load_calib_text(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

inline JointCalib load_joint_calib(const std::string& text,
                                   const std::string& name) {
  const std::string key = std::string("\"") + name + "\"";
  const auto start = text.find(key);
  if (start == std::string::npos) {
    throw std::runtime_error("no calibration entry for joint " + name);
  }
  const auto brace = text.find('{', start);
  const auto end = text.find('}', brace);
  const std::string block = text.substr(brace, end - brace + 1);
  JointCalib out;
  out.id = calib_json_int(block, "id");
  out.zero_ticks = calib_json_int(block, "zero_ticks");
  out.min_ticks = calib_json_int(block, "min_ticks");
  out.max_ticks = calib_json_int(block, "max_ticks");
  try {
    out.sign = calib_json_int(block, "sign");
  } catch (const std::runtime_error&) {
    out.sign = 1;
  }
  if (out.sign != 1 && out.sign != -1) {
    throw std::runtime_error("sign must be ±1 for joint " + name);
  }
  return out;
}

}  // namespace so101_hardware
