#include "tmrl/driver/script_commands.h"
#include "tmrl/utils/conversions.h"

#include <iomanip>

namespace tmrl
{
namespace driver
{

namespace cmd
{

std::string queue_tag(int tag, int wait)
{
  std::stringstream ss;
  ss << "QueueTag(" << tag << "," << wait << ")";
  return ss.str();
}
std::string wait_queue_tag(int tag, int timeout_ms)
{
  std::stringstream ss;
  ss << "WaitQueueTag(" << tag << "," << timeout_ms << ")";
  return ss.str();
}
std::string IO(IOModule module, IOType type, int pin, float state)
{
  static std::string io_module_name[] = { "ControlBox", "EndModule" };
  static std::string io_type_name[] = { "DI", "DO", "InstantDO", "AI", "AO", "InstantAO" };

  std::string script =
    "IO[" + io_module_name[(int)(module)]
    + "]." + io_type_name[(int)(type)]
    + "[" + std::to_string(pin) + "]=";
  if (type == IOType::DI || type == IOType::DO || type == IOType::InstantDO) {
    if (state == 0.0f)
      script += "0";
    else
      script += "1";
  }
  else {
    script += std::to_string(state);
  }
  return script;
}
std::string PTP_J(const vector6d &angs,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision)
{
  auto angs_deg = utils::degs(angs);
  int acct_ms = (int)(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PTP(\"JPP\",";
  for (auto &value : angs_deg) { ss << value << ","; }
  ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha << fine_goal << ")";
  return ss.str();
}
std::string PTP_T(const PoseEular &pose,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision)
{
  auto pose_mmdeg = utils::mmdeg(pose);
  int acct_ms = (int)(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PTP(\"CPP\",";
  for (auto &value : pose_mmdeg) { ss << value << ","; }
  ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha << fine_goal << ")";
  return ss.str();
}
std::string Line_T(const PoseEular &pose,
  double vel, double acc_time, int blend_percent, bool fine_goal, int precision)
{
  auto pose_mmdeg = utils::mmdeg(pose);
  int vel_mm = (int)(1000.0 * vel);
  int acct_ms = (int)(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "Line(\"CAP\",";
  for (auto &value : pose_mmdeg) { ss << value << ","; }
  ss << vel_mm << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha <<fine_goal << ")";
  return ss.str();
}
std::string pvt_point(PvtMode mode,
  double t, const vectorXd &pos, const vectorXd &vel, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PVTPoint(";
  if (mode == PvtMode::Joint) {
    for (auto &value : pos) { ss << utils::deg(value) << ","; }
    for (auto &value : vel) { ss << utils::deg(value) << ","; }
  }
  else {
    auto pv = utils::mmdeg(to_arrayd<6>(pos));
    for (auto &value : pv) { ss << value << ","; }
    auto vv = utils::mmdeg(to_arrayd<6>(vel));
    for (auto &value : vv) { ss << value << ","; }
  }
  ss << t << ")";
  return ss.str();
}
std::string pvt_traj(const PvtTraj &pvts, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  if (pvts.mode == PvtMode::Joint) {
    ss << "PVTEnter(0)\r\n";
    for (auto &point : pvts.points) {
      ss << "PVTPoint(";
      for (auto &value : point.positions) { ss << utils::deg(value) << ","; }
      for (auto &value : point.velocities) { ss << utils::deg(value) << ","; }
      ss << point.time << ")\r\n";
    }
  }
  else {
    ss << "PVTEnter(1)\r\n";
    for (auto &point : pvts.points) {
      ss << "PVTPoint(";
      auto pv = utils::mmdeg(to_arrayd<6>(point.positions));
      for (auto &value : pv) { ss << value << ","; }
      auto vv = utils::mmdeg(to_arrayd<6>(point.velocities));
      for (auto &value : vv) { ss << value << ","; }
      ss << point.time << ")\r\n";
    }
  }
  ss << "PVTExit()";
  return ss.str();
}


std::string vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop)
{
  if (mode == VelMode::Joint) {
    return "ContinueVJog()";
  }
  else {
    return "ContinueVLine(" +
      std::to_string((int)(1000.0 * timeout_zero_vel)) + ", " +
      std::to_string((int)(1000.0 * timeout_stop)) + ")";
  }
}
std::string vel_mode_target(VelMode mode, const vector6d &vel, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  if (mode == VelMode::Joint) {
    ss << "SetContinueVJog(";
    if (vel.size() > 0) {
      size_t i = 0;
      for (; i < vel.size() - 1; ++i) {
        ss << utils::deg(vel[i]) << ",";
      }
      ss << utils::deg(vel[i]);
    }
    ss << ")";
  }
  else {
    ss << "SetContinueVLine(";
    if (vel.size() >= 3) {
      size_t i = 0;
      for (; i < 3; ++i) {
        ss << (1000.0 * vel[i]) << ",";
      }
      for (; i < vel.size() - 1; ++i) {
        ss << utils::deg(vel[i]) << ",";
      }
      ss << utils::deg(vel[i]);
    }
    ss << ")";
  }
  return ss.str();
}

}
}
}