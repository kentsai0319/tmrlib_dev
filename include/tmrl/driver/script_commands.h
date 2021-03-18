#pragma once

#include "tmrl/types.h"

namespace tmrl
{
namespace driver
{

enum class IOModule { ControlBox, EndEffector };
enum class IOType { DI, DO, InstantDO, AI, AO, InstantAO };

enum class PvtMode { Joint, Tool };
struct PvtPoint {
  double time;
  vectorXd positions;
  vectorXd velocities;
};
struct PvtTraj {
  PvtMode mode;
  std::vector<PvtPoint> points;
  double total_time;
};

enum class VelMode { Joint, Tool };

namespace cmd
{

inline std::string script_exit() { return "ScriptExit()"; }

std::string queue_tag(int tag, int wait = 0);

std::string wait_queue_tag(int tag, int timeout_ms = 0);

inline std::string stop() { return "StopAndClearBuffer()"; }

inline std::string pause() { return "Pause()"; }

inline std::string resume() { return "Resume()"; }

std::string IO(IOModule module, IOType type, int pin, float state);

std::string PTP_J(const vector6d &angs,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision = 5);

std::string PTP_T(const PoseEular &pose,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision = 5);

std::string Line_T(const PoseEular &pose,
  double vel, double acc_time, int blend_percent, bool fine_goal, int precision = 5);

inline std::string pvt_enter(int mode) { return "PVTEnter(" + std::to_string(mode) + ")"; }

inline std::string pvt_exit() { return "PVTExit()"; }

std::string pvt_point(PvtMode mode,
  double t, const vectorXd &pos, const vectorXd &vel, int precision = 5);

inline std::string pvt_point(PvtMode mode, const PvtPoint &point, int precision = 5)
{
  return pvt_point(mode, point.time, point.positions, point.velocities, precision);
}

std::string pvt_traj(const PvtTraj &pvts, int precision = 5);


std::string vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop);
inline std::string vel_mode_stop() { return "StopContinueVmode()"; }
std::string vel_mode_target(VelMode mode, const vector6d &vel, int precision = 5);

}
}
}