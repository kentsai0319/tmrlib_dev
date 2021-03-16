#pragma once

#include "tmrl/driver/tmsvr_client.h"
#include "tmrl/driver/tmsct_client.h"
#include "tmrl/driver/script_commands.h"

namespace tmrl
{
namespace driver
{

class Driver
{
public:
  RobotState &state;
  TmsvrClient &tmsvr;
  TmsctClient &tmsct;

  explicit Driver(TmsvrClient &svr, TmsctClient &sct);

  bool start(bool stick_play = false);

  void halt();

  ////////////////////////////////
  // TMSVR Robot Command (write_XXX | send_XXX)
  ////////////////////////////////

  bool send_stick_play();

  ////////////////////////////////
  // TMSCT Robot Command (set_XXX)
  ////////////////////////////////

  bool set_script_exit(const std::string &id = "Exit");
  bool set_tag(int tag, int wait = 0, const std::string &id = "Tag");
  bool set_wait_tag(int tag, int timeout_ms = 0, const std::string &id = "WaitTag");
  bool set_stop(const std::string &id = "Stop");
  bool set_pause(const std::string &id = "Pause");
  bool set_resume(const std::string &id = "Resume");

  bool set_io(IOModule module, IOType type, int pin, float state, const std::string &id = "io");
  bool set_joint_pos_PTP(const vector6d &angs,
    int vel_percent, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPJ");
  bool set_tool_pose_PTP(const PoseEular &pose,
    int vel_percent, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPT");
  bool set_tool_pose_Line(const PoseEular &pose,
    double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "Line");
  // set_tool_pose_PLine

  bool set_pvt_enter(PvtMode mode, const std::string &id = "PvtEnter");
  bool set_pvt_exit(const std::string &id = "PvtExit");
  bool set_pvt_point(PvtMode mode,
    double t, const vectorXd &pos, const vectorXd &vel, const std::string &id = "PvtPt");
  bool set_pvt_point(PvtMode mode, const PvtPoint &point, const std::string &id = "PvtPt");

  bool set_pvt_traj(const PvtTraj &pvts, const std::string &id = "PvtTraj");


  bool set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string &id = "VModeStart");
  bool set_vel_mode_stop(const std::string &id = "VModeStop");
  bool set_vel_mode_target(VelMode mode, const vector6d &vel, const std::string &id = "VModeTrgt");

  //
  // PVT motion
  //

  bool run_pvt_traj(const PvtTraj &pvts);
  void stop_pvt_traj();

  void cubic_interp(PvtPoint &p, const PvtPoint &p0, const PvtPoint &p1, double t);
  bool fake_run_pvt_traj(const PvtTraj &pvts);

private:
  bool _keep_pvt_running = false;
};

inline bool Driver::send_stick_play()
{
  return tmsvr.send_content("Play", "Stick_PlayPause=1");
}

inline bool Driver::set_script_exit(const std::string &id)
{
  return tmsct.send_script(id, cmd::script_exit());
}
inline bool Driver::set_tag(int tag, int wait, const std::string &id)
{
  return tmsct.send_script(id, cmd::queue_tag(tag, wait));
}
inline bool Driver::set_wait_tag(int tag, int timeout_ms, const std::string &id)
{
  return tmsct.send_script(id, cmd::wait_queue_tag(tag, timeout_ms));
}
inline bool Driver::set_stop(const std::string &id)
{
  return tmsct.send_script(id, cmd::stop());
}
inline bool Driver::set_pause(const std::string &id)
{
  return tmsct.send_script(id, cmd::pause());
}
inline bool Driver::set_resume(const std::string &id)
{
  return tmsct.send_script(id, cmd::resume());
}
inline bool Driver::set_io(IOModule module, IOType type, int pin, float state, const std::string &id)
{
  return tmsct.send_script(id, cmd::IO(module, type, pin, state));
}
inline bool Driver::set_joint_pos_PTP(const vector6d &angs,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
  return tmsct.send_script(id, cmd::PTP_J(angs, vel_percent, acc_time, blend_percent, fine_goal));
}
inline bool Driver::set_tool_pose_PTP(const PoseEular &pose,
  int vel_percent, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
  return tmsct.send_script(id, cmd::PTP_T(pose, vel_percent, acc_time, blend_percent, fine_goal));
}
inline bool Driver::set_tool_pose_Line(const PoseEular &pose,
  double vel, double acc_time, int blend_percent, bool fine_goal, const std::string &id)
{
  return tmsct.send_script(id, cmd::Line_T(pose, vel, acc_time, blend_percent, fine_goal));
}
inline bool Driver::set_pvt_enter(PvtMode mode, const std::string &id)
{
  return tmsct.send_script(id, cmd::pvt_enter((int)(mode)));
}
inline bool Driver::set_pvt_exit(const std::string &id)
{
  return tmsct.send_script(id, cmd::pvt_exit());
}
inline bool Driver::set_pvt_point(PvtMode mode,
  double t, const vectorXd &pos, const vectorXd &vel, const std::string &id)
{
  return tmsct.send_script(id, cmd::pvt_point(mode, t, pos, vel), comm::Client::LOG_NOTHING);
}
inline bool Driver::set_pvt_point(PvtMode mode, const PvtPoint &point, const std::string &id)
{
  return tmsct.send_script(id, cmd::pvt_point(mode, point));//, comm::Client::LOG_NOTHING);
}
inline bool Driver::set_pvt_traj(const PvtTraj &pvts, const std::string &id)
{
  return tmsct.send_script(id, cmd::pvt_traj(pvts));//, comm::Client::LOG_NOTHING);
}

inline bool Driver::set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string &id)
{
  return tmsct.send_script(id, cmd::vel_mode_start(mode, timeout_zero_vel, timeout_stop));
}
inline bool Driver::set_vel_mode_stop(const std::string &id)
{
  return tmsct.send_script(id, cmd::vel_mode_stop());
}
inline bool Driver::set_vel_mode_target(VelMode mode, const vector6d &vel, const std::string &id)
{
  return tmsct.send_script(id, cmd::vel_mode_target(mode, vel), comm::Client::LOG_NOTHING);
}

}
}
