#include "tmrl/driver/driver.h"

#include "tmrl/utils/logger.h"

namespace tmrl
{
namespace driver
{

Driver::Driver(TmsvrClient &svr, TmsctClient &sct)
  : state(svr.robot_state)
  , tmsvr(svr)
  , tmsct(sct)
{
}

bool Driver::start(bool stick_play)
{
  halt();
  tmrl_INFO_STREAM("TM_DRV: start");
  // connect to server
  bool rb = tmsvr.start();
  // send command to play project
  if (rb && stick_play) {
    send_stick_play();
  }
  // connect to listen node
  rb = tmsct.start();
  return rb;
}
void Driver::halt()
{
  tmrl_INFO_STREAM("TM_DRV: halt");
  stop_pvt_traj();
  if (tmsct.client().is_connected()) {
    set_script_exit();
  }
  tmsct.stop();
  if (tmsvr.client().is_connected()) {
    // send command to stop project
  }
  tmsvr.stop();
}

bool Driver::run_pvt_traj(const PvtTraj &pvts)
{
  auto time_start = std::chrono::steady_clock::now();
  auto time_now = time_start;

  if (pvts.points.size() == 0) return false;

  if (!tmsct.client().is_connected()) return false;

  if (!_keep_pvt_running) {
    _keep_pvt_running = true;
  }

  tmrl_INFO_STREAM("TM_DRV: traj. total time: " << pvts.total_time);

  if (!set_pvt_traj(pvts)) {
    _keep_pvt_running = false;
  }
  // TODO
  // check status and feedback
  // for now, only wait
  double time = 0.0;
  while (_keep_pvt_running && time < pvts.total_time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_now = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
    //time += 0.001;
  }

  tmrl_INFO_STREAM("TM_DRV: traj. exec. time: " << time);

  if (_keep_pvt_running) {
    _keep_pvt_running = false;
  }
  else {
    set_stop();
  }
  return true;
}
void Driver::stop_pvt_traj()
{
  _keep_pvt_running = false;
}

void Driver::cubic_interp(PvtPoint &p, const PvtPoint &p0, const PvtPoint &p1, double t)
{
  double c, d, T = p1.time;

  if (t < 0.0) t = 0.0;
  else if (t > T) t = T;

  p.time = t;

  for (size_t i = 0; i < p.positions.size(); ++i) {
    c = ((3.0 * (p1.positions[i] - p0.positions[i]) / T) - 2.0 * p0.velocities[i] - p1.velocities[i]) / T;
    d = ((2.0 * (p0.positions[i] - p1.positions[i]) / T) + p0.velocities[i] + p1.velocities[i]) / (T*T);
    p.positions[i] = p0.positions[i] + p0.velocities[i] * t + c * t*t + d * t*t*t;
    p.velocities[i] = p0.velocities[i] + 2.0 * c * t + 3.0 * d * t*t;
  }
}

bool Driver::fake_run_pvt_traj(const PvtTraj &pvts)
{
  const size_t dof = state.DOF;

  auto time_init = std::chrono::steady_clock::now();
  auto time_start = time_init;
  auto time_now = time_init;

  if (pvts.mode != PvtMode::Joint || pvts.points.size() < 2) return false;

  //for (auto &p : pvts.points) tmrl_INFO_STREAM(cmd::pvt_point(pvts.mode, p));

  if (!_keep_pvt_running) {
    _keep_pvt_running = true;
  }

  RobotState::Ulock lck(state.mtx);
  PvtPoint p_start;
  p_start.time = 0.0;
  p_start.positions = to_vectorXd(state.joint_angle());
  p_start.velocities = to_vectorXd(state.joint_speed());
  lck.unlock();

  PvtPoint &p0 = p_start;
  PvtPoint point = p_start;
  vector6d zeros{0};
  size_t idx = 0;

  // first point
  tmrl_INFO_STREAM("TM_DRV: traj. total time: " << pvts.total_time);
  tmrl_INFO_STREAM(cmd::pvt_point(pvts.mode, p0));
  tmrl_INFO_STREAM(cmd::pvt_point(pvts.mode, pvts.points[idx]));
  point.time = pvts.points[0].time;

  while (_keep_pvt_running) {
    cubic_interp(point, p0, pvts.points[idx], point.time);

    lck.lock();
    state.set_joint_states(to_arrayd<dof>(point.positions), to_arrayd<dof>(point.velocities), zeros);
    lck.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    time_now = std::chrono::steady_clock::now();
    point.time = 1.0 * std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
    if (point.time > pvts.points[idx].time) {
      p0 = pvts.points[idx];
      point.time -= pvts.points[idx].time;
      time_start = time_now;
      ++idx;
      if (idx == pvts.points.size()) break;

      tmrl_INFO_STREAM(cmd::pvt_point(pvts.mode, pvts.points[idx]));
    }
  }
  // last point
  if (_keep_pvt_running) {
    idx = pvts.points.size() - 1;
    cubic_interp(point, pvts.points[idx - 1], pvts.points[idx], pvts.points[idx].time);
  }

  lck.lock();
  state.set_joint_states(to_arrayd<dof>(point.positions), zeros, zeros);
  lck.unlock();

  time_now = std::chrono::steady_clock::now();
  point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_init).count();
  tmrl_INFO_STREAM("TM_DRV: traj. exec. time: " << point.time);

  if (_keep_pvt_running) {
    _keep_pvt_running = false;
  }
  return true;
}

}
}
