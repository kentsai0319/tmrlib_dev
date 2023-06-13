#include "tmrl/driver/sim_pvt_motion.h"

#include "tmrl/utils/logger.h"

namespace tmrl
{
namespace driver
{

SimPvtMotion::SimPvtMotion(RobotState &state)
  : rs_(state)
{
  remaining_time_ = 0.0;
  pvt_count_ = 0;
  pvt_buffer_.clear();

  pvt_curr_.positions = to_vectorXd(rs_.joint_angle());
  pvt_curr_.velocities = std::vector<double>(rs_.DOF, 0.0);
  pvt_trgt_ = pvt_curr_;

  keep_alive_ = true;
  pvt_thread_ = std::thread(std::bind(&SimPvtMotion::run, this));
}

void SimPvtMotion::stop()
{
  keep_alive_ = enabled = false;
  if (pvt_thread_.joinable()) pvt_thread_.join();
}

void SimPvtMotion::enter(const vectorXd &init_joint_angle)
{
  if (enabled) return;

  enabled = true;
  // keep_alive_ = true;

  const size_t dof = rs_.DOF;
  const vector6d zeros{0};
  rs_.set_joint_states(to_arrayd<dof>(init_joint_angle), zeros, zeros);

  //pvt_thread_ = std::thread(std::bind(&SimPvtMotion::run, this));
  // std::thread(std::bind(&SimPvtMotion::run, this)).detach();
}
void SimPvtMotion::exit()
{
  enabled = false;
}

void SimPvtMotion::add_point(const PvtPoint &p)
{
  if (!enabled) return;
 
  double rt;
  size_t cnt;
  std::unique_lock<std::mutex> lck(pvt_mtx_);

  pvt_buffer_.push_back(p);
  cnt = pvt_count_ = pvt_buffer_.size();
  remaining_time_ += p.time;
  rt = remaining_time_;

  // lck.unlock();

  // tmrl_INFO_STREAM("++pvt, count: " << cnt << ", remaining_time: " << rt);
}

SimPvtMotion::State SimPvtMotion::idle()
{
  State state = IDLE;
  std::unique_lock<std::mutex> lck(pvt_mtx_);

  if (pvt_buffer_.size() > 0) {
    pvt_trgt_ = pvt_buffer_.front();

    lck.unlock();

    pvt_curr_.time = 0.0;

    tmrl_INFO_STREAM("moving...");
    state = MOVING;

    t_start_ = std::chrono::steady_clock::now();
    t_last_ = t_start_;
  }
  return state;
}
SimPvtMotion::State SimPvtMotion::moving()
{
  State state = MOVING;

  auto check_is_zeros = [](const std::vector<double> &vec)
  {
    bool is_zeros = true;
    for (auto &v : vec) {
      if (v >= 0.00001) { is_zeros = false; break; } 
    }
    return is_zeros;
  };

  // time
  auto t_curr = std::chrono::steady_clock::now();
  double t    = std::chrono::duration_cast<std::chrono::duration<double> >(t_curr - t_start_).count();
  double dur  = std::chrono::duration_cast<std::chrono::duration<double> >(t_curr - t_last_).count();
  t_last_ = t_curr;

  // interp
  PvtPoint p;
  p.positions.resize(pvt_curr_.positions.size());
  p.velocities.resize(pvt_curr_.velocities.size());
  interp(p, pvt_curr_, pvt_trgt_, t);

  // remaining time
  pvt_mtx_.lock();
  remaining_time_ -= dur;
  if (remaining_time_ < 0.0) { remaining_time_ = 0.0; }
  pvt_mtx_.unlock();

  // next point
  if (t >= pvt_trgt_.time) {
    t_start_ = t_curr;
    pvt_curr_ = pvt_trgt_;

    double rt;
    std::size_t cnt;
    std::unique_lock<std::mutex> lck(pvt_mtx_);
    //pvt_buffer_.erase(pvt_buffer_.begin());
    pvt_buffer_.pop_front();
    cnt = pvt_count_ = pvt_buffer_.size();
    rt = remaining_time_;

    if (cnt > 0) {
      pvt_trgt_ = pvt_buffer_.front();

      lck.unlock();

      // tmrl_INFO_STREAM("--pvt, count: " << cnt << ", remaining_time: " << rt);
    }
    else {
      rt = remaining_time_ = 0.0;

      lck.unlock();

      // check zeros
      if (!check_is_zeros(pvt_curr_.velocities)) {
        tmrl_WARN_STREAM("pvt end point has speed");
      }
      pvt_curr_.velocities = std::vector<double>(pvt_curr_.positions.size(), 0.0);
      p.positions = pvt_curr_.positions;
      p.velocities = pvt_curr_.velocities;

      // tmrl_INFO_STREAM("--pvt, count: " << cnt << ", remaining_time: " << rt);
      tmrl_INFO_STREAM("idle...");
      state = IDLE;
    }
  }

  // update joint state
  const size_t dof = rs_.DOF;
  rs_.set_joint_states(to_arrayd<dof>(p.positions), to_arrayd<dof>(p.velocities), vector6d{0});

  return state;
}

void SimPvtMotion::run()
{
  tmrl_INFO_STREAM("pvt thread begin");

  State state = IDLE;

  size_t cnt_pb = 0;
  size_t cnt_ct = 0;
  auto t0 = std::chrono::steady_clock::now();

  while (keep_alive_)
  {
    // if (!enabled && pvt_count_ == 0) break;
    while (enabled || pvt_count_ != 0)
    {
    state = state_func[state]();

    // update joint state every 50ms
    if (cnt_pb == 0) {
      // ...
    }
    cnt_pb = (cnt_pb + 1) % 50;

    // check and fix timer every 100ms
    bool slp = true;
    if (cnt_ct == 0) {
      // check timer
      auto t1 = std::chrono::steady_clock::now();
      auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
      t0 = t1;
      if (dur_ms > 100) {
        slp = false;
      }
    }
    cnt_ct = (cnt_ct + 1) % 100;

    if (slp) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  tmrl_INFO_STREAM("pvt thread end");
}

}
}
