#include "tmrl/driver/script_commands.h"
#include "tmrl/driver/robot_state.h"

#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>

namespace tmrl
{
namespace driver
{

class SimPvtMotion
{
public:
  SimPvtMotion(RobotState &state);
  ~SimPvtMotion() { stop(); }

  void stop();

  void enter(const vectorXd &init_joint_states);
  void exit();

  void add_point(const PvtPoint &p);

private:
  void interp(PvtPoint &p, const PvtPoint &p0, const PvtPoint &p1, double t)
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

  enum State {
    IDLE,
    MOVING,
    COUNT
  };
  State idle();
  State moving();

  void run();

private:
  RobotState &rs_;

  bool keep_alive_ = false;

  bool enabled = false;
  //std::mutex enable_mtx_;

  //vectorXd joint_angle_;
  double remaining_time_ = 0.0;
  std::size_t pvt_count_ = 0;
  std::deque<PvtPoint> pvt_buffer_;
  std::mutex pvt_mtx_;

  PvtPoint pvt_curr_;
  PvtPoint pvt_trgt_;

  std::chrono::steady_clock::time_point t_start_;
  std::chrono::steady_clock::time_point t_last_;

  std::function<State()> state_func[COUNT] = {
    std::bind(&SimPvtMotion::idle, this),
    std::bind(&SimPvtMotion::moving, this)
  };

  std::thread pvt_thread_;
};

}
}
