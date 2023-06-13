#include "tmrl/driver/robot_state.h"
#include "tmrl/utils/conversions.h"
#include "tmrl/utils/logger.h"

//#include <memory>
#include <cstring>

#include <iostream>

namespace tmrl
{
namespace driver
{

RobotState::RobotState()
  :_data_table(new DataTable(this))
{
  tmrl_DEBUG_STREAM("tmrl::driver::RobotState::RobotState");

  _f_deserialize_item[0] = std::bind(&RobotState::_deserialize_skip,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  _f_deserialize_item[1] = std::bind(&RobotState::_deserialize_copy_wo_check,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  _f_deserialize = std::bind(&RobotState::_deserialize_first_time, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}
RobotState::~RobotState()
{
  tmrl_DEBUG_STREAM("tmrl::driver::RobotState::~RobotState");

  delete _data_table;
}

size_t RobotState::_deserialize_name(std::string &name, const char *data, size_t offset)
{
  size_t boffset = offset;
  unsigned short uslen; // 2 bytes

  // item name length
  memcpy(&uslen, data + boffset, 2);
  boffset += 2;
  // item name
  name = std::string{data + boffset, uslen};
  boffset += uslen;
  // skip item
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;
  return boffset;
}
size_t RobotState::_deserialize_skip(void *dst, const char *data, size_t offset)
{
  size_t boffset = offset;
  unsigned short uslen; // 2 bytes

  // skip item name
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;
  // skip item
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;

  if (dst) {}
  return boffset;
}
size_t RobotState::_deserialize_copy_wo_check(void *dst, const char *data, size_t offset)
{
  size_t boffset = offset;
  //size_t bsize = 2;
  unsigned short uslen; // 2 bytes

  // skip item name
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;
  // item data length
  memcpy(&uslen, data + boffset, 2);
  boffset += 2;
  // item data
  //bsize = uslen;
  memcpy(dst, data + boffset, uslen);
  boffset += uslen;
  return boffset;
}
size_t RobotState::_deserialize_first_time(const char *data, size_t size, bool lock)
{
  size_t boffset = 0;
  size_t count = 0;
  size_t check_count = 0;
  size_t skip_count = 0;
  unsigned short uslen = 0; // 2 bytes
  std::string item_name;

  tmrl_INFO_STREAM("TM Flow DataTable Checked Item: ");
  _item_updates.clear();

  while (boffset < size && count < 100) {
    // item name length
    memcpy(&uslen, data + boffset, 2);
    boffset += 2;
    // item name
    item_name = std::string{data + boffset, uslen};
    boffset += uslen;

    ItemUpdate update{ nullptr, ItemUpdate::SKIP };

    auto iter = _data_table->find(item_name);
    if (iter != _data_table->end()) {
      update.dst = iter->second.dst;
      update.func = ItemUpdate::UPDATE;

      iter->second.checked = true;
      tmrl_INFO_STREAM("- " << item_name << " - checked");
      ++check_count;
    }
    else {
      tmrl_INFO_STREAM("- " << item_name << " - skipped");
      ++skip_count;
    }
    _item_updates.push_back({ update.dst, update.func });

    // item data length
    memcpy(&uslen, data + boffset, 2);
    boffset += 2;
    if (update.func == ItemUpdate::SKIP) {
      // skip item
      boffset += uslen;
    }
    else {
      // item data
      memcpy(update.dst, data + boffset, uslen);
      boffset += uslen;
    }
    ++count;
  }
  tmrl_INFO_STREAM("Total " << _item_updates.size() << " item," <<
    check_count << " checked, " << skip_count << " skipped");

  _deserialize_update(lock);

  for (auto iter : _data_table->get()) {
    if (iter.second.required && !iter.second.checked) {
      tmrl_ERROR_STREAM("Required item " << iter.first << " is NOT checked");
    }
  }

  _f_deserialize = std::bind(&RobotState::_deserialize, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  return boffset;
}
size_t RobotState::_deserialize(const char *data, size_t size, bool lock)
{
  size_t boffset = 0;

  for (auto &update : _item_updates) {
    boffset = _f_deserialize_item[update.func](update.dst, data, boffset);
  }

  _deserialize_update(lock);

  if (boffset > size) {
  }
  return boffset;
}
inline double _meter(float mm) { return 0.001 * (double)(mm); }
inline vector6d _rads(float *ang)
{
  vector6d rv;
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = utils::rad((double)(ang[i])); }
  return rv;
}
inline vector6d _meters(float *mm)
{
  vector6d rv;
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = _meter(mm[i]); }
  return rv;
}
inline PoseEular _si_pose(float *pose)
{
  PoseEular rp;
  for (size_t i = 0; i < 3; ++i) { rp[i] = _meter(pose[i]); }
  for (size_t i = 3; i < 6; ++i) { rp[i] = utils::rad((double)(pose[i])); }
  return rp;
}
void RobotState::_deserialize_update(bool lock)
{
  // ---------------
  // update together
  // ---------------
  {
    std::unique_lock<std::mutex> lck(mtx, std::defer_lock);
    if (lock) { lck.lock(); }

    //const char c0 = 0;

    _is_linked = _is_linked_;
    _has_error = _has_error_;
    _is_proj_running = _is_proj_running_;
    _is_proj_paused = _is_proj_paused_;

    _is_safeguard_A_triggered = _is_safeguard_A_triggered_;
    _is_ESTOP_pressed = _is_ESTOP_pressed_;
    _camera_light = _camera_light_;
    _error_code = _error_code_;

    _proj_speed = _proj_speed_;
    _ma_mode = _ma_mode_;
    //_stick_play_pause = _stick_play_pause_;

    _robot_light = _robot_light_;

    _joint_angle = _rads(_joint_angle_);

    _flange_pose = _si_pose(_flange_pose_);

    _tool_pose = _si_pose(_tool_pose_);

    for (size_t i = 0; i < 3; ++i) { _tcp_force_vec[i] = (double)(_tcp_force_vec_[i]); }

    _tcp_force = (double)(_tcp_force_);

    _tcp_speed_vec = _si_pose(_tcp_speed_vec_);

    _tcp_speed = _meter(_tcp_speed_);

    _joint_speed = _rads(_joint_speed_);

    _joint_torque = _meters(_joint_torque_);

    // IO

    for (size_t i = 0; i < 16; ++i) { _ctrller_DO[i] = _ctrller_DO_[i]; }
    for (size_t i = 0; i < 16; ++i) { _ctrller_DI[i] = _ctrller_DI_[i]; }
    for (size_t i = 0; i <  2; ++i) { _ctrller_AO[i] = _ctrller_AO_[i]; }
    for (size_t i = 0; i <  2; ++i) { _ctrller_AI[i] = _ctrller_AI_[i]; }
    for (size_t i = 0; i <  4; ++i) { _ee_DO[i] = _ee_DO_[i]; }
    for (size_t i = 0; i <  4; ++i) { _ee_DI[i] = _ee_DI_[i]; }
    for (size_t i = 0; i <  2; ++i) { _ee_AO[i] = _ee_AO_[i]; }
    for (size_t i = 0; i <  2; ++i) { _ee_AI[i] = _ee_AI_[i]; }
  }
}

template<std::size_t N>
std::string _arrayuc_to_string(const std::array<unsigned char, N> &vec)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    ss << (int)(vec[i]);
    if (i != vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}
void RobotState::print() const
{
  std::cout << "Robot_Link=" << (int)_is_linked << "\n";
  std::cout << "Robot_Error=" << (int)_has_error << "\n";
  std::cout << "Project_Run=" << (int)_is_proj_running << "\n";
  std::cout << "Project_Pause=" << (int)_is_proj_paused << "\n";
  std::cout << "Safetyguard_A=" << (int)_is_safeguard_A_triggered << "\n";
  std::cout << "ESTOP=" << (int)_is_ESTOP_pressed << "\n";
  std::cout << "Camera_Light=" << (int)_camera_light << "\n\n";

  std::cout << "Error_Code=" << _error_code << "\n";
  std::cout << "Error_Content=" << _error_content << "\n\n";

  std::cout << "Joint_Angle=" << to_string(_joint_angle) << "\n";
  std::cout << "Coord_Robot_Tool0=" << to_string(_flange_pose) << "\n";
  std::cout << "Coord_Robot_Tool=" << to_string(_tool_pose) << "\n\n";

  std::cout << "Project_Speed=" << _proj_speed << "\n";
  std::cout << "MA_Mode=" << _ma_mode << "\n";
  std::cout << "Robot_Light=" << _robot_light << "\n\n";

  std::cout << "End_DO=" << _arrayuc_to_string(_ee_DO) << "\n";
  std::cout << "End_DI=" << _arrayuc_to_string(_ee_DI) << "\n";
}

}
}
