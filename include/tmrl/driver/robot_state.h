#pragma once

#include "tmrl/types.h"

#include <mutex>
#include <functional>
#include <map>
//#include <unordered_map>

namespace tmrl
{
namespace driver
{

class DataTable;

class RobotState
{
  friend class DataTable;

public:
  explicit RobotState();
  ~RobotState();

  enum { DOF = 6 };

  using Ulock = std::unique_lock<std::mutex>;

  mutable std::mutex mtx;

private:
  DataTable *_data_table;

  // robot state

  unsigned char _is_linked {0};
  unsigned char _has_error {0};
  unsigned char _is_proj_running {0};
  unsigned char _is_proj_paused {0};
  unsigned char _is_safeguard_A_triggered {0};
  unsigned char _is_ESTOP_pressed {0};
  unsigned char _camera_light {0};

  int _error_code {0};
  std::string _error_content;

  vector6d _joint_angle {0};	
  PoseEular _flange_pose {0};
  PoseEular _tool_pose {0};

  vector3d _tcp_force_vec {0};
  double _tcp_force {0};
  vector6d _tcp_speed_vec {0};
  double _tcp_speed {0};
  vector6d _joint_speed {0};
  vector6d _joint_torque {0};

  PoseEular _tcp_frame {0};
  double _tcp_mass {0};
  PoseEular _tcp_cog {0};

  int _proj_speed {0};
  int _ma_mode {0};

  unsigned char _stick_play_pause {0};

  int _robot_light {0};

  std::array<unsigned char, 16> _ctrller_DO {0};
  std::array<unsigned char, 16> _ctrller_DI {0};
  std::array<float, 2> _ctrller_AO {0};
  std::array<float, 2> _ctrller_AI {0};
  std::array<unsigned char, 4> _ee_DO {0};
  std::array<unsigned char, 4> _ee_DI {0};
  std::array<float, 2> _ee_AO {0};
  std::array<float, 2> _ee_AI {0};

  // parsing tmp.

  unsigned char _is_linked_ {0};
  unsigned char _has_error_ {0};
  unsigned char _is_proj_running_ {0};
  unsigned char _is_proj_paused_ {0};
  unsigned char _is_safeguard_A_triggered_ {0};
  unsigned char _is_ESTOP_pressed_ {0};
  unsigned char _camera_light_ {0};

  int _error_code_ {0};

  float _joint_angle_[DOF] {0};
  float _flange_pose_[6] {0};
  float _tool_pose_[6] {0};
  float _tcp_frame_[6] {0};
  float _tcp_mass_ {0};
  float _tcp_cog_[6] {0};

  float _tcp_force_vec_[3] {0};
  float _tcp_force_ {0};
  float _tcp_speed_vec_[6] {0};
  float _tcp_speed_ {0};
  float _joint_speed_[DOF] {0};
  float _joint_torque_[DOF] {0};

  int _proj_speed_ {0};
  int _ma_mode_ {0};

  unsigned char _stick_play_pause_ {0};

  int _robot_light_ {0};

  unsigned char _ctrller_DO_[16] {0};
  unsigned char _ctrller_DI_[16] {0};
  float _ctrller_AO_[2] {0};
  float _ctrller_AI_[2] {0};
  unsigned char _ee_DO_[4] {0};
  unsigned char _ee_DI_[4] {0};
  float _ee_AO_[2] {0};
  float _ee_AI_[2] {0};

  // deserializer

  std::function<size_t (void *, const char *, size_t)> _f_deserialize_item[2];
  std::function<size_t (const char *, size_t, bool)> _f_deserialize;
  struct ItemUpdate {
    void *dst;
    size_t func;
    enum { SKIP, UPDATE };
  };
  std::vector<ItemUpdate> _item_updates;

  static size_t _deserialize_name(std::string &name, const char *data, size_t offset);
  static size_t _deserialize_skip(void *dst, const char *data, size_t offset);
  static size_t _deserialize_copy_wo_check(void *dst, const char *data, size_t offset);

  size_t _deserialize_first_time(const char *data, size_t size, bool lock);
  size_t _deserialize(const char *data, size_t size, bool lock);
  void _deserialize_update(bool lock);

public:
  size_t deserialize(const char *data, size_t size)
  {
    return _f_deserialize(data, size, false);
  }
  size_t deserialize_with_lock(const char *data, size_t size)
  {
    return _f_deserialize(data, size, true);
  }

  unsigned char is_linked() const { return _is_linked; }
  unsigned char has_error() const { return _has_error; }

  unsigned char is_project_running() const { return _is_proj_running; }
  unsigned char is_project_paused() const { return _is_proj_paused; }

  unsigned char is_safeguard_A() const { return _is_safeguard_A_triggered; }
  unsigned char is_EStop() const { return _is_ESTOP_pressed; }

  unsigned char camera_light() const { return _camera_light; } // R/W

  int error_code() const { return _error_code; }
  std::string error_content() const { return _error_content; }

  vector6d joint_angle() const { return _joint_angle; }
  PoseEular flange_pose() const { return _flange_pose; }
  PoseEular tool_pose() const { return _tool_pose; }

  vector3d tcp_force_vec() const { return _tcp_force_vec; }
  double tcp_force() const { return _tcp_force; }
  vector6d tcp_speed_vec() const { return _tcp_speed_vec; }
  double tcp_speed() const{ return _tcp_speed; }
  vector6d joint_speed() const { return _joint_speed; }
  vector6d joint_torque() const { return _joint_torque; }

  int project_speed() const { return _proj_speed; }
  int ma_mode() const { return _ma_mode; }

  unsigned char stick_play_pause() const { return _stick_play_pause; } // R/W

  int robot_light() const { return _robot_light; }

  std::array<unsigned char, 16> ctrller_DO() const { return _ctrller_DO; }
  std::array<unsigned char, 16> ctrller_DI() const { return _ctrller_DI; }
  std::array<float, 2> ctrller_AO() const { return _ctrller_AO; }
  std::array<float, 2> ctrller_AI() const { return _ctrller_AI; }

  std::array<unsigned char, 4> ee_DO() const { return _ee_DO; }
  std::array<unsigned char, 4> ee_DI() const { return _ee_DI; }
  //std::array<float> ee_AO() const { return _ee_AO; }
  std::array<float, 2> ee_AI() const { return _ee_AI; }

  void set_joint_states(
    const vector6d &pos, const vector6d &vel, const vector6d &tor)
  {
    _joint_angle = pos;
    _joint_speed = vel;
    _joint_torque = tor;
  }

  void print() const;
};

class DataTable
{
public:
  struct Item {
    void *dst;
    bool required;
    bool checked;
    enum { REQUIRED = 1 };
    Item() : dst(nullptr), required(false), checked(false) {};
    Item(void *d) : dst(d), required(false), checked(false) {};
    Item(void *d, bool r) : dst(d), required(r), checked(false) {};
  };
  DataTable(RobotState *rs)
  {
    _item_map.clear();
    //_item_map[""] = { &rs-, };
    _item_map["Robot_Link"         ] = { &rs->_is_linked_, };
    _item_map["Robot_Error"        ] = { &rs->_has_error_, Item::REQUIRED };
    _item_map["Project_Run"        ] = { &rs->_is_proj_running_ };
    _item_map["Project_Pause"      ] = { &rs->_is_proj_paused_ };
    _item_map["Safeguard_A"        ] = { &rs->_is_safeguard_A_triggered_};
    _item_map["ESTOP"              ] = { &rs->_is_ESTOP_pressed_ };
    _item_map["Camera_Light"       ] = { &rs->_camera_light_ };
    _item_map["Error_Code"         ] = { &rs->_error_code_ };
    _item_map["Joint_Angle"        ] = { &rs->_joint_angle_, Item::REQUIRED };
    _item_map["Coord_Robot_Flange" ] = { &rs->_flange_pose_ };
    _item_map["Coord_Robot_Tool"   ] = { &rs->_tool_pose_, Item::REQUIRED };
    _item_map["TCP_Force"          ] = { &rs->_tcp_force_vec_ };
    _item_map["TCP_Force3D"        ] = { &rs->_tcp_force_ };
    _item_map["TCP_Speed"          ] = { &rs->_tcp_speed_vec_ };
    _item_map["TCP_Speed3D"        ] = { &rs->_tcp_speed_ };
    _item_map["Joint_Speed"        ] = { &rs->_joint_speed_ };
    _item_map["Joint_Torque"       ] = { &rs->_joint_torque_ };
    _item_map["Project_Speed"      ] = { &rs->_proj_speed_ };
    _item_map["MA_Mode"            ] = { &rs->_ma_mode_ };
    _item_map["Robot_Light"        ] = { &rs->_robot_light_ };
    _item_map["Ctrl_DO0"           ] = { &rs->_ctrller_DO_[ 0] };
    _item_map["Ctrl_DO1"           ] = { &rs->_ctrller_DO_[ 1] };
    _item_map["Ctrl_DO2"           ] = { &rs->_ctrller_DO_[ 2] };
    _item_map["Ctrl_DO3"           ] = { &rs->_ctrller_DO_[ 3] };
    _item_map["Ctrl_DO4"           ] = { &rs->_ctrller_DO_[ 4] };
    _item_map["Ctrl_DO5"           ] = { &rs->_ctrller_DO_[ 5] };
    _item_map["Ctrl_DO6"           ] = { &rs->_ctrller_DO_[ 6] };
    _item_map["Ctrl_DO7"           ] = { &rs->_ctrller_DO_[ 7] };
    _item_map["Ctrl_DO8"           ] = { &rs->_ctrller_DO_[ 8] };
    _item_map["Ctrl_DO9"           ] = { &rs->_ctrller_DO_[ 9] };
    _item_map["Ctrl_DO10"          ] = { &rs->_ctrller_DO_[10] };
    _item_map["Ctrl_DO11"          ] = { &rs->_ctrller_DO_[11] };
    _item_map["Ctrl_DO12"          ] = { &rs->_ctrller_DO_[12] };
    _item_map["Ctrl_DO13"          ] = { &rs->_ctrller_DO_[13] };
    _item_map["Ctrl_DO14"          ] = { &rs->_ctrller_DO_[14] };
    _item_map["Ctrl_DO15"          ] = { &rs->_ctrller_DO_[15] };
    _item_map["Ctrl_DI0"           ] = { &rs->_ctrller_DI_[ 0] };
    _item_map["Ctrl_DI1"           ] = { &rs->_ctrller_DI_[ 1] };
    _item_map["Ctrl_DI2"           ] = { &rs->_ctrller_DI_[ 2] };
    _item_map["Ctrl_DI3"           ] = { &rs->_ctrller_DI_[ 3] };
    _item_map["Ctrl_DI4"           ] = { &rs->_ctrller_DI_[ 4] };
    _item_map["Ctrl_DI5"           ] = { &rs->_ctrller_DI_[ 5] };
    _item_map["Ctrl_DI6"           ] = { &rs->_ctrller_DI_[ 6] };
    _item_map["Ctrl_DI7"           ] = { &rs->_ctrller_DI_[ 7] };
    _item_map["Ctrl_DI8"           ] = { &rs->_ctrller_DI_[ 8] };
    _item_map["Ctrl_DI9"           ] = { &rs->_ctrller_DI_[ 9] };
    _item_map["Ctrl_DI10"          ] = { &rs->_ctrller_DI_[10] };
    _item_map["Ctrl_DI11"          ] = { &rs->_ctrller_DI_[11] };
    _item_map["Ctrl_DI12"          ] = { &rs->_ctrller_DI_[12] };
    _item_map["Ctrl_DI13"          ] = { &rs->_ctrller_DI_[13] };
    _item_map["Ctrl_DI14"          ] = { &rs->_ctrller_DI_[14] };
    _item_map["Ctrl_DI15"          ] = { &rs->_ctrller_DI_[15] };
    _item_map["Ctrl_AO0"           ] = { &rs->_ctrller_AO_[ 0] };
    _item_map["Ctrl_AO1"           ] = { &rs->_ctrller_AO_[ 1] };
    _item_map["Ctrl_AI0"           ] = { &rs->_ctrller_AI_[ 0] };
    _item_map["Ctrl_AI1"           ] = { &rs->_ctrller_AI_[ 1] };
    _item_map["End_DO0"            ] = { &rs->_ee_DO_[0] };
    _item_map["End_DO1"            ] = { &rs->_ee_DO_[1] };
    _item_map["End_DO2"            ] = { &rs->_ee_DO_[2] };
    _item_map["End_DO3"            ] = { &rs->_ee_DO_[3] };
    _item_map["End_DI0"            ] = { &rs->_ee_DI_[0] };
    _item_map["End_DI1"            ] = { &rs->_ee_DI_[1] };
    _item_map["End_DI2"            ] = { &rs->_ee_DI_[2] };
    _item_map["End_DI3"            ] = { &rs->_ee_DI_[3] };
    _item_map["End_AO0"            ] = { &rs->_ee_AO_[0] };
    _item_map["End_AO1"            ] = { &rs->_ee_AO_[1] };
    _item_map["End_AI0"            ] = { &rs->_ee_AI_[0] };
    _item_map["End_AI1"            ] = { &rs->_ee_AI_[1] };
  }
  std::map<std::string, Item> & get() { return _item_map; }
  std::map<std::string, Item>::iterator find(const std::string &name) { return _item_map.find(name); }
  std::map<std::string, Item>::iterator end() { return _item_map.end(); }
  //std::unordered_map<std::string, Item> & get() { return _item_map; }
  //std::unordered_map<std::string, Item>::iterator find(const std::string &name) { return _item_map.find(name); }
  //std::unordered_map<std::string, Item>::iterator end() { return _item_map.end(); }
private:
  std::map<std::string, Item> _item_map;
  //std::unordered_map<std::string, Item> _item_map;
};

}
}