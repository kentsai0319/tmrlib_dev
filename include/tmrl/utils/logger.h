#pragma once

#include <functional>
#include <string>
#include <sstream>

namespace tmrl
{
namespace utils
{

class logger
{
public:
  enum level
  {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
    NOTHING
  };
  typedef std::function<void(const std::string &)> print_str_f;
private:
  level _level;
  print_str_f _print_str_f[NOTHING];

  logger();
public:
  static logger & get();
  void set_level(level lv) { _level = lv; }
  void setup_debug(print_str_f fd) { _print_str_f[DEBUG] = fd; }
  void setup_info (print_str_f fi) { _print_str_f[ INFO] = fi; }
  void setup_warn (print_str_f fw) { _print_str_f[ WARN] = fw; }
  void setup_error(print_str_f fe) { _print_str_f[ERROR] = fe; }
  void setup_fatal(print_str_f ff) { _print_str_f[FATAL] = ff; }

  void debug(const std::string &msg) { if (_level == DEBUG) _print_str_f[DEBUG](msg); }
  void  info(const std::string &msg) { if (_level <=  INFO) _print_str_f[ INFO](msg); }
  void  warn(const std::string &msg) { if (_level <=  WARN) _print_str_f[ WARN](msg); }
  void error(const std::string &msg) { if (_level <= ERROR) _print_str_f[ERROR](msg); }
  void fatal(const std::string &msg) { if (_level <= FATAL) _print_str_f[FATAL](msg); }
};

inline logger & get_logger() { return logger::get(); }

}
}

#define tmrl_DEBUG_STREAM(msg) do { \
  std::stringstream _ss_; _ss_ << msg; \
  tmrl::utils::get_logger().debug(_ss_.str()); \
} while (false)

#define tmrl_INFO_STREAM(msg)  do { \
  std::stringstream _ss_; _ss_ << msg; \
  tmrl::utils::get_logger().info(_ss_.str()); \
} while (false)

#define tmrl_WARN_STREAM(msg)  do { \
  std::stringstream _ss_; _ss_ << msg; \
  tmrl::utils::get_logger().warn(_ss_.str()); \
} while (false)

#define tmrl_ERROR_STREAM(msg) do { \
  std::stringstream _ss_; _ss_ << msg; \
  tmrl::utils::get_logger().error(_ss_.str()); \
} while (false)

#define tmrl_FATAL_STREAM(msg) do { \
  std::stringstream _ss_; _ss_ << msg; \
  tmrl::utils::get_logger().fatal(_ss_.str()); \
} while (false)
