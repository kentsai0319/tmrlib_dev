#pragma once

#include <string>
#include <sstream>
#include <functional>

namespace tmr
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
		COUNT
	};
	typedef std::function<void(const std::string &)> print_str;
private:
	level level_;
	print_str f_print_str_[COUNT];

	logger();
public:
	static logger & ref();

	void setup_level(level lv) { level_ = lv; }
	void setup_debug(print_str fd) { f_print_str_[DEBUG] = fd; }
	void setup_info (print_str fi) { f_print_str_[ INFO] = fi; }
	void setup_warn (print_str fw) { f_print_str_[ WARN] = fw; }
	void setup_error(print_str fe) { f_print_str_[ERROR] = fe; }
	void setup_fatal(print_str ff) { f_print_str_[FATAL] = ff; }

	void debug(const std::string &msg) { if (level_ == DEBUG) f_print_str_[DEBUG](msg); }
	void  info(const std::string &msg) { if (level_ <=  INFO) f_print_str_[ INFO](msg); }
	void  warn(const std::string &msg) { if (level_ <=  WARN) f_print_str_[ WARN](msg); }
	void error(const std::string &msg) { if (level_ <= ERROR) f_print_str_[ERROR](msg); }
	void fatal(const std::string &msg) { if (level_ <= FATAL) f_print_str_[FATAL](msg); }
};
}

#define tmr_DEBUG_STREAM(msg) do { \
	std::stringstream _ss_; _ss_ << msg; \
	tmr::logger::ref().debug(_ss_.str()); \
} while (false)

#define tmr_INFO_STREAM(msg)  do { \
	std::stringstream _ss_; _ss_ << msg; \
	tmr::logger::ref().info(_ss_.str()); \
} while (false)

#define tmr_WARN_STREAM(msg)  do { \
	std::stringstream _ss_; _ss_ << msg; \
	tmr::logger::ref().warn(_ss_.str()); \
} while (false)

#define tmr_ERROR_STREAM(msg) do { \
	std::stringstream _ss_; _ss_ << msg; \
	tmr::logger::ref().error(_ss_.str()); \
} while (false)

#define tmr_FATAL_STREAM(msg) do { \
	std::stringstream _ss_; _ss_ << msg; \
	tmr::logger::ref().fatal(_ss_.str()); \
} while (false)
