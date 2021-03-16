#include "tmrl/driver/driver.h"
#include "tmrl/utils/logger.h"

#include <iostream>
#include <condition_variable>

#ifdef _WIN32
#include <conio.h>
#else
#include <string.h>
#include <unistd.h>
#include <termios.h>
#endif

#ifdef _WIN32
void init_termios(int echo) {}
void reset_termios() {}
int kb_hit() { return _kbhit(); }
int get_char() { return _getche(); }
#else
static struct termios oldt, newt;
/* Initialize new terminal i/o settings */
void init_termios(int echo) {
  tcgetattr(STDIN_FILENO, &oldt); /* grab old terminal i/o settings */
  newt = oldt; /* make new settings same as old settings */
  newt.c_lflag &= ~ICANON; /* disable buffered i/o */
  newt.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); /* use these new terminal i/o settings now */
}
/* Restore old terminal i/o settings */
void reset_termios() {
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
int kb_hit() {
  struct timeval tv;
  fd_set rdfs;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&rdfs);
  FD_SET(STDIN_FILENO, &rdfs);
  select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}
int get_char() { return getchar(); }
#endif

void print_packet(const tmrl::comm::Packet &pck, const std::string &name)
{
  std::cout << name << "\n";
  std::cout << "header: " << pck.header_str() << "\ndata  : ";
  std::cout << pck.get_data_str() << "\n";
}

void tui(const std::string &host)
{
  tmrl::driver::TmsvrClient svr{host};
  tmrl::driver::TmsctClient sct{host};
  tmrl::driver::Driver iface{svr, sct};

  int id_cnt = 0;

  char cstr[512];
  char delim[] = " ,;\t";
  char c;
  while (1) {
    memset(cstr, 0, 512);
    fgets(cstr, 512, stdin);
    int n = (int)(strlen(cstr));
    if (n > 0) {
      if (cstr[n - 1] == '\n') { cstr[n - 1] = '\0'; }
    }
    if (strncmp(cstr, "quit", 4) == 0) {
      iface.halt();
      break;
    }
    else if (strncmp(cstr, "start", 5) == 0) {
      iface.start(true);
    }
    else if (strncmp(cstr, "halt", 4) == 0) {
      iface.halt();
    }
    else if (strncmp(cstr, "show", 4) == 0) {
      iface.state.print();
      std::cout << "---\n";
    }
    else if (strncmp(cstr, "loop", 4) == 0) {
      std::mutex mtx;
      std::condition_variable cv;
      bool updated = false;
      char tmp_c = '\0';
      init_termios(1);
      iface.tmsvr.set_feedback_callback(
        [&mtx, &cv, &updated](const tmrl::driver::RobotState &rs)
      {
        std::unique_lock<std::mutex> lck(mtx);
        updated = true;
        lck.unlock();
        cv.notify_one();
      });
      while (true) {
        if (kb_hit()) {
          tmp_c = get_char();
          if (tmp_c == 'q' || tmp_c == 'Q')
            break;
        }
        std::unique_lock<std::mutex> lck(mtx);
        while (!updated) { cv.wait(lck); }
        iface.state.print();
        std::cout << "---\n";
      }
      iface.tmsvr.set_feedback_callback(
        [](const tmrl::driver::RobotState &rs){}
      );
      reset_termios();
    }
    else if (strncmp(cstr, "stop", 4) == 0) {
      iface.tmsct.send_script(std::to_string(id_cnt), "StopAndClearBuffer()");
    }
    else if (strncmp(cstr, "home", 4) == 0) {
      std::string script = "PTP(\"JPP\",0,0,0,0,0,0,10,200,0,false)";
      iface.tmsct.send_script(std::to_string(id_cnt), script);
    }
    else if (strncmp(cstr, "ready", 4) == 0) {
      tmrl::vector6d ang{0.0, 0.0, 1.5707, 0.0, 1.5707, 0.0};
      iface.set_joint_pos_PTP(ang, 10, 0.2, 0);
    }
    else if (strncmp(cstr, "pvt_test", 8) == 0) {
      tmrl::driver::PvtPoint point;
      tmrl::driver::PvtTraj traj;
      point.time = 5.0;
      point.positions.assign(6, 0.0);
      point.velocities.assign(6, 0.0);
      traj.mode = tmrl::driver::PvtMode::Joint;
      traj.points.push_back(point);
      traj.total_time = point.time;

      std::thread(std::bind(&tmrl::driver::Driver::run_pvt_traj, &iface, traj)).detach();

      double t = 0.0;
      char tmp_c = '\0';
      init_termios(1);
      while (t < traj.total_time) {
        if (kb_hit()) {
          tmp_c = get_char();
          if (tmp_c == 'q' || tmp_c == 'Q') {
            iface.stop_pvt_traj();
            break;
          }
        }
        printf(".\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        t += 0.1;
      }
      reset_termios();
    }
    else {
      std::string cmd{ cstr };
      std::cout << "send cmd: " << cmd << "\n";

      iface.tmsct.send_script(std::to_string(id_cnt), cmd);
    }

    ++id_cnt;
    if (id_cnt > 9) { id_cnt = 9; }
  }

}

/*int main(int argc, char **argv)
{
  std::string host;
  if (argc > 1) {
    host = argv[1];
  }
  else {
    tmrl_ERROR_STREAM("no ip-address");
    return 1;
  }
  tui(host);
  return 0;
}*/
