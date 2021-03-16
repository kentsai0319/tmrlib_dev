#pragma once

#include "tmrl/comm/packet.h"

#include <thread>
#include <condition_variable>
#include <functional>

namespace tmrl
{
namespace comm
{

enum class RetCode
{
  ERR = -1,
  OK = 0,
  TIMEOUT,
  NOTREADY,
  NOTCONNECT,
  NOTSENDALL,
  INVALIDPACK,
  NOVALIDPACK,
};

class RecvBuf;

class Client
{
public:
  explicit Client(const std::string &ip, unsigned short port, size_t buffer_size);
  /*virtual*/ ~Client();

  bool Connect(int timeout_ms = 0);
  void Close();

  const std::string & ip_address() const { return _ip; }

  bool is_connected() const { return (_sockfd > 0); }

  int socket_fd() const { return _sockfd; }
  int socket_fd(int sockfd) { _sockfd = sockfd; return _sockfd; }

  RetCode send_bytes(const char *bytes, int len, int *n = NULL);
  RetCode send_bytes_all(const char *bytes, int len, int *n = NULL);

  RetCode send_packet(Packet &packet, bool info = false);
  RetCode send_packet_all(Packet &packet, bool info = false);
  RetCode send_packet_(Packet &packet, bool info = false);
  
  static const bool LOG_INFO = true;
  static const bool LOG_NOTHING = false;

  /*
   * Call init_receiver() before call receiver_spin_once(...)
   */
  bool init_receiver();

  RetCode receiver_spin_once(int timeval_ms, int *n = NULL);

  std::vector<Packet> &packet_vector() { return _packet_vec; }

  Packet & last_packet() { return _packet_vec.back(); }

private:
  int _connect(int sockfd, const char *ip, unsigned short port, int timeout_ms);

  RecvBuf       *_recv;
  std::string    _ip;
  unsigned short _port;
  int            _sockfd;
  int            _optflag;
  RetCode        _recv_rc;
  bool           _recv_ready;

  std::vector<Packet> _packet_vec;
};

class ClientThread
{
public:
  using IsOkPredicate = std::function<bool()>;

  explicit ClientThread(const std::string &ip, unsigned short port, size_t buffer_size);
  virtual ~ClientThread();

  const Client & client() const { return _client; }

  bool start(int timeout_ms);
  bool start();
  void stop();

  void set_is_ok_predicate(IsOkPredicate is_ok_pred) { _isOk = is_ok_pred; }

  void set_reconnect_timeval(double sec) { _reconnect_timeval_ms = (int)(1000.0 * sec); }
  void set_reconnect_timeout(double sec) { _reconnect_timeout_ms = (int)(1000.0 * sec); }

protected:
  virtual bool receive(const std::vector<Packet> &pack_vec) = 0;

  void run();
  void reconnect();
  bool isOk() { return (_keep_alive && _isOk()); }

  Client _client;
  std::string _hdr;
  std::thread _thd;
  bool _keep_alive = false;
  IsOkPredicate _isOk;
  int _reconnect_timeval_ms = 3000;
  int _reconnect_timeout_ms = 1000;
};

}
}