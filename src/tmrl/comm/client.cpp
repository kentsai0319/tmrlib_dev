#include "tmrl/comm/client.h"
#include "tmrl/comm/sbuffer.h"
#include "tmrl/utils/logger.h"

//
// socket
//

#ifdef _WIN32
// windows socket
#pragma comment (lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

namespace tmrl
{
namespace comm
{

class RecvBuf
{
  // rc: return code
  // rn: return num of bytes
  // nb: num of bytes
  // tv: timeval
  // len: length
  // buf: buffer
  // recv: reciever
  // sockfd: socket file description

public:
  explicit RecvBuf(int recv_buf_len);
  ~RecvBuf();

  bool init(int sockfd);
  RetCode spin_once(int timeval_ms, int *n = NULL);
  void commit_spin_once();
  SBuffer & buffer() { return _sbuf; }

private:

  SBuffer _sbuf;

  char *_recv_buf = NULL;
  int   _recv_buf_len = 0;

  int    _sockfd = -1;
  fd_set _masterfs;
  fd_set _readfs;

  int     _rn = 0;
  RetCode _rc = RetCode::OK;

};

// RecvBuf

RecvBuf::RecvBuf(int recv_buf_len)
{
  tmrl_DEBUG_STREAM("tmrl::comm::RecvBuf::RecvBuf");

  if (recv_buf_len < 0x200) recv_buf_len = 0x200;
  //else if (recv_buf_len > 0x10000) recv_buf_len = 0x10000;

  _recv_buf = new char[recv_buf_len];
  _recv_buf_len = recv_buf_len;

  memset(_recv_buf, 0, _recv_buf_len);
}
RecvBuf::~RecvBuf()
{
  tmrl_DEBUG_STREAM("tmrl::comm::RecvBuf::~RecvBuf");

  delete _recv_buf;
}
bool RecvBuf::init(int sockfd)
{
  if (sockfd <= 0) return false;

  _sbuf.clear();
  _sockfd = sockfd;

  FD_ZERO(&_masterfs);

  FD_SET(sockfd, &_masterfs);

  _rc = RetCode::OK;
  return true;
}
RetCode RecvBuf::spin_once(int timeval_ms, int *n)
{
  RetCode rc = RetCode::OK;
  int nb = 0;
  int rv = 0;
  //int sp = 0;
  timeval tv;

  if (timeval_ms < 8) timeval_ms = 8;

  tv.tv_sec = (timeval_ms / 1000);
  tv.tv_usec = (timeval_ms % 1000) * 1000;

  _readfs = _masterfs; // re-init

  rv = select(_sockfd + 1, &_readfs, NULL, NULL, &tv);

  if (n) *n = 0;

  if (rv < 0) {
    rc = RetCode::ERR;
  }
  else if (rv == 0) {
    rc = RetCode::TIMEOUT;
  }
  else if (FD_ISSET(_sockfd, &_readfs)) {
    nb = recv(_sockfd, _recv_buf, _recv_buf_len, 0);

    if (nb < 0) {
      // error
      rc = RetCode::ERR;
    }
    else if (nb == 0) {
      // sever is closed
      rc = RetCode::NOTCONNECT;
    }
    else {
      // recv n bytes
      _sbuf.append(_recv_buf, nb);

      if (n) *n = nb;
    }
  }
  else {
    rc = RetCode::NOTREADY;
  }
  _rn = nb;
  _rc = rc;
  return rc;
}
void RecvBuf::commit_spin_once()
{
  _sbuf.pop_front(_rn);
}

// Client

Client::Client(const std::string &ip, unsigned short port, size_t buffer_size)
  : _recv(new RecvBuf{(int)(buffer_size)})
  , _ip(ip)
  , _port(port)
  , _sockfd(-1)
  , _optflag(1)
  , _recv_rc(RetCode::OK)
  , _recv_ready(false)
{
  tmrl_DEBUG_STREAM("tmrl::comm::Client::Client");

#ifdef _WIN32
  // Initialize Winsock
  WSADATA wsaData;
  int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != 0) {
    //
  }
#endif
}
Client::~Client()
{
  tmrl_DEBUG_STREAM("tmrl::comm::Client::~Client");
  delete _recv;
}
int Client::_connect(int sockfd, const char *ip, unsigned short port, int timeout_ms)
{
  int rv = 0;
  int flags = 0;
  int err = 0;
  int err_len = 0;
  sockaddr_in addr;
  timeval tv;
  fd_set wset;

  tmrl_INFO_STREAM("TM_COM: ip:=" << ip);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, ip, &(addr.sin_addr));

  tv.tv_sec = (timeout_ms / 1000);
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  FD_ZERO(&wset);
  FD_SET(sockfd, &wset);

#ifndef _WIN32
  //Get Flag of Fcntl
  if ((flags = fcntl(sockfd, F_GETFL, 0)) < 0 ) {
    tmrl_WARN_STREAM("TM_COM: The flag of fcntl is not ok");
    return -1;
  }
#endif

  rv = connect(sockfd, (sockaddr *)&addr, 16);
  tmrl_INFO_STREAM("TM_COM: rv:=" << rv);

  if (rv < 0) {
    if (errno != EINPROGRESS) return -1;
  }
  if (rv == 0) {
    tmrl_INFO_STREAM("TM_COM: Connection is ok");
    return rv;
  }
  else {
    //Wait for Connect OK by checking Write buffer
    if ((rv = select(sockfd + 1, NULL, &wset, NULL, &tv)) < 0) {
      return rv;
    }
    if (rv == 0) {
      tmrl_WARN_STREAM("TM_COM: Connection timeout");
      //errno = ETIMEDOUT;
      return -1;
    }
    if (FD_ISSET(sockfd, &wset)) {
#ifdef _WIN32
      if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &err_len) < 0) {
#else
      if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &err, (socklen_t *)&err_len) < 0) {
#endif
        tmrl_ERROR_STREAM("TM_COM: Get socketopt SO_ERROR FAIL");
        errno = err;
        return -1;
      }
    }
    else {
      tmrl_ERROR_STREAM("TM_COM: Connection is not ready");
      return -1;
    }
    if (err != 0) {
      errno = err;
      tmrl_ERROR_STREAM("TM_COM: Connection error");
      return -1;
    }
  }
  return rv;
}
bool Client::Connect(int timeout_ms)
{
  if (_sockfd > 0) return true;

  if (timeout_ms < 0) timeout_ms = 0;

#ifdef _WIN32
  addrinfo hints;
  ZeroMemory(&hints, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_HOPOPTS;

  _sockfd = socket(hints.ai_family, hints.ai_socktype, hints.ai_protocol);
#else
  _sockfd = socket(AF_INET, SOCK_STREAM, 0);
#endif
  if (_sockfd < 0) {
    tmrl_ERROR_STREAM("TM_COM: Error socket");
    return false;
  }

  if (setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&_optflag, sizeof(_optflag)) < 0) {
    tmrl_WARN_STREAM("TM_COM: setsockopt TCP_NODELAY failed");
  }
#ifndef _WIN32
  if (setsockopt(_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *)&_optflag, sizeof(_optflag)) < 0) {
    tmrl_WARN_STREAM("TM_COM: setsockopt TCP_QUICKACK failed");
  }
#endif
  if (setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&_optflag, sizeof(_optflag)) < 0) {
    tmrl_WARN_STREAM("TM_COM: setsockopt SO_REUSEADDR failed");
  }
#ifdef _WIN32
  DWORD tv = timeout_ms;
  if (setsockopt(_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv)) < 0) {
    tmr_WARN_STREAM("TM_COM: setsockopt SO_SNDTIMEO failed");
  }
#else
  timeval tv;
  tv.tv_sec = (timeout_ms / 1000);
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  // already timeout by select(...)
  //if (setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0) {
  //	tmr_WARN_STREAM("TM_COM: setsockopt SO_RCVTIMEO failed");
  //}
  if (setsockopt(_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv)) < 0) {
    tmrl_WARN_STREAM("TM_COM: setsockopt SO_SNDTIMEO failed");
  }
#endif

  if (_connect(_sockfd, _ip.c_str(), _port, timeout_ms) == 0) {
    tmrl_INFO_STREAM("TM_COM: O_NONBLOCK connection is ok");
    tmrl_INFO_STREAM("TM_COM: TM robot is connected. sockfd:=" << _sockfd);
    return true;
  }
  else {
    tmrl_INFO_STREAM("TM_COM: O_NONBLOCK connection is fail");
    //_sockfd = -1;
    return false;
  }
}
void Client::Close()
{
  // reset
  _recv_rc = RetCode::OK;
  _recv_ready = false;

  if (_sockfd <= 0) return;
#ifdef _WIN32
  closesocket((SOCKET)_sockfd);
#else
  close(_sockfd);
#endif
  _sockfd = -1;
}

RetCode Client::send_bytes(const char *bytes, int len, int *n)
{
  RetCode rc = RetCode::OK;
  if (n) *n = 0;

  if (len <= 0) return RetCode::OK;
  if (_sockfd < 0) return RetCode::NOTREADY;

  int nb = send(_sockfd, bytes, len, 0);

  if (nb < 0) {
    rc = RetCode::ERR;
  }
  else if (nb < len) {
    rc = RetCode::NOTSENDALL;

    if (n) *n = nb;
  }
  return rc;
}
RetCode Client::send_bytes_all(const char *bytes, int len, int *n)
{
  RetCode rc = RetCode::OK;

  if (n) *n = 0;

  if (len <= 0) return RetCode::OK;
  if (_sockfd < 0) return RetCode::NOTREADY;

  int ntotal = 0;
  int nb = 0;
  int nleft = len;

  while (ntotal < len) {
    nb = send(_sockfd, bytes, nleft, 0);
    if (nb < 0) {
      rc = RetCode::ERR;
      break;
    }
    ntotal += nb;
    nleft -= nb;
  }
  if (n) *n = ntotal;
  return rc;
}

RetCode Client::send_packet(Packet &packet, bool info)
{
  vectorXbyte bytes;
  packet.pack(bytes);
  if (info) tmrl_INFO_STREAM(to_string(bytes));
  return send_bytes(bytes.data(), bytes.size());
}
RetCode Client::send_packet_all(Packet &packet, bool info)
{
  vectorXbyte bytes;
  packet.pack(bytes);
  if (info) tmrl_INFO_STREAM(to_string(bytes));
  return send_bytes_all(bytes.data(), bytes.size());
}
RetCode Client::send_packet_(Packet &packet, bool info)
{
  vectorXbyte bytes;
  packet.pack(bytes);
  if (info) tmrl_INFO_STREAM(to_string(bytes));
  if (bytes.size() > 0x1000)
    return send_bytes_all(bytes.data(), bytes.size());
  else
    return send_bytes(bytes.data(), bytes.size());
}

bool Client::init_receiver()
{
  _recv_ready = _recv->init(_sockfd);
  return _recv_ready;
}
RetCode Client::receiver_spin_once(int timeval_ms, int *n)
{
  RetCode rc = RetCode::OK;

  //if (n) *n = 0;

  // spin once
  int nb = 0;
  rc = _recv->spin_once(timeval_ms, &nb);

  if (n) *n = nb;

  // error handling
  if (rc != RetCode::OK) {
    _recv_rc = rc;
    return rc;
  }

  // find packet
  int loop_cnt = 0;
  int pack_cnt = 0;
  char *bdata = NULL;
  size_t blen = 0;
  size_t size = 0;
  size_t len = 0;
  bool cs = false;
  bool ok = false;

  while (loop_cnt < 10 || pack_cnt < 10) {

    blen = _recv->buffer().size();
    if (blen < 9) {
      break;
    }
    bdata = _recv->buffer().data();

    //tmrl_DEBUG_STREAM("data: " << bdata << ", " << loop_cnt);

    ++size;
    _packet_vec.resize(size);

    len = _packet_vec.back().unpack(bdata, blen);

    cs = _packet_vec.back().is_checked();
    ok = _packet_vec.back().is_valid();

    if (ok || !cs) {
      _recv->buffer().pop_front(len);
    }
    if (!cs) {
      tmrl_ERROR_STREAM("TM_COM: checksum error! cs: "
        << (int)(_packet_vec.back().checksum())
        << " len: " << len);
    }
    if (ok) {
      ++pack_cnt;
    }
    else {
      tmrl_ERROR_STREAM("TM_COM: invalid packet");
      if (size > 1) {
        _packet_vec.resize(size - 1);
      }
      //if (pack_cnt != 0) break;
      break;
    }
    ++loop_cnt;
  }
  if (pack_cnt == 0) {
    rc = RetCode::NOVALIDPACK;
  }
  _recv_rc = rc;
  return rc;
}


ClientThread::ClientThread(const std::string &ip, unsigned short port, size_t buffer_size)
  :_client(ip, port, buffer_size)
{
  tmrl_DEBUG_STREAM("tmrl::comm::ClientThread::ClientThread");

  _isOk = []{ return true; };
}
ClientThread::~ClientThread()
{
  tmrl_DEBUG_STREAM("tmrl::comm::ClientThread::~ClientThread");

  stop();
}
bool ClientThread::start(int timeout_ms)
{
  stop();
  bool rb = _client.Connect(timeout_ms);
  _keep_alive = true;
  _thd = std::thread{std::bind(&ClientThread::run, this)};
  return rb;
}
bool ClientThread::start()
{
  return start(_reconnect_timeout_ms);
}
void ClientThread::stop()
{
  _keep_alive = false;
  if (_thd.joinable())
    _thd.join();
}
void ClientThread::run()
{
  tmrl_INFO_STREAM(_hdr << ": thread begin");

  while (isOk()) {
    if (!_client.init_receiver()) {
      tmrl_INFO_STREAM(_hdr << ": is not connected");
    }
    while (isOk() && _client.is_connected()) {
      int n;
      auto rc = _client.receiver_spin_once(1000, &n);
      //tmrl_DEBUG_STREAM(_hdr << ": rc: " << (int)(rc) << " n: " << n);
      if (rc == RetCode::ERR ||
          rc == RetCode::NOTREADY ||
          rc == RetCode::NOTCONNECT) {
        break;
      }
      if (rc == RetCode::OK) {
        //tmrl_DEBUG_STREAM(_hdr << ": pn: " << _client.packet_vector().size());
        if (!receive(_client.packet_vector())) {
          break;
        }
      }
    }
    _client.Close();

    reconnect();
  }
  _client.Close();
  tmrl_INFO_STREAM(_hdr << ": thread end");
}
void ClientThread::reconnect()
{
  if (!isOk()) return;

  if (_reconnect_timeval_ms < 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return;
  }
  else if (_reconnect_timeval_ms <= 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  tmrl_INFO_STREAM(_hdr << ": reconnect in ");
  int cnt = 0;
  while (isOk() && cnt < _reconnect_timeval_ms) {
    if (cnt % 500 == 0) {
      tmrl_INFO_STREAM(0.001 * (_reconnect_timeval_ms - cnt) << " sec...");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cnt += 10;
  }
  if (isOk() && _reconnect_timeval_ms >= 0) {
    tmrl_INFO_STREAM("0 sec.");
    tmrl_INFO_STREAM(_hdr << ": connect( " << _reconnect_timeout_ms << " ms )...");
    _client.Connect(_reconnect_timeout_ms);
  }
}

}
}