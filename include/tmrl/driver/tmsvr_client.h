#pragma once

#include "tmrl/comm/client.h"
#include "tmrl/driver/robot_state.h"

namespace tmrl
{
namespace driver
{

class TmsvrClient : public comm::ClientThread
{
public:
  using ResponseCallback = std::function<void(const comm::TmsvrPacket &)>;
  using ReadCallback = std::function<void(const comm::TmsvrPacket &)>;
  using FeedbackCallback = std::function<void(const RobotState &rs)>;
  using CperrCallback = std::function<void(const comm::CperrPacket &)>;

  explicit TmsvrClient(const std::string &ip, size_t buf_n = 0x1000);
  ~TmsvrClient() = default;

  void set_response_callback(ResponseCallback cb) { _responseCallback = cb; }
  void set_read_callback(ReadCallback cb) { _readCallback = cb; }
  void set_feedback_callback(FeedbackCallback cb) { _feedbackCallback = cb; }
  void set_cperr_callback(CperrCallback cb) { _cperrCallback = cb; }

  bool send_content(
    const std::string &id, const std::string &content,
    comm::TmsvrPacket::Mode mode = comm::TmsvrPacket::Mode::STRING);

  RobotState robot_state;

private:
  bool receive(const std::vector<comm::Packet> &pack_vec) override;

  ResponseCallback _responseCallback;
  ReadCallback _readCallback;
  FeedbackCallback _feedbackCallback;
  CperrCallback _cperrCallback;
};

}
}