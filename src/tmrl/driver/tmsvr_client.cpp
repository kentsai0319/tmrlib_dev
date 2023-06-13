#include "tmrl/driver/tmsvr_client.h"
#include "tmrl/utils/logger.h"

namespace tmrl
{
namespace driver
{

TmsvrClient::TmsvrClient(const std::string &ip, size_t buf_n)
  : ClientThread(ip, 5891, buf_n, true)
{
  _hdr = "TM_SVR";
  _responseCallback = [](const comm::TmsvrPacket &pack)
  {
    tmrl_INFO_STREAM("$TMSVR: RESPONSE: id: " << pack.transaction_id()
      << ", errcode: " << (int)(pack.errcode())
      <<", content:\n" << pack.content());
  };
  _readCallback = [](const comm::TmsvrPacket &pack)
  {
    tmrl_INFO_STREAM("$TMSVR: READ: id: "
      << pack.transaction_id() << ", content:\n" << pack.content());
  };
  _feedbackCallback = [](const RobotState &/*rs*/)
  {
  };
  _cperrCallback = [](const comm::CperrPacket &/*pack*/)
  {
  };
}

bool TmsvrClient::send_content(
  const std::string &id, const std::string &content, comm::TmsvrPacket::Mode mode)
{
  comm::TmsvrPacket tmsvr;
  tmsvr.set_content(id, mode, content);
  comm::RetCode rc = _client.send_packet_all(tmsvr, comm::Client::LOG_INFO);
  if (rc == comm::RetCode::ERR) 
    set_reconnet();
  return (rc == comm::RetCode::OK);
}

bool TmsvrClient::receive(const std::vector<comm::Packet> &pack_vec)
{
  using namespace comm;
  TmsvrPacket tmsvr;
  CperrPacket cperr;
  bool fb = false;

  for (auto &pack : pack_vec) {
    if (pack.header() == Packet::Header::TMSVR) {
      tmsvr.unpack_content(pack.data().data(), pack.data().size());

      // tmsvr response
      switch (tmsvr.mode()) {
      case TmsvrPacket::Mode::RESPONSE:
        _responseCallback(tmsvr);
        break;
      case TmsvrPacket::Mode::BINARY:
        tmsvr.unpack_content(pack.data().data(), pack.data().size());
        // parse robot state
        robot_state.deserialize_with_lock(tmsvr.content().data(), tmsvr.content().size());
        fb = true;
        break;
      case TmsvrPacket::Mode::READ_STRING:
      case TmsvrPacket::Mode::READ_JSON:
        _readCallback(tmsvr);
        break;
      default:
        tmrl_WARN_STREAM("$TMSVR: unsupported mode: " << (int)(tmsvr.mode())
          << " id: " << tmsvr.transaction_id());
        break;
      }
    }
    else if (pack.header() == Packet::Header::CPERR) {
      cperr.unpack_errcode(pack.data().data(), pack.data().size());
      tmrl_WARN_STREAM("$TMSVR: CPERR: error code: " << (int)(cperr.errcode()));

      // cperr response
      _cperrCallback(cperr);
    }
    else {
      tmrl_ERROR_STREAM("TM_SVR: invalid header");
    }
  }
  if (fb) {
    _feedbackCallback(robot_state);
  }
  return true;
}

}
}
