#include "tmrl/driver/tmsct_client.h"
#include "tmrl/utils/logger.h"

namespace tmrl
{
namespace driver
{

TmsctClient::TmsctClient(const std::string &ip, size_t buf_n)
  : ClientThread(ip, 5890, buf_n)
{
  _hdr = "TM_SCT";
  _tmsctCallback = [](const comm::TmsctPacket &pack)
  {
    std::string res;
    if (pack.has_error())
      res = "err";
    else
      res = "res";

    tmrl_INFO_STREAM("$TMSCT: " << res << ": id: "
        << pack.id() << ", script:\n" << pack.script());
  };
  _tmstaCallback = [](const comm::TmstaPacket &pack)
  {
    tmrl_INFO_STREAM("$TMSTA: res: subcmd: "
      << pack.subcmd() << ", subdata:\n" << pack.subdata());
  };
  _cperrCallback = [](const comm::CperrPacket &/*pack*/)
  {
  };
}

bool TmsctClient::send_script(const std::string &id, const std::string script, bool info)
{
  comm::TmsctPacket tmsct;
  tmsct.set_script(id, script);
  return (_client.send_packet_all(tmsct, info) == comm::RetCode::OK);
}
bool TmsctClient::send_sta_request(const std::string &subcmd, const std::string &subdata)
{
  comm::TmstaPacket tmsta;
  tmsta.set_subdata(subcmd, subdata);
  return (_client.send_packet_all(tmsta, comm::Client::LOG_INFO) == comm::RetCode::OK);
}

bool TmsctClient::receive(const std::vector<comm::Packet> &pack_vec)
{
  using namespace comm;
  TmsctPacket tmsct;
  TmstaPacket tmsta;
  CperrPacket cperr;

  for (auto &pack : pack_vec) {
    switch (pack.header()) {
    case Packet::Header::TMSCT:
      tmsct.unpack_script(pack.data().data(), pack.data().size());

      // tmsct response
      _tmsctCallback(tmsct);
      break;
    case Packet::Header::TMSTA:
      tmsta.unpack_subdata(pack.data().data(), pack.data().size());

      // tmsta response
      _tmstaCallback(tmsta);
      break;
    case Packet::Header::CPERR:
      cperr.unpack_errcode(pack.data().data(), pack.data().size());
      tmrl_WARN_STREAM("$TMSCT: CPERR: error code: " << (int)(cperr.errcode()));

      // cperr response
      _cperrCallback(cperr);
      break;
    default:
      tmrl_ERROR_STREAM("TM_SCT: invalid header");
      break;
    }
  }
  return true;
}

}
}
