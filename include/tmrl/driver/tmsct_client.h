#pragma once

#include "tmrl/comm/client.h"

namespace tmrl
{
namespace driver
{

class TmsctClient : public comm::ClientThread
{
public:
  using TmsctCallback = std::function<void(const comm::TmsctPacket &)>;
  using TmstaCallback = std::function<void(const comm::TmstaPacket &)>;
  using CperrCallback = std::function<void(const comm::CperrPacket &)>;

  explicit TmsctClient(const std::string &ip, size_t buf_n = 0x800);
  ~TmsctClient() = default;

  void set_tmsct_callback(TmsctCallback cb) { _tmsctCallback = cb; }
  void set_tmsta_callback(TmstaCallback cb) { _tmstaCallback = cb; }
  void set_cperr_callback(CperrCallback cb) { _cperrCallback = cb; }

  bool send_script(const std::string &id, const std::string script, bool info = true);
  bool send_sta_request(const std::string &subcmd, const std::string &subdata);

private:
  bool receive(const std::vector<comm::Packet> &pack_vec) override;

  TmsctCallback _tmsctCallback;
  TmstaCallback _tmstaCallback;
  CperrCallback _cperrCallback;
};

}
}