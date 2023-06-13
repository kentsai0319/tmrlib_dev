
#include "tmrl/utils/logger.h"

#include "tmrl/utils/conversions.h"

#include "tmrl/comm/packet.h"

#include <iostream>

extern void tui(const std::string &host);

int main(int argc, char **argv)
{
  tmrl_INFO_STREAM("Hello!");

  tmrl::utils::get_logger().set_level(tmrl::utils::logger::DEBUG);

  tmrl::vector3d v3{0.1, 1.2, 2.3};
  std::cout << tmrl::to_string(v3) << "\n";
  tmrl::vectorXd vx{1.1, 2.2, 3.3};
  std::cout << tmrl::to_string(vx) << "\n";
  vx = tmrl::to_vectorXd(v3);
  std::cout << tmrl::to_string(vx) << "\n";

  tmrl::comm::vectorXbyte bytes;
  tmrl::comm::TmsvrPacket packet;
  packet.set_content("1", tmrl::comm::TmsvrPacket::Mode::STRING, "123").pack(bytes);
  std::cout << tmrl::comm::to_string(bytes) << "\n";

  tmrl::comm::TmsvrPacket packet2;
  packet2.unpack(bytes.data(), bytes.size());
  std::cout << packet2.header_str() << "\n";
  std::cout << packet2.get_data_str() << "\n";
  std::cout << packet2.get_content_str() << "\n";
  std::cout << packet2.is_error_checksum() << "\n";
  std::cout << packet2.is_valid() << "\n";

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
}
