#include "tmrl/comm/packet.h"

#include <sstream>
#include <iomanip>

namespace tmrl
{
namespace comm
{

const char Packet::P_HEAD = 0x24; // $
const char Packet::P_END1 = 0x0D; // \r
const char Packet::P_END2 = 0x0A; // \n
const char Packet::P_SEPR = 0x2C; // ,
const char Packet::P_CSUM = 0x2A; // *

const std::string Packet::HDR_CPERR = "CPERR";
const std::string Packet::HDR_TMSCT = "TMSCT";
const std::string Packet::HDR_TMSTA = "TMSTA";
const std::string Packet::HDR_TMSVR = "TMSVR";

void Packet::set_header(Header header)
{
  _header = header;
  switch (header) {
  case Header::EMPTY: _header_str.clear(); break;
  case Header::CPERR: _header_str = HDR_CPERR; break;
  case Header::TMSCT: _header_str = HDR_TMSCT; break;
  case Header::TMSTA: _header_str = HDR_TMSTA; break;
  case Header::TMSVR: _header_str = HDR_TMSVR; break;
  case Header::OTHER: break;
  }
}
void Packet::set_header(const std::string &hdr_str)
{
  if (hdr_str.size() == 0) {
    _header = Header::EMPTY;
    return;
  }
  if (hdr_str.compare(HDR_CPERR) == 0) {
    _header = Header::CPERR;
  }
  else if (hdr_str.compare(HDR_TMSCT) == 0) {
    _header = Header::TMSCT;
  }
  else if (hdr_str.compare(HDR_TMSTA) == 0) {
    _header = Header::TMSTA;
  }
  else if (hdr_str.compare(HDR_TMSVR) == 0) {
    _header = Header::TMSVR;
  }
  else {
    _header = Header::OTHER;
  }
  _header_str = hdr_str;
}

char Packet::checksum_xor(const char *data, size_t size)
{
  char cs = 0x00;
  for (size_t i = 0; i < size; ++i) { cs ^= (char)(data[i]); }
  return cs;
}
std::string Packet::string_from_hex_uint8(unsigned char num)
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << std::hex << int(num);
  return ss.str();
}
unsigned char Packet::hex_uint8_from_string(const std::string &s)
{
  if (s.size() != 2) return 0;
  int val;
  std::stringstream ss;
  ss << std::hex << s;
  ss >> val;
  return (unsigned char)(val);
}

size_t Packet::pack(vectorXbyte &bytes)
{
  char cs = 0x00;
  bytes.clear();
  // Header
  bytes.push_back(P_HEAD);
  bytes.insert(bytes.end(), std::begin(_header_str), std::end(_header_str));
  bytes.push_back(P_SEPR);
  // Length
  size_t length = _data.size();
  std::string slen = std::to_string(length);
  bytes.insert(bytes.end(), std::begin(slen), std::end(slen));
  bytes.push_back(P_SEPR);
  // Data
  bytes.insert(bytes.end(), std::begin(_data), std::end(_data));
  bytes.push_back(P_SEPR);
  // Checksum
  cs = checksum_xor(bytes.data() + 1, bytes.size() - 1);
  //cs = checksum_xor(bytes, 1, -1);
  bytes.push_back(P_CSUM);
  std::string shex = string_from_hex_uint8(cs);
  bytes.insert(bytes.end(), std::begin(shex), std::end(shex));
  // End
  bytes.push_back(P_END1);
  bytes.push_back(P_END2);
  // save info
  _size = bytes.size();
  _checksum = cs;
  _is_valid = true;
  return _size;
}

size_t Packet::unpack(const char *bytes, size_t size)
{
  size_t ind_e = 1, ind_b = 1;
  size_t length = 0;
  char cs = 0;
  bool is_valid = true;

  if (size < 9) {
    is_valid = false;
  }
  if (bytes[0] != P_HEAD) {
    is_valid = false;
  }
  if (!is_valid) goto end;

  //size_t ind_b = 1, ind_e = 1;

  // find end of header (first P_SEPR)
  while (ind_e < size && bytes[ind_e] != P_SEPR) {
    ++ind_e;
  }
  // didn't find header
  if (ind_e + 8 > size) {
    is_valid = false; goto end;
  }
  // setup header
  if (ind_e > 1) {
    std::string hdr{ bytes + ind_b, ind_e - ind_b };
    set_header(hdr);
  }
  else {
    set_header(Header::EMPTY);
  }
  ++ind_e;
  ind_b = ind_e;

  // check header

  // find end of length
  while (ind_e < size && bytes[ind_e] != P_SEPR) {
    ++ind_e;
  }
  // didn't find length
  if (ind_e + 7 > size) {
    is_valid = false; goto end;
  }
  // get length
  if (ind_e > ind_b) {
    std::string len{ bytes + ind_b, ind_e - ind_b };
    length = std::stoi(len);
  }
  ++ind_e;
  ind_b = ind_e;

  // check length
  if (ind_e + length + 6 > size || bytes[ind_e + length] != P_SEPR) {
    is_valid = false; goto end;
  }

  // find and save data, and checksum
  _data.clear();
  _data.resize(length);

  for (size_t i = 0; i < length; ++i) {
    _data[i] = bytes[ind_b + i];
    //cs ^= bytes[ind_b + i];
  }
  cs = checksum_xor(bytes + 1, ind_e + length);
  ind_e += length + 1;
  ind_b = ind_e;

  // check checksum (P_CSUM)
  if (bytes[ind_e] != P_CSUM) {
    is_valid = false; goto end;
  }
  ++ind_e;
  ind_b = ind_e;

  // check checksum 
  {
    int val;
    std::stringstream ss;
    ss << std::hex << bytes[ind_e] << bytes[ind_e + 1];
    ss >> val;
    _checksum = (char)(val);
    if (cs != _checksum) {
      //packet._is_cs_failed = true;
      is_valid = false;
    }
    else {
      _is_checked = true;
    }
  }
  ind_e += 2;
  ind_b = ind_e;

  // check end
  if (bytes[ind_e] != P_END1 || bytes[ind_e + 1] != P_END2) {
    ++ind_e;
    is_valid = false; goto end;
  }
  ind_e += 2;
  ind_b = ind_e;

  _size = ind_e;
  _is_valid = true;
end:
  if (!is_valid) {
    reset();
    _size = ind_e;
  }
  return ind_e;
}

//
// TmsvrPacket
//

size_t TmsvrPacket::pack(vectorXbyte &bytes)
{
  _header = Header::TMSVR;
  _header_str = HDR_TMSVR;
  pack_content(_data);
  return Packet::pack(bytes);
}
size_t TmsvrPacket::unpack(const char *bytes, size_t size)
{
  size_t len = Packet::unpack(bytes, size);
  unpack_content(_data.data(), _data.size());
  if (_header != Header::TMSVR) {
  }
  return len;
}
TmsvrPacket::ErrCode TmsvrPacket::unpack_errcode(const char *buf)
{
  char cc[3] = { buf[0], buf[1], '\0' };
  int ic = std::atoi(cc);
  if (ic < (int)(ErrCode::Other)) {
    return ErrCode(ic);
  }
  else {
    return ErrCode::Other;
  }
}
void TmsvrPacket::pack_content(vectorXbyte &data)
{
  data.clear();
  data.insert(data.end(), std::begin(_transaction_id), std::end(_transaction_id));
  data.push_back(P_SEPR);
  std::string smode = std::to_string((int)(_mode));
  data.insert(data.end(), std::begin(smode), std::end(smode));
  data.push_back(P_SEPR);
  data.insert(data.end(), std::begin(_content), std::end(_content));
  /*size_t ind_b = data.size();
  size_t ind_e = ind_b;
  size_t new_size = ind_e + _content.size();
  data.resize(new_size);
  for (; ind_e < new_size; ++ind_e) {
    data[ind_e] = _content[ind_e - ind_b];
  }*/
  //_data_size = data.size();
}
void TmsvrPacket::unpack_content(const char *data, size_t size)
{
  _is_valid = false;

  if (!data) {
    return;
  }

  //size_t ind_b = 0;
  size_t ind_e = 0;

  // find end of transaction ID (P_SEPR)
  while (ind_e < size && data[ind_e] != P_SEPR) {
    ++ind_e;
  }
  if (ind_e + 2 > size) {
    return;
  }

  _transaction_id = std::string{ data, ind_e };

  ++ind_e;
  //ind_b = ind_e;

  // mode 0/1/2/3/ (deprecated)

  /*char amode[] = { data[ind_e], '\0' };
  char cmode = std::atoi(amode);
  if (cmode > (char)(Mode::UNKNOW) || data[ind_e] < (char)(Mode::RESPONSE)) {
    return;
  }
  _mode = (Mode)(cmode);
  ++ind_e;
  if (data[ind_e] != P_SEPR) {
    return;
  }
  ++ind_e;*/

  // mode 0/1/2/3/ AND 11/12/13

  char amode[] = { data[ind_e], '\0', '\0' };
  char cmode = 0;
  if (data[ind_e + 1] == P_SEPR) {
    ++ind_e;
  }
  else if (data[ind_e + 2] == P_SEPR) {
    amode[1] = data[ind_e + 1];
    ind_e += 2;
  }
  else {
    return;
  }
  cmode = std::atoi(amode);
  if (cmode > (char)(Mode::UNKNOW)) {
    return;
  }
  ++ind_e;
  _mode = (Mode)(cmode);

  // mode 0~255 ?

  // find end of mode (P_SEPR)
  /*size_t ind_b = ind_e;
  while (ind_e < size && data[ind_e] != P_SEPR) {
    ++ind_e;
  }
  std::string smode = std::string{ data + ind_b, ind_e - ind_b };
  char cmode = std::atoi(smode.c_str());
  if (cmode > (char)(Mode::UNKNOW)) {
    return;
  }
  ++ind_e;
  _mode = Mode(cmode);*/

  if (_mode == Mode::RESPONSE) {
    if (ind_e + 3 > size) {
      return;
    }
    _err_code = unpack_errcode(data + ind_e);

    ind_e += 2;

    if (data[ind_e] != P_SEPR) {
      return;
    }

    ++ind_e;
  }
  else {
    _err_code = ErrCode::Ok;
  }

  //_content_size = size - ind_e;

  _content = std::string{ data + ind_e, size - ind_e };

  //_data_size = size;
  _is_valid = true;
}

//
// TmsctPacket
//

size_t TmsctPacket::pack(vectorXbyte &bytes)
{
  _header = Header::TMSCT;
  _header_str = HDR_TMSCT;
  pack_script(_data);
  return Packet::pack(bytes);
}
size_t TmsctPacket::unpack(const char *bytes, size_t size)
{
  size_t len = Packet::unpack(bytes, size);
  unpack_script(_data.data(), _data.size());
  if (_header != Header::TMSCT) {
  }
  return len;
}
void TmsctPacket::pack_script(vectorXbyte &data)
{
  data.clear();
  data.insert(data.end(), std::begin(_id), std::end(_id));
  data.push_back(P_SEPR);
  data.insert(data.end(), std::begin(_script), std::end(_script));
  /*size_t ind_b = data.size();
  size_t ind_e = ind_b;
  size_t new_size = ind_e + _script.size();
  data.resize(new_size);
  for (; ind_e < new_size; ++ind_e) {
    data[ind_e] = _script[ind_e - ind_b];
  }*/
  //_data_size = data.size();
}
void TmsctPacket::unpack_script(const char *data, size_t size)
{
  _is_valid = false;

  if (!data) {
    return;
  }

  size_t ind_e = 0;
  // find end of script ID (P_SEPR)
  while (ind_e < size && data[ind_e] != P_SEPR) {
    ++ind_e;
  }
  if (ind_e + 1 > size) {
    return;
  }
  _id = std::string{ data, ind_e };

  ++ind_e;

  _script = std::string{ data + ind_e, size - ind_e };

  if (_script.compare("OK") == 0) {
    _has_error = false;
  }
  else if (_script.compare("ERROR") == 0) {
    _has_error = true;
  }

  //_data_size = size;
  _is_valid = true;
}

//
// TmstaPacket
//

size_t TmstaPacket::pack(vectorXbyte &bytes)
{
  _header = Header::TMSTA;
  _header_str = HDR_TMSTA;
  pack_subdata(_data);
  return Packet::pack(bytes);
}
size_t TmstaPacket::unpack(const char *bytes, size_t size)
{
  size_t len = Packet::unpack(bytes, size);
  unpack_subdata(_data.data(), _data.size());
  if (_header != Header::TMSTA) {
  }
  return len;
}
void TmstaPacket::pack_subdata(vectorXbyte &data)
{
  data.clear();
  data.insert(data.end(), std::begin(_subcmd), std::end(_subcmd));
  data.push_back(P_SEPR);
  data.insert(data.end(), std::begin(_subdata), std::end(_subdata));
  /*size_t ind_b = data.size();
  size_t ind_e = ind_b;
  size_t new_size = ind_e + _subdata.size();
  data.resize(new_size);
  for (; ind_e < new_size; ++ind_e) {
    data[ind_e] = _subdata[ind_e - ind_b];
  }*/
  //_data_size = data.size();
}
void TmstaPacket::unpack_subdata(const char *data, size_t size)
{
  _is_valid = false;

  size_t ind_e = 0;
  // find end of SubCmd (P_SEPR)
  while (ind_e < size && data[ind_e] != P_SEPR) {
    ++ind_e;
  }
  if (ind_e + 1 > size) {
    return;
  }
  _subcmd = std::string{ data, ind_e };
  //unsigned char cmd = hex_uint8_from_string(_subcmd);

  ++ind_e;

  _subdata = std::string{ data + ind_e, size - ind_e };

  //_data_size = size;
  _is_valid = true;
}

//
// CperrPacket
//

size_t CperrPacket::pack(vectorXbyte &bytes)
{
  _header = Header::CPERR;
  _header_str = HDR_CPERR;
  pack_errcode(_data);
  return Packet::pack(bytes);
}
size_t CperrPacket::unpack(const char *bytes, size_t size)
{
  size_t len = Packet::unpack(bytes, size);
  unpack_errcode(_data.data(), _data.size());
  if (_header != Header::CPERR) {
  }
  return len;
}
void CperrPacket::pack_errcode(vectorXbyte &data)
{
  data.clear();
  std::string ec_str = string_from_hex_uint8((unsigned char)(_err_code));
  data.insert(data.end(), std::begin(ec_str), std::end(ec_str));
}
void CperrPacket::unpack_errcode(const char *data, size_t size)
{
  if (size != 2) {
    _err_code = ErrCode::Other;
    _data.clear();
    return;
  }
  _err_code = (ErrCode)(hex_uint8_from_string(std::string{ data, size }));
}


}
}