#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace tmrl
{
namespace comm
{

using vectorXbyte = std::vector<char>;

inline std::string to_string(const vectorXbyte &bytes)
{
  return std::string{bytes.begin(), bytes.end()};
}

class Packet
{
public:
  static const char P_HEAD;
  static const char P_END1;
  static const char P_END2;
  static const char P_SEPR;
  static const char P_CSUM;

  static const std::string HDR_CPERR;
  static const std::string HDR_TMSCT;
  static const std::string HDR_TMSTA;
  static const std::string HDR_TMSVR;

  enum class Header {
    EMPTY,
    CPERR,
    TMSCT,
    TMSTA,
    TMSVR,
    OTHER
  };

  Packet() = default;
  Packet(const Packet &) = default;
  Packet & operator=(const Packet &) = default;
  Packet(Packet &&) = default;
  Packet & operator=(Packet &&) = default;

  virtual ~Packet() = default;

  virtual size_t pack(vectorXbyte &bytes);
  virtual size_t unpack(const char *bytes, size_t size);

  virtual void reset()
  {
    set_header(Header::EMPTY);
    //_data.clear();
    _size = 0;
    _checksum = 0;
    _is_checksum_error = false;
    _is_valid = false;
  }

  Packet & set_data(Header header, const vectorXbyte &data)
  {
    set_header(header);
    _data = data;
    return *this;
  }
  Packet & set_data(Header header, vectorXbyte &&data)
  {
    set_header(header);
    _data = std::move(data);
    return *this;
  }

  Header header() const { return _header; }
  const std::string & header_str() const { return _header_str; }
  const vectorXbyte & data() const { return _data; };
  std::string get_data_str() const { return std::string{_data.begin(), _data.end()}; }

  size_t size() const { return _size; }
  char checksum() const { return _checksum; }
  bool is_checksum_error() const { return _is_checksum_error; }
  bool is_valid() const { return _is_valid; }

protected:
  void set_header(Header header);
  void set_header(const std::string &hdr_str);

  Header _header = Header::EMPTY;
  std::string _header_str;
  vectorXbyte _data;

  size_t _size = 0;
  char _checksum = 0;
  bool _is_checksum_error = false;
  bool _is_valid = false;

  // static helpers

  static char checksum_xor(const char *data, size_t size);
  static std::string string_from_hex_uint8(unsigned char num);
  static unsigned char hex_uint8_from_string(const std::string &s);
};

class TmsvrPacket : public Packet
{
public:
  enum class Mode : char {
    RESPONSE = 0,
    BINARY,
    STRING,
    JSON,
    READ_BINARY = 11,
    READ_STRING,
    READ_JSON,
    UNKNOW
  };
  enum class ErrCode {
    Ok,
    NotSupport,
    WritePermission,
    InvalidData,
    NotExist,
    ReadOnly,
    ModeError,
    ValueError,
    Other
  };

  size_t pack(vectorXbyte &bytes) override;
  size_t unpack(const char *bytes, size_t size) override;

  void pack_content(vectorXbyte &data);
  void unpack_content(const char *data, size_t size);

  void reset() override
  {
    Packet::reset();
    _err_code = ErrCode::Ok;
  }

  TmsvrPacket & set_content(const std::string &id, Mode mode, const std::string &content)
  {
    _transaction_id = id;
    _mode = mode;
    _content = content;
    return *this;
  }
  TmsvrPacket & set_content(const std::string &id, Mode mode, std::string &&content)
  {
    _transaction_id = id;
    _mode = mode;
    _content = std::move(content);
    return *this;
  }

  const std::string & transaction_id() const { return _transaction_id; }
  Mode mode() const { return _mode; }
  const std::string & content() const { return _content; }
  std::string get_content_str() const { return _content; }

  ErrCode errcode() const { return _err_code; }

private:
  ErrCode unpack_errcode(const char *buf);

  Mode _mode = Mode::RESPONSE;
  std::string _transaction_id;
  std::string _content;

  ErrCode _err_code = ErrCode::Ok;

  //size_t _data_size;
};

class TmsctPacket : public Packet
{
public:
  size_t pack(vectorXbyte &bytes) override;
  size_t unpack(const char *bytes, size_t size) override;

  void pack_script(vectorXbyte &data);
  void unpack_script(const char *data, size_t size);

  void reset() override
  {
    Packet::reset();
    _has_error = false;
  }

  TmsctPacket & set_script(const std::string &id, const std::string &script)
  {
    _id = id;
    _script = script;
    return *this;
  }
  TmsctPacket & set_script(const std::string &id, std::string &&script)
  {
    _id = id;
    _script = std::move(script);
    return *this;
  }

  const std::string & id() const { return _id; }
  const std::string & script() const { return _script; }
  std::string get_script_str() const { return _script; }

  bool has_error() const { return _has_error; }

private:
  std::string _id;
  std::string _script;

  bool _has_error = false;

  //size_t _data_size;
};

class TmstaPacket : public Packet
{
public:
  size_t pack(vectorXbyte &bytes) override;
  size_t unpack(const char *bytes, size_t size) override;

  void pack_subdata(vectorXbyte &data);
  void unpack_subdata(const char *data, size_t size);

  void reset() override
  {
    Packet::reset();
  }

  TmstaPacket & set_subdata(const std::string &subcmd, const std::string &subdata)
  {
    _subcmd = subcmd;
    _subdata = subdata;
    return *this;
  }
  TmstaPacket & set_subdata(const std::string &subcmd, std::string &&subdata)
  {
    _subcmd = subcmd;
    _subdata = std::move(subdata);
    return *this;
  }

  const std::string & subcmd() const { return _subcmd; }
  const std::string & subdata() const { return _subdata; }
  std::string get_subdata_str() const { return _subdata; }

private:
  std::string _subcmd;
  std::string _subdata;

  //size_t _data_size;
};

class CperrPacket : public Packet
{
public:
  enum class ErrCode : unsigned char
  {
    Ok,
    PacketErr,
    ChecksumErr,
    HeaderErr,
    DataErr,
    NoExtSctMode = 0xf1,
    Other = 0xff
  };

  size_t pack(vectorXbyte &bytes) override;
  size_t unpack(const char *bytes, size_t size) override;

  void pack_errcode(vectorXbyte &data);
  void unpack_errcode(const char *data, size_t size);

  void reset() override
  {
    Packet::reset();
    _err_code = ErrCode::Ok;
  }

  CperrPacket & set_errcode(ErrCode err_code)
  {
    _err_code = err_code;
    return *this;
  }

  ErrCode errcode() { return _err_code; }

private:
  ErrCode _err_code;

  //size_t _data_size;
};

}
}
