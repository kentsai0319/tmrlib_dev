#pragma once

#include <vector>

namespace tmrl
{
namespace comm
{

class SBuffer
{
public:
  int append(const char *bdata, int blen)
  {
    if (blen <= 0) return 0;

    size_t old_size = _bytes.size();
    size_t new_size = old_size + blen;
    _bytes.resize(new_size);
    for (size_t i = 0; i < size_t(blen); ++i) {
      _bytes[old_size + i] = bdata[i];
    }
    //tmrl_DEBUG_STREAM("SBuffer::append " << (int)(blen) << " bytes");
    return blen;
  }
  void pop_front(int len = 1)
  {
    // commit extract
    if (len <= 0) return;

    if ((size_t)(len) < _bytes.size()) {
      //std::vector<char> tmp{ _bytes.begin() + len, _bytes.end() };
      //_bytes.clear();
      //_bytes.insert(_bytes.end(), tmp.begin(), tmp.end());
      _bytes.erase(_bytes.begin(), _bytes.begin() + len);
    }
    else {
      _bytes.clear();
    }
    //tmrl_DEBUG_STREAM("SBuffer::pop_front ", << (int)(len) << " bytes");
  }
  void clear() { _bytes.clear(); }
  char *data() { return _bytes.data(); }
  int length() const { return (int)(_bytes.size()); }
  size_t size() const { return _bytes.size(); }

private:
  std::vector<char> _bytes;
};

}
}