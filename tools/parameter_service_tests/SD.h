#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
constexpr int FILE_READ=0, FILE_WRITE_BEGIN=1;
extern bool sdFails;
extern unsigned sdWrites;
extern std::map<std::string,std::vector<uint8_t>> files;
struct File {
  std::string path;
  bool opened=false;
  explicit operator bool() const { return opened; }
  size_t size() const { return files[path].size(); }
  size_t read(uint8_t *data,size_t length) {
    length=std::min(length,files[path].size()); memcpy(data,files[path].data(),length); return length;
  }
  bool truncate(size_t length) { if (sdFails) return false; files[path].resize(length); return true; }
  size_t write(const uint8_t *data,size_t length) {
    ++sdWrites; if (sdFails) return 0; files[path].assign(data,data+length); return length;
  }
  void close() {}
  void flush() {}
};
struct MockSD {
  File open(const char *path,int mode) { return {path,!sdFails && (mode!=FILE_READ || files.count(path)!=0)}; }
  void remove(const char *path) { files.erase(path); }
};
extern MockSD SD;
