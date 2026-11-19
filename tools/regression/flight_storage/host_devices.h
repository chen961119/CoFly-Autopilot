#pragma once
// Deterministic single-core executor. Requests pump the actual storage task only
// when the protocol client blocks. Card contents, short writes and time are explicit.
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class String : public std::string {
public:
  using std::string::string;
  String(const std::string &s) : std::string(s) {}
  String(unsigned long long n) : std::string(std::to_string(n)) {}
  template<class T> explicit String(T n) : std::string(std::to_string(n)) {}
  String(float n, int precision) {
    std::ostringstream s; s << std::fixed << std::setprecision(precision) << n; assign(s.str());
  }
  bool startsWith(const char *s) const { return rfind(s, 0) == 0; }
  bool endsWith(const char *s) const { return size() >= strlen(s) && compare(size()-strlen(s),strlen(s),s)==0; }
  String substring(size_t a, size_t b) const { return substr(a, b-a); }
  char charAt(size_t i) const { return at(i); }
  long toInt() const { return std::stol(*this); }
};
struct Stream {
  template<class T> void print(const T &) {}
  template<class T> void println(const T &) {}
};
inline Stream Serial;
struct SPISettings {};
struct SBUS {};
inline uint32_t nowUs;
inline uint32_t micros() { return nowUs; }
inline uint32_t millis() { return nowUs / 1000; }
inline void delay(unsigned) {}
inline bool isDigit(char c) { return std::isdigit(c) != 0; }
template<class T,class U> T constrain(T n,U low,U high) { return std::clamp(n,T(low),T(high)); }

using TaskHandle_t = void *;
inline int taskIdentity[2];
inline TaskHandle_t currentTask = taskIdentity;
inline auto xTaskGetCurrentTaskHandle() { return currentTask; }
inline void taskENTER_CRITICAL() {}
inline void taskEXIT_CRITICAL() {}
#define configASSERT(x) assert(x)
constexpr int pdTRUE = 1, portMAX_DELAY = -1;
struct StaticQueue_t {
  size_t depth, width;
  std::deque<std::vector<uint8_t>> entries;
};
using QueueHandle_t = StaticQueue_t *;
inline auto xQueueCreateStatic(size_t depth,size_t width,uint8_t *,StaticQueue_t *queue) {
  queue->depth = depth; queue->width = width; return queue;
}
inline int xQueueSend(QueueHandle_t q,const void *p,int) {
  if (q->entries.size() == q->depth) return 0;
  auto *item = static_cast<const uint8_t *>(p);
  q->entries.emplace_back(item,item+q->width); return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t q,void *out,int) {
  if (q->entries.empty()) return 0;
  memcpy(out,q->entries.front().data(),q->width); q->entries.pop_front(); return pdTRUE;
}
inline size_t uxQueueMessagesWaiting(QueueHandle_t q) { return q->entries.size(); }
using StaticSemaphore_t = bool;
using SemaphoreHandle_t = bool *;
inline auto xSemaphoreCreateBinaryStatic(bool *s) { *s=false; return s; }
inline void xSemaphoreGive(bool *s) { *s=true; }
void pumpStorage();
inline int xSemaphoreTake(bool *s,int) {
  if (!*s) pumpStorage();
  assert(*s); *s=false; return pdTRUE;
}

constexpr int BUILTIN_SDCARD=1, FILE_READ=0, FILE_WRITE=1, FILE_WRITE_BEGIN=2, O_RDWR=1, O_CREAT=2, O_AT_END=4;
struct DateTimeFields { int year,mon,mday,hour,min,sec; };
inline std::map<std::string,std::string> files;
inline bool failWrite=false, failSync=false;
inline std::string shortWritePath, syncErrorPath, closeErrorPath;
class File {
public:
  File() = default;
  explicit File(std::string p) : path(std::move(p)) {}
  explicit operator bool() const { return !path.empty(); }
  size_t println(const char *s) {
    auto &out=files.at(path); out+=s;
    if (failWrite) return strlen(s);
    out+="\r\n"; return strlen(s)+2;
  }
  size_t println(const String &s) { return println(s.c_str()); }
  bool close() {
    const bool ok = path != closeErrorPath;
    path.clear(); return ok;
  }
  void flush() {}
  bool sync() { return !failSync && path != syncErrorPath; }
  bool getWriteError() const { return failWrite; }
  uint64_t curPosition() const { return size(); }
  bool truncate(size_t n) { files.at(path).resize(n); return true; }
  File openNextFile(int = FILE_READ) { return {}; }
  const char *name() const { return path.c_str()+1; }
  bool isDirectory() const { return path=="/"; }
  bool getModifyTime(DateTimeFields &) { return false; }
  size_t size() const { return files.at(path).size(); }
  bool seek(size_t n) { pos=n; return pos<=size(); }
  size_t read(void *out,size_t n) {
    n=std::min(n,size()-pos); memcpy(out,files.at(path).data()+pos,n); pos+=n; return n;
  }
  size_t write(const void *data,size_t n) {
    assert(currentTask==taskIdentity+1);
    if(path==shortWritePath && n) --n;
    files.at(path).replace(pos,n,static_cast<const char *>(data),n);
    pos+=n; return n;
  }
private:
  std::string path;
  size_t pos=0;
};
using FsFile = File;
struct Card {
  bool begin(int) { return true; }
  void end() {}
  bool remove(std::string name) { return files.erase(name.front()=='/' ? name : "/"+name)>0; }
  File open(std::string name,int mode=FILE_READ) {
    if (name.front()!='/') name="/"+name;
    if (name!="/" && !mode && !files.count(name)) return {};
    files[name]; return File(name);
  }
};
inline struct : Card { Card sdfs; } SD;
