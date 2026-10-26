#include "ground_station_service/sd_file_service.h"
#include "logging/logger.h"
#include "system/flight_lock.h"
#include <SD.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr size_t LEGACY_BLOCK_SIZE = 256;
constexpr uint32_t IDLE_MS = 15000;
File directory, source;
bool active = false;
uint32_t lastActivity = 0;
uint64_t fileSize = 0, offset = 0;
uint32_t fileCrc = 0xffffffff;
// Repeat the last request verbatim after a timeout without advancing NEXT/READ.
char lastCommand[768] = {}, lastResponse[SD_FILE_RESPONSE_SIZE] = {};
uint8_t block[SD_FILE_BLOCK_SIZE];

void release() {
  directory.close();
  source.close();
  active = false;
  lastCommand[0] = lastResponse[0] = 0;
}

bool number(const char *text, uint64_t &value) {
  if (!text || !*text) return false;
  value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9' || value > (UINT64_MAX - (*text - '0')) / 10) return false;
    value = value * 10 + (*text - '0');
  }
  return true;
}
int nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
void hex(const uint8_t *data, size_t size, char *out) {
  const char digits[] = "0123456789abcdef";
  for (size_t i = 0; i < size; ++i) {
    *out++ = digits[data[i] >> 4];
    *out++ = digits[data[i] & 15];
  }
  *out = 0;
}
bool pathFromHex(const char *encoded, char *path) {
  const size_t length = strlen(encoded);
  if (!length || length % 2 || length > 510) return false;
  for (size_t i = 0; i < length / 2; ++i) {
    const int a = nibble(encoded[2 * i]), b = nibble(encoded[2 * i + 1]);
    if (a < 0 || b < 0) return false;
    const unsigned c = (a << 4) | b;
    if (c < 32 || c == 127 || c == '\\' || c == ':') return false;
    path[i] = c;
  }
  path[length / 2] = 0;
  if (path[0] != '/') return false;
  // Reject traversal and noncanonical paths, including trailing separators.
  const char *start = path + 1;
  while (*start) {
    const char *end = strchr(start, '/');
    const size_t size = end ? static_cast<size_t>(end - start) : strlen(start);
    if (!size || (size == 1 && start[0] == '.') ||
        (size == 2 && start[0] == '.' && start[1] == '.')) return false;
    if (!end) break;
    start = end + 1;
    if (!*start) return false;
  }
  return true;
}
uint32_t crcUpdate(uint32_t crc, const uint8_t *data, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return crc;
}
}

bool sdFileSessionActive() { return active; }

void pollSdFileService(bool connected) {
  if (!connected || !isFlightLocked() ||
      (active && static_cast<uint32_t>(millis() - lastActivity) > IDLE_MS)) release();
}

void handleSdFileCommand(char *line, char *response, size_t capacity) {
  char command[768];
  snprintf(command, sizeof(command), "%s", line);
  char *tokens[6], *save;
  size_t count = 0;
  for (char *p = strtok_r(line, " \t", &save); p; p = strtok_r(nullptr, " \t", &save)) {
    if (count == 6) { count = 0; break; }
    tokens[count++] = p;
  }
  uint64_t id = 0;
  if (count < 3 || strcmp(tokens[0], "SD") || !number(tokens[2], id) || id > UINT32_MAX) {
    snprintf(response, capacity, "@COFLY\t0\tERROR\tSYNTAX\tInvalid SD command\n");
    return;
  }
  auto error = [&](const char *code) {
    snprintf(response, capacity, "@COFLY\t%lu\tERROR\t%s\tSD request failed\n", (unsigned long)id, code);
  };
  // CANCEL is always available, including after an unlock or idle timeout.
  if (!strcmp(tokens[1], "CANCEL") && count == 3) {
    release();
    snprintf(response, capacity, "@COFLY\t%lu\tSD_CANCEL\n", (unsigned long)id);
    return;
  }
  if (!isFlightLocked()) { release(); error("LOCK_REQUIRED"); return; }
  if (!loggerSdReady()) { release(); error("NO_SD"); return; }
  lastActivity = millis();
  if (active && !strcmp(command, lastCommand)) {
    snprintf(response, capacity, "%s", lastResponse);
    return;
  }
  if ((!strcmp(tokens[1], "LIST") || !strcmp(tokens[1], "OPEN")) && count == 4) {
    char path[256];
    if (!pathFromHex(tokens[3], path)) { error("PATH"); return; }
    if (!active) flushLogger();
    active = true;
    directory.close();
    source.close();
    File opened = SD.open(path, FILE_READ);
    const bool listing = !strcmp(tokens[1], "LIST");
    if (!opened || opened.isDirectory() != listing) {
      opened.close();
      error("OPEN_FAILED");
    } else if (listing) {
      directory = opened;
      snprintf(response, capacity, "@COFLY\t%lu\tSD_BEGIN\n", (unsigned long)id);
    } else {
      source = opened;
      fileSize = source.size();
      offset = 0;
      fileCrc = 0xffffffff;
      snprintf(response, capacity, "@COFLY\t%lu\tSD_OPEN\t%llu\n", (unsigned long)id, (unsigned long long)fileSize);
    }
  } else if (!active) {
    error("SESSION_EXPIRED");
  } else if (!strcmp(tokens[1], "KEEP") && count == 3) {
    snprintf(response, capacity, "@COFLY\t%lu\tSD_KEEP\n", (unsigned long)id);
  } else if (!strcmp(tokens[1], "NEXT") && count == 3 && directory) {
    File entry = directory.openNextFile(FILE_READ);
    if (!entry) {
      snprintf(response, capacity, "@COFLY\t%lu\tSD_END\n", (unsigned long)id);
    } else {
      const char *name = entry.name();
      const size_t length = strlen(name);
      if (!length || length > 255) error("NAME_TOO_LONG");
      else {
        char encoded[511], timestamp[24] = "-";
        hex(reinterpret_cast<const uint8_t *>(name), length, encoded);
        DateTimeFields tm = {};
        if (entry.getModifyTime(tm))
          snprintf(timestamp, sizeof(timestamp), "%04u-%02u-%02uT%02u:%02u:%02u",
                   tm.year + 1900, tm.mon + 1, tm.mday, tm.hour, tm.min, tm.sec);
        snprintf(response, capacity, "@COFLY\t%lu\tSD_ENTRY\t%s\t%llu\t%s\t%s\n",
                 (unsigned long)id, encoded, (unsigned long long)entry.size(), timestamp,
                 entry.isDirectory() ? "DIR" : "FILE");
      }
      entry.close();
    }
  } else if (!strcmp(tokens[1], "READ") && (count == 4 || count == 5) && source) {
    uint64_t requested, blockSize = LEGACY_BLOCK_SIZE;
    if (!number(tokens[3], requested) || requested != offset) error("OFFSET");
    else if (count == 5 && (!number(tokens[4], blockSize) || !blockSize || blockSize > SD_FILE_BLOCK_SIZE)) error("BLOCK_SIZE");
    else if (capacity < blockSize * 2 + 128) error("RESPONSE_SIZE");
    else if (source.size() != fileSize) error("FILE_CHANGED");
    else if (offset == fileSize) {
      snprintf(response, capacity, "@COFLY\t%lu\tSD_EOF\t%llu\t%08lx\n", (unsigned long)id,
               (unsigned long long)fileSize, (unsigned long)(fileCrc ^ 0xffffffff));
    } else {
      const size_t amount = fileSize - offset < blockSize ? fileSize - offset : blockSize;
      if (!source.seek(offset) || source.read(block, amount) != amount) error("READ_FAILED");
      else {
        const int prefix = snprintf(response, capacity, "@COFLY\t%lu\tSD_DATA\t%llu\t",
                                    (unsigned long)id, (unsigned long long)offset);
        hex(block, amount, response + prefix);
        const size_t tail = prefix + amount * 2;
        snprintf(response + tail, capacity - tail, "\t%08lx\n",
                 (unsigned long)(crcUpdate(0xffffffff, block, amount) ^ 0xffffffff));
        fileCrc = crcUpdate(fileCrc, block, amount);
        offset += amount;
      }
    }
  } else error("SYNTAX");
  snprintf(lastCommand, sizeof(lastCommand), "%s", command);
  snprintf(lastResponse, sizeof(lastResponse), "%s", response);
}
