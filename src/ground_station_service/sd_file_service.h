#pragma once
#include <stddef.h>

constexpr size_t SD_FILE_BLOCK_SIZE = 4096;
constexpr size_t SD_FILE_RESPONSE_SIZE = SD_FILE_BLOCK_SIZE * 2 + 128;

// Read-only USB file session. The logger and deferred SD saves pause while active.
bool sdFileSessionActive();
void pollSdFileService(bool connected);
void handleSdFileCommand(char *line, char *response, size_t capacity);
