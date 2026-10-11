#pragma once
#include "parameters/parameter_registry.h"

// Shared text conversion for USB and persistent name:type=value records.
namespace ParameterRecord {
bool parseUnsigned(const char *text, uint32_t &value);
bool parseValue(const FlightParameter &p, const char *text, double &value);
void formatValue(const FlightParameter &p, double value, char *text, size_t capacity);
bool split(char *line, char *&name, char *&type, char *&value);
int encode(const FlightParameter &p, double value, char *out, size_t capacity);
} // namespace ParameterRecord
