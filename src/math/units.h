#pragma once

// Shared physical state uses SI. Legacy controller parameters, sensor registers,
// and wire formats are converted only at their boundaries.
namespace Units {
constexpr float radiansPerDegree = 0.01745329251994329577f;
constexpr float degreesPerRadian = 57.29577951308232088f;
constexpr float standardGravity = 9.80665f;
constexpr float toRadians(float degrees) { return degrees * radiansPerDegree; }
constexpr float toDegrees(float radians) { return radians * degreesPerRadian; }
constexpr float acceleration(float gravityUnits) { return gravityUnits * standardGravity; }
constexpr float gravityUnits(float acceleration) { return acceleration / standardGravity; }
} // namespace Units
