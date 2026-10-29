#pragma once
#include <algorithm>
#include <cmath>

namespace fleet_rviz_tools {
// Display-only voltage gauge, not a calibrated state-of-charge model.
// 3.50 V/cell is the displayed empty reference, NOT a discharge cutoff.
inline double voltagePercent(double volts, int cells)
{
  if (!std::isfinite(volts) || volts <= 0.0 || cells <= 0) return NAN;
  return std::clamp((volts / cells - 3.50) / (4.20 - 3.50) * 100.0, 0.0, 100.0);
}
}  // namespace fleet_rviz_tools
