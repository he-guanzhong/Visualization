#ifndef EXT_REM_TOOLS_H_
#define EXT_REM_TOOLS_H_

#include "visualization/show_basic_tools.h"

typedef struct {
  Point egoDpPoint[20];
  Point tarDpPoint[20];
  int nums;
} RemPointsInfo;

void drawDpPoints(const Point* pointsInfo, const int nums, const int color);

/// @brief REM graph, x-axis_forward_vertical, y-axis_lateral_horizontal
/// @param config         basic configuration
/// @param zeroOffsetX    distance behind ego vehicle to be displayed
/// @param linesInfo      all lane lines info
/// @param motionInfo     ego vehicle motion status
/// @param remPointsInfo  ego and tar dp points
void showRemGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const LinesInfo* linesInfo,
                  const MotionInfo* motionInfo,
                  const RemPointsInfo* remPointsInfo);

#endif  // EXT_REM_TOOLS_H_
