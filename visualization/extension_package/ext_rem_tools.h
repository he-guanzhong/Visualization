#ifndef EXT_REM_TOOLS_H_
#define EXT_REM_TOOLS_H_

#include "visualization/show_basic_tools.h"

typedef struct {
  int point_nums;
  Point ego_dp_point[20];
  Point tar_dp_point[20];
  float ego_dp_org[8];
  float tar_dp_org[8];
  float ego_dp_off[8];
  float tar_dp_off[8];
} RemInfo;

void drawDpPoints(const Point* pointsInfo, const int nums, const int color);

void drawRemDpInfo(const GraphConfig* config,
                   const RemInfo* remInfo,
                   const int egoOrgColor,
                   const int tarOrgColor,
                   const int egoOffColor,
                   const int tarOffColor);

void drawRemRuler(const float zeroOffsetX);

/// @brief REM graph, x-axis_forward_vertical, y-axis_lateral_horizontal
/// @param config         basic configuration
/// @param zeroOffsetX    distance behind ego vehicle to be displayed
/// @param linesInfo      all lane lines info
/// @param motionInfo     ego vehicle motion status
/// @param remInfo        ego/tar dp lines and points
void showRemGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const LinesInfo* linesInfo,
                  const MotionInfo* motionInfo,
                  const RemInfo* remInfo);

#endif  // EXT_REM_TOOLS_H_
