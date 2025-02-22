#ifndef EXT_REM_TOOLS_H_
#define EXT_REM_TOOLS_H_

#include "visualization/extension_package/ext_rem_load_log.h"
#include "visualization/show_basic_tools.h"

typedef struct {
  int ego_point_nums;
  int tar_point_nums;
  Point ego_dp_point[30];
  Point tar_dp_point[30];
  float ego_dp_org[8];
  float tar_dp_org[8];
  float ego_dp_off[10];
  float tar_dp_off[10];
  int dp_usage[2];
  float special_point_distance[2];  // 0:merge 1:split
  int split_attribute[2];
  int forward_lanes[4];
  float left_lanemark[3];
  float right_lanemark[3];
} RemInfo;

void drawDpPoints(const Point* pointsInfo, const int nums, const int color);

void drawRemDpInfo(const GraphConfig* config,
                   const RemInfo* remInfo,
                   const int egoOrgColor,
                   const int tarOrgColor,
                   const int egoOffColor,
                   const int tarOffColor);

void drawRemRuler(const float zeroOffsetX,
                  const float rangeX,
                  const float rangeY);

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

void ShowRemInterface(const GraphConfig* config,
                      const float zeroOffsetX,
                      const LinesInfo* linesInfo,
                      const MotionInfo* motionInfo,
                      const RemInfo* remInfo,
                      const ExMessage* msg,
                      const float* time_data,
                      const int curFrame,
                      const int totalFrame);

#endif  // EXT_REM_TOOLS_H_
