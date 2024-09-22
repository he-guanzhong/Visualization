#ifndef SHOW_AGSM_TOOLS_H_
#define SHOW_AGSM_TOOLS_H_
#include "visualization/show_basic_tools.h"

typedef struct {
  float c0;
  float c1;
  float c2;
  float c3;
  float start;
  float end;
} LaneMkr;

typedef struct {
  float c0;
  float c1;
  float c2;
  float c3_array[3];
  float segment_length_array[3];
  int valid;
} ConftPathTyp;

typedef struct {
  ConftPathTyp conft_path_record;
  LaneMkr LH0;
  LaneMkr LH1;
} AgsmLinesInfo;

void drawLaneMkr(const LaneMkr* path, const int color);

void drawConftPath(const ConftPathTyp* path,
                   const int color,
                   const float startX);

/// @brief AGSM graph, x-axis_forward_vertical, y-axis_lateral_horizontal
/// @param config         basic configuration
/// @param zeroOffsetX    distance behind ego vehicle to be displayed
/// @param ssmObjs        obstacles info
/// @param agsmlinesInfo  agsm lines info
/// @param motionInfo     ego spd, acc, etc..
void showAGSMGraph(const GraphConfig* config,
                   const float zeroOffsetX,
                   const SsmObjType* ssmObjs,
                   const AgsmLinesInfo* agsmlinesInfo,
                   const MotionInfo* motionInfo);

#endif  // SHOW_AGSM_TOOLS_H_
