#ifndef SHOW_EXT_READ_DATA_H_
#define SHOW_EXT_READ_DATA_H_

#include "visualization/extension_package/show_ext_load_log.h"
#include "visualization/extension_package/show_ext_tools.h"

#ifdef AGSM_DEMO_TEST
void ReadAgsmInputData(const int t,
                       MotionInfo* motionInfo,
                       AgsmLinesInfo* agsmLinesInfo,
                       SsmObjType* ssmObjs);
#endif

#ifdef RADAR_DEMO_TEST
void ReadRadarInputData(const int t, RadarObjInfo* radarInfo);
#endif

#endif  // SHOW_EXT_READ_DATA_H_
