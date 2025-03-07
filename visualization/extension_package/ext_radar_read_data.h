#ifndef EXT_RADAR_READ_DATA_H_
#define EXT_RADAR_READ_DATA_H_

#include "visualization/extension_package/ext_radar_load_log.h"
#include "visualization/extension_package/ext_radar_tools.h"

#ifdef AGSM_DEMO_TEST
void ReadAgsmInputData(const int t,
                       MotionInfo* motionInfo,
                       AgsmLinesInfo* agsmLinesInfo,
                       SsmObjType* ssmObjs);
#endif

#ifdef RADAR_DEMO_TEST
void ReadRadarInputData(const int t, RadarObjInfo* radarObjsInfo);
#endif

#ifdef MEOBJ_DEMO_TEST
void ReadMeObjInputData(const int t, MeObjInfo* meObjsInfo);
#endif

#endif  // EXT_RADAR_READ_DATA_H_
