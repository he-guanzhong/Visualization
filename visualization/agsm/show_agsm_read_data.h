#ifndef SHOW_AGSM_READ_DATA_H_
#define SHOW_AGSM_READ_DATA_H_

#include "visualization/agsm/show_agsm_load_log.h"
#include "visualization/agsm/show_agsm_tools.h"
#ifdef AGSM_LOCAL_TEST

void ReadAgsmInputData(const int t,
                       MotionInfo* motionInfo,
                       AgsmLinesInfo* agsmLinesInfo,
                       SsmObjType* ssmObjs);
#endif

#endif  // SHOW_AGSM_READ_DATA_H_
