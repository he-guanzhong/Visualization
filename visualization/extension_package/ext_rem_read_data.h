#ifndef EXT_REM_READ_DATA_H_
#define EXT_REM_READ_DATA_H_

#include "visualization/extension_package/ext_rem_load_log.h"
#include "visualization/extension_package/ext_rem_tools.h"

#ifdef REM_DEMO_TEST
void ReadRemInputData(const int t,
                      MotionInfo* motionInfo,
                      LinesInfo* linesInfo,
                      RemPointsInfo* remPointsInfo);
#endif

#endif  // EXT_REM_READ_DATA_H_
