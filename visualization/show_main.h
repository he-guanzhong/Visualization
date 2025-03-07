#ifndef SHOW_MAIN_H_
#define SHOW_MAIN_H_

// ordinary user: only replay function provided
#ifdef SPDPLAN_LOCAL_TEST
#include "spdplanning/pa_speed_planning.h"
#include "visualization/show_test_cases.h"
#endif

#include "visualization/extension_package/ext_radar_read_data.h"
#include "visualization/extension_package/ext_rem_read_data.h"
#include "visualization/show_basic_tools.h"
#include "visualization/show_load_log.h"

typedef enum {
  ONESTEP = 1,
  LOG = 2,
  LOOPBACK = 3,
  SIMULATION = 4,
  LINECHART = 5,
  AGSM = 6,
  RADAR = 7,
  REM = 8,
  MEOBJ = 9
} PLAYMODE;

void ReadInputData(const int t);
void ReadOutputData(const int t);
void WriteOutputData(const int t);

void ConvertMotionInfo();
void Time2Str(const float time, char* str, const int strSize);

void ShowOutputKeyInfo(const int posY);
void ShowBasicFrameInfo(int* t, int* cycle, const int length, const int width);
void ShowSpdPlanInterface(const int length, const int width, const int offset);

void DisplaySpdPlanLineChart(const int length,
                             const int width,
                             const int offset,
                             const int oriX,
                             const int oriY,
                             const int curFrame,
                             const int winFrames);

void DisplayLog(const int length, const int width, const int offset);

#ifdef SPEED_PLANNING_H_
void ExecuteSpdPlan(const AlcPathVcc* alcPathVcc,
                    const AgsmEnvModel* agsmEnvModel,
                    const SsmObjType* ssmObjs);
void PrintOutputInfo(const DpSpeedPoints* output);

void CalcOneStep();

void DisplayOneStep(const int length, const int width, const int offset);

void LoopbackCalculation();

void GenerateLocalData();

void DisplayLoopbackCurve(const int length, const int width, const int offset);
#endif

void ReleaseWrapper(int length, int width, int offset);

#endif  // SHOW_MAIN_H_
