#ifndef SHOW_MAIN_H_
#define SHOW_MAIN_H_

// ordinary user: only replay function provided
#ifdef SPDPLAN_LOCAL_TEST
#include "spdplanning/pa_speed_planning.h"
#endif

#include "visualization/show_basic_tools.h"
#include "visualization/show_load_log.h"

enum PLAYMODE {
  ONESTEP = 1,
  LOG = 2,
  LOOPBACK = 3,
  SIMULATION = 4,
  FUSION = 5,
  LINECHART = 6
};

void ReadInputData(const int t);
void ReadOutputData(const int t);

void Time2Str(const float time, char* str);
void DisplayLog(const int length, const int width, const int offset);

#ifdef SPEED_PLANNING_H_
void CalcOneStep();
void DisplayOneStep(const int length, const int width, const int offset);

void LoopbackCalculation();

void LocalDummySsmData(SsmObjType* ssmObjs);
void GenerateLocalData();

void DisplayLineChart(const int length, const int width, const int offset);
#endif

BOOL GetFileFromUser(char* filePath, int size);
void ReleaseWrapper();

#endif  // SHOW_MAIN_H_