#ifndef SHOW_MAIN_H_
#define SHOW_MAIN_H_

// for ordinary user or secondary developer, please assign it as 1
#define ORDINARY_USER 1
#if ORDINARY_USER == 0
#include "spdplanning/pa_speed_planning.h"
#endif

#include "visualization/show_basic_tools.h"
#include "visualization/show_load_log.h"

enum PLAYMODE {
  ONESTEP = 1,
  LOG = 2,
  LOOPBACK = 3,
  SIMULATION = 4,
  FUSION = 5
};

void ReadInputData(const int t);
void ReadOutputData(const int t);

void Time2Str(const float time, char* str);
void DisplayLog(const int length, const int width, const int offset);

#ifdef SPEED_PLANNING_H_
void CalcOneStep();
void DisplayOneStep(const int length, const int width, const int offset);

void LoopbackCalculation();
void GenerateLocalData();
#endif

BOOL GetFileFromUser(char* filePath, int size);
void ReleaseWrapper();

#endif  // SHOW_MAIN_H_