#ifndef SHOW_BASIC_TOOLS_H_
#define SHOW_BASIC_TOOLS_H_

#ifdef _WIN32
#include <easyx.h>
#else
#error EasyX is only for Windows
#endif
#include <stdio.h>
#include "header/Rte_Type_Spd.h"
#include "visualization/show_math.h"

typedef struct {
  float x;
  float y;
} Point;

typedef struct {
  int length;    // plot area length
  int width;     // plot area width
  int offset;    // distance between graph and plot area boundary
  int oriX;      // plot area origin x in overall canvas (LH coordinate)
  int oriY;      // plot area origin y in overall canvas (LH coordinate)
  float rangeX;  // display range of x-axis
  float rangeY;  // display range of y-axis
} GraphConfig;

typedef struct {
  char* title;
  Point* points;     // point coordinates to be displayed
  Point* ctrlPoint;  // a-t graph only, accelerations sent to control
  int startIndex;
  int pointNums;
  int pointColor;
  bool showPoly;
  float* quinticPoly;
} PlotInfo;

typedef struct {
  bool valid;
  int type;
  float pos_x;
  float pos_y;
} TsrSign;

typedef struct {
  int tsr_spd;
  bool tsr_spd_warn;
  int tsr_tsi[2];
  TsrSign tsr_signs[3];
} TsrInfo;

typedef struct {
  float egoSpd;
  float egoAcc;
  float spdLmt;
  int accMode;
  int tauGap;
  int enblSts;
  float egoPredSpd;
  float innerSpdLmt;
  int specCaseFlg;
  int scenarioFlg;
  int gapIndex;
  float gapTarS;
  float gapTarV;
  AlcBehavior alcBehav;
} MotionInfo;

typedef struct {
  float alc_coeffs[8];
  EgoPathVcc ego_coeffs;
  float left_coeffs[8];
  float leftleft_coeffs[8];
  float right_coeffs[8];
  float rightright_coeffs[8];
  float ego_dp[8];
  float tar_dp[8];
} LinesInfo;

void coordinateTrans1(Point* point);
void coordinateTrans2(Point* point);

void strCompletion(char str[2][8], const int index, const int spd);
float getCubicPolynomial(const float x, const float* line);
float getPiecewiseCubicPolynomial(const float x, const EgoPathVcc* egoPath);

void drawCar(Point* car,
             const char str[2][8],
             int carType,
             const float yaw,
             const int index);

void drawPolygon(const Point* center, const int num, const float rotateDegree);
void drawTsrSign(const TsrInfo* tsrInfo);
void drawMotionInfo(const MotionInfo* motionInfo);

void drawQuinticPolyTraj(const float* coeffs,
                         const int color,
                         const float startX,
                         const float lengthX,
                         const float lengthS,
                         Point* predictPosn);

void drawPiecewiseCubicPolyTraj(const EgoPathVcc* egoPath,
                                const int color,
                                const float startX,
                                Point* predictPosn);

void drawBasicGraph(const int len,
                    const int wid,
                    const float rangeX,
                    const float rangeY,
                    const float offsetY);

void drawObstacles(const SsmObjType* ssmObjs,
                   const EgoPathVcc* egoPath,
                   const float* LH0,
                   const float* LH1,
                   const float cur_spd);

void drawBEVRuler(const float zeroOffsetX);

void initBEVGraph(const GraphConfig* config, const float zeroOffsetX);

/// @brief  Show basic x-y graph
/// @param config         basic graph configuration
/// @param zeroOffsetY    vertical distance from origin to lower-left corner
/// @param plot           scatter plot data to be displayed
void showXYGraph(const GraphConfig* config,
                 const float zeroOffsetY,
                 const PlotInfo* plot);

/// @brief BEV graph, x-axis_forward_vertical, y-axis_lateral_horizontal
/// @param config         basic configuration
/// @param zeroOffsetX    distance behind ego vehicle to be displayed
/// @param ssmObjs        obstacles info
/// @param linesInfo      lane lines info
/// @param tsrInfo        environmental TSR info and ego TSR status
/// @param motionInfo     ego vehicle motion status
void showBEVGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const SsmObjType* ssmObjs,
                  const LinesInfo* linesInfo,
                  const TsrInfo* tsrInfo,
                  const MotionInfo* motionInfo);

bool inArea(int mx, int my, int x, int y, int w, int h);
bool buttonShowPred(ExMessage* msg, int x, int y, int w, int h, bool* swt);
bool buttonOneStep(ExMessage* msg,
                   int x,
                   int y,
                   int w,
                   int h,
                   const char* text);
int functionButton(ExMessage msg);
void keyboardTest();

#endif  // SHOW_BASIC_TOOLS_H_
