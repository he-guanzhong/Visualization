#ifndef SHOW_BASIC_TOOLS_H_
#define SHOW_BASIC_TOOLS_H_

#ifdef _WIN32
#include <easyx.h>
#else
#error EasyX is only for Windows
#endif
#include <math.h>
#include <stdio.h>
#include "header/Rte_Type_Spd.h"

#define MAT_SIZE 6

typedef struct {
  float x;
  float y;
} Point;

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
  float cur_spd;
  float pred_spd;
  float set_spd;
  int acc_mode;
  int alc_side;
  int alc_sts;
} SpdInfo;

typedef struct {
  float* ego_coeffs;
  float* left_coeffs;
  float* leftleft_coeffs;
  float* right_coeffs;
  float* rightright_coeffs;
  float* left_coeffs_me;
  float* leftleft_coeffs_me;
  float* right_coeffs_me;
  float* rightright_coeffs_me;
} LinesInfo;

int combination(int n, int k);
void bezierPoint(float tau, int n, float points[][2], float* x, float* y);
void bezierDerivative(float tau,
                      int n,
                      float points[][2],
                      float* dx,
                      float* dy);
void bezierSecDerivative(float tau,
                         int n,
                         float points[][2],
                         float* ddx,
                         float* ddy);

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]);
void quinticPolyFit(float T,
                    float s0,
                    float v0,
                    float a0,
                    float s1,
                    float v1,
                    float a1,
                    float coeffi[MAT_SIZE]);

void coordinateTrans1(Point* point);
void coordinateTrans2(Point* point);

void strCompletion(char str[2][8], const int index, const int spd);
void drawCar(Point* car,
             const char str[2][8],
             int carType,
             const float yaw,
             const int index);

void drawPolygon(const Point* center, const int num, const float rotateDegree);
void drawTsrSign(const TsrInfo* tsr_info);
void drawMotionInfo(const SpdInfo* spd_info);

void drawTrajectory(const float* coeffs,
                    const int color,
                    const float startX,
                    const float lengthS,
                    Point* predictPosn);

void drawBasicGraph(const int len,
                    const int wid,
                    const float rangeX,
                    const float rangeY,
                    const float offsetY);

void drawBEVRuler();

void showSTGraph(const int length,
                 const int width,
                 const int offset,
                 const int oriX,
                 const int oriY,
                 const float zeroOffsetY,
                 const float rangeX,
                 const float rangeY,
                 const char* title,
                 const int pointColor,
                 Point* points,
                 Point* ctrlPoint);

void showBEVGraph(const int length,
                  const int width,
                  const int offset,
                  const int oriX,
                  const int oriY,
                  const float zeroOffsetX,
                  const float rangeX,
                  const TsrInfo* tsr_info,
                  const SsmFrameType* g_ssmFrameType,
                  const LinesInfo* lines_info,
                  const SpdInfo* spd_info);

bool inArea(int mx, int my, int x, int y, int w, int h);
bool button(ExMessage* msg, int x, int y, int w, int h, bool* swt);
bool functionButton(ExMessage msg);
void keyboardTest();

#endif  // SHOW_BASIC_TOOLS_H_