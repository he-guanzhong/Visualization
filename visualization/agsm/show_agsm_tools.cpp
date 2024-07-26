
#include "visualization/agsm/show_agsm_tools.h"

void drawLaneMkr(const LaneMkr* path, const int color) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  for (float x = path->start; x < path->end; x += 3.0f) {
    float y = path->c0 + path->c1 * x + path->c2 * x * x + path->c3 * x * x * x;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != 0.0f)
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    lastDrawPoint = curDrawPoint;
  }
}

void drawConftPath(const ConftPathTyp* path,
                   const int color,
                   const float startX) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  float c0_arr[3] = {path->c0, 0, 0}, c1_arr[3] = {path->c1, 0, 0},
        c2_arr[3] = {path->c2, 0, 0},
        c3_arr[3] = {path->c3_array[0], path->c3_array[1], path->c3_array[2]};
  float totalLen = path->segment_length_array[0] +
                   path->segment_length_array[1] +
                   path->segment_length_array[2];
  float len_arr[3] = {path->segment_length_array[0],
                      path->segment_length_array[1],
                      path->segment_length_array[2]};
  for (int i = 0; i <= 1; i++) {
    c0_arr[i + 1] = c0_arr[i] + c1_arr[i] * len_arr[i] +
                    c2_arr[i] * len_arr[i] * len_arr[i] +
                    c3_arr[i] * len_arr[i] * len_arr[i] * len_arr[i];
    c1_arr[i + 1] = c1_arr[i] + 2 * c2_arr[i] * len_arr[i] +
                    3 * c3_arr[i] * len_arr[i] * len_arr[i];
    c2_arr[i + 1] = 2 * c2_arr[i] + 6 * c3_arr[i] * len_arr[i];
  }

  for (float x = startX; x < totalLen; x += 2.0f) {
    float x_prime = x, y = 0;
    float c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    if (x <= len_arr[0]) {
      setlinecolor(RED);
      c0 = c0_arr[0], c1 = c1_arr[0], c2 = c2_arr[0], c3 = c3_arr[0];
    } else if (x <= len_arr[0] + len_arr[1]) {
      setlinecolor(LIGHTBLUE);
      x_prime = x - len_arr[0];
      c0 = c0_arr[1], c1 = c1_arr[1], c2 = c2_arr[1], c3 = c3_arr[1];
    } else {
      setlinecolor(GREEN);
      x_prime = x - len_arr[0] - len_arr[1];
      c0 = c0_arr[2], c1 = c1_arr[2], c2 = c2_arr[2], c3 = c3_arr[2];
    }
    y = c0 + c1 * x_prime + c2 * x_prime * x_prime +
        c3 * x_prime * x_prime * x_prime;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != startX)
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    lastDrawPoint = curDrawPoint;
  }
}

void showAGSMGraph(const GraphConfig* config,
                   const float zeroOffsetX,
                   const SsmObjType* ssmObjs,
                   const AgsmLinesInfo* agsmlinesInfo,
                   const MotionInfo* motionInfo) {
  initBEVGraph(config, zeroOffsetX);

  // obstacles
  EgoPathVcc default_ego_path = {0};
  drawObstacles(ssmObjs, &default_ego_path, motionInfo->egoSpd);

  // ego lane path
  drawConftPath(&agsmlinesInfo->conft_path_record, MAGENTA, 0.0f);

  // ME original road lines
  drawLaneMkr(&agsmlinesInfo->LH0, LIGHTGRAY);
  drawLaneMkr(&agsmlinesInfo->LH1, LIGHTGRAY);

  // ego car
  setfillcolor(RED);
  setlinestyle(PS_SOLID);
  setlinecolor(RED);
  Point ego = {0.0f, 0};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  strCompletion(str_ego, 10, motionInfo->egoSpd);
  drawCar(&ego, str_ego, 1, 0, 10);

  // ego spd info and lane change status
  drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);
}
