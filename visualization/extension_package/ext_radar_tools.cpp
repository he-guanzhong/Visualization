
#include "visualization/extension_package/ext_radar_tools.h"

void drawLaneMkr(const LaneMkr* path, const int color) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  for (float x = path->start; x < path->end; x += 3.0f) {
    const float y =
        path->c0 + path->c1 * x + path->c2 * x * x + path->c3 * x * x * x;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != 0.0f) {
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    }
    lastDrawPoint = curDrawPoint;
  }
  return;
}

void drawConftPath(const ConftPathTyp* path,
                   const int color,
                   const float startX) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  float c0_arr[3] = {path->c0, 0, 0}, c1_arr[3] = {path->c1, 0, 0},
        c2_arr[3] = {path->c2, 0, 0},
        c3_arr[3] = {path->c3_array[0], path->c3_array[1], path->c3_array[2]};
  const float totalLen = path->segment_length_array[0] +
                         path->segment_length_array[1] +
                         path->segment_length_array[2];
  const float len_arr[3] = {path->segment_length_array[0],
                            path->segment_length_array[1],
                            path->segment_length_array[2]};
  for (int i = 0; i <= 1; i++) {
    c0_arr[i + 1] = c0_arr[i] + c1_arr[i] * len_arr[i] +
                    c2_arr[i] * len_arr[i] * len_arr[i] +
                    c3_arr[i] * len_arr[i] * len_arr[i] * len_arr[i];
    c1_arr[i + 1] = c1_arr[i] + 2 * c2_arr[i] * len_arr[i] +
                    3 * c3_arr[i] * len_arr[i] * len_arr[i];
    c2_arr[i + 1] = (2 * c2_arr[i] + 6 * c3_arr[i] * len_arr[i]) / 2.0f;
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
    if (lastDrawPoint.x != startX) {
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    }
    lastDrawPoint = curDrawPoint;
  }
  return;
}

void showAGSMGraph(const GraphConfig* config,
                   const float zeroOffsetX,
                   const SsmObjType* ssmObjs,
                   const AgsmLinesInfo* agsmlinesInfo,
                   const MotionInfo* motionInfo) {
  initBEVGraph(config, zeroOffsetX);

  // obstacles
  EgoPathVcc default_ego_path = {0};
  float LH0[8] = {0}, LH1[8] = {0}, ssmObjSpdY[10] = {0};
  drawObstacles(ssmObjs, &default_ego_path, LH0, LH1, motionInfo->egoSpd,
                ssmObjSpdY);

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
  strCompletion(str_ego, 14, motionInfo->egoSpd);
  drawCar(&ego, str_ego, 1, 5.0f, 1.8f, 0.0f, 14);

  // ego spd info and lane change status
  drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);
  return;
}

void drawRadarObj(const RadarObjInfo* radarInfo,
                  const int dir,
                  const int color) {
  for (int j = 0; j < 20; j++) {
    if (radarInfo->iObjectId[j] == 0 || radarInfo->fExistProb[j] < 0.1f) {
      continue;
    }
    Point obj_posn = {radarInfo->fDistX[j], radarInfo->fDistY[j]};
    char obj_id[10] = "";
    switch (dir) {
      case 0:
        strcpy(obj_id, "FL");
        break;
      case 1:
        strcpy(obj_id, "FR");
        break;
      case 2:
        strcpy(obj_id, "RL");
        break;
      case 3:
        strcpy(obj_id, "RR");
        break;
      default:
        break;
    }
    const int obj_len = strlen(obj_id);
    snprintf(obj_id + obj_len, sizeof(obj_id) - obj_len, "%d", j);
    coordinateTrans2(&obj_posn);

    setlinecolor(BLACK);
    setfillcolor(color);
    solidcircle(obj_posn.x, obj_posn.y, 5);
    const int off_disp = 10;
    if (0 == dir || 2 == dir) {
      outtextxy(obj_posn.x - off_disp - textwidth(obj_id),
                obj_posn.y - textheight(obj_id) / 2, obj_id);
    } else {
      outtextxy(obj_posn.x + off_disp, obj_posn.y - textheight(obj_id) / 2,
                obj_id);
    }
  }
  return;
}

void showRadarGraph(const GraphConfig* config,
                    const float zeroOffsetX,
                    const RadarObjInfo* radarObjsInfo) {
  initBEVGraph(config, zeroOffsetX);

  // ego car
  setfillcolor(LIGHTGRAY);
  Point ego = {0.0f, 0.0f};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  drawCar(&ego, str_ego, 1, 5.0f, 1.8f, 0.0f, 0);

  drawRadarObj(&radarObjsInfo[0], 0, RED);
  drawRadarObj(&radarObjsInfo[1], 1, BLUE);
  drawRadarObj(&radarObjsInfo[2], 2, MAGENTA);
  drawRadarObj(&radarObjsInfo[3], 3, GREEN);

  drawBEVRuler(zeroOffsetX);
  return;
}

void drawBox(Point* center, float len, float wid, float yaw) {
  // display: left-hand system. control: right-hand system
  // vertice order: upper-right -> lower-right-> lower-left -> upper-left
  const float halfLenCos = len / 2.0f * cosf(yaw),
              halfLenSin = len / 2.0f * sinf(yaw);
  const float halfWidCos = wid / 2.0f * cosf(yaw),
              halfWidSin = wid / 2.0f * sinf(yaw);
  Point vertices[4] = {{center->x + halfLenCos - halfWidSin,
                        center->y + halfLenSin + halfWidCos},
                       {center->x + halfLenCos + halfWidSin,
                        center->y + halfLenSin - halfWidCos},
                       {center->x - halfLenCos + halfWidSin,
                        center->y - halfLenSin - halfWidCos},
                       {center->x - halfLenCos - halfWidSin,
                        center->y - halfLenSin + halfWidCos}};
  coordinateTrans2(center);
  for (int i = 0; i < 4; ++i)
    coordinateTrans2(&vertices[i]);
  POINT vertices_show[4];
  for (int i = 0; i < 4; ++i) {
    vertices_show[i].x = vertices[i].x;
    vertices_show[i].y = vertices[i].y;
  }
  fillpolygon(vertices_show, 4);
  return;
}

void drawMeObj(const MeObjInfo* meInfo) {
  for (int j = 0; j < 12; ++j) {
    if (meInfo->iId[j] == 0 || meInfo->fLongDis[j] < 0) {
      continue;
    }
    Point obj_posn = {meInfo->fLongDis[j], meInfo->fLatDis[j]};
    char obj_id[10] = "";
    snprintf(obj_id, sizeof(obj_id), "%d", j + 1);
    const int obj_len = strlen(obj_id);
    snprintf(obj_id + obj_len, sizeof(obj_id) - obj_len, "-ID: %d",
             meInfo->iId[j]);
    char obj_spd[10] = "";
    snprintf(obj_spd, sizeof(obj_id), "%.1f m/s", meInfo->fLongSpd[j]);

    setlinecolor(BLACK);
    if (meInfo->iClass[j] == 1) {
      setfillcolor(BLUE);
    } else if (meInfo->iClass[j] == 2) {
      setfillcolor(CYAN);
    } else {
      setfillcolor(RED);
    }
    drawBox(&obj_posn, meInfo->fLen[j], meInfo->fWid[j], meInfo->fHeading[j]);

    const int off_disp = 20;
    if (meInfo->fLatDis[j] > 3.0f) {
      outtextxy(obj_posn.x - off_disp - textwidth(obj_id),
                obj_posn.y - textheight(obj_id), obj_id);
      outtextxy(obj_posn.x - off_disp - textwidth(obj_spd), obj_posn.y,
                obj_spd);
    } else if (meInfo->fLatDis[j] < -3.0f) {
      outtextxy(obj_posn.x + off_disp, obj_posn.y - textheight(obj_id), obj_id);
      outtextxy(obj_posn.x + off_disp, obj_posn.y, obj_spd);
    } else {
      outtextxy(obj_posn.x - textwidth(obj_id) / 2, obj_posn.y + off_disp,
                obj_id);
      outtextxy(obj_posn.x - textwidth(obj_spd) / 2,
                obj_posn.y + off_disp + textheight(obj_spd), obj_spd);
    }
  }
  return;
}

void showMeObjGraph(const GraphConfig* config,
                    const float zeroOffsetX,
                    const MeObjInfo* meObjsInfo) {
  initBEVGraph(config, zeroOffsetX);

  // ego car
  setfillcolor(LIGHTGRAY);
  Point ego = {0.0f, 0.0f};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  drawCar(&ego, str_ego, 1, 5.0f, 1.8f, 0.0f, 0);

  drawMeObj(meObjsInfo);

  drawBEVRuler(zeroOffsetX);
  return;
}
