#include "visualization/show_basic_tools.h"

#define CURVE_FITTING_TYPE 2
bool show_predict_swt = false;
// float fit_coeffi[6] = {0};

// Draw graph origins
static Point s_origin1 = {0.0f, 0.0f};
static float s_xScale1 = 0, s_yScale1 = 0;
static Point s_origin2 = {0.0f, 0.0f};
static float s_xScale2 = 0, s_yScale2 = 0;
static int s_infoAreaBoundary;

const static float car_len_tbl[7] = {
    2.2f + 0.4f, 5.0f + 0.4f, 10.0f + 0.4f, 2.5f, 2.5f, 1.0f, 2.0f};
const static float car_wid_tbl[7] = {
    1.0f + 0.4f, 1.5f + 0.4f, 1.6f + 0.4f, 0.6f, 0.6f, 1.0f, 2.0f};
const static int tsr_spd_table1[14] = {10, 20, 30,  40,  50,  60,  70,
                                       80, 90, 100, 110, 120, 130, 140};
const static int tsr_spd_table2[15] = {5,  15, 25,  35,  45,  55,  65, 75,
                                       85, 95, 105, 115, 125, 135, 145};

void coordinateTrans1(Point* point) {
  point->x = s_origin1.x + point->x * s_xScale1;
  point->y = s_origin1.y - point->y * s_yScale1;
}

void coordinateTrans2(Point* point) {
  float tmp = point->x;
  point->x = s_origin2.x - point->y * s_xScale2;
  point->y = s_origin2.y - tmp * s_yScale2;
}

void strCompletion(char str[2][8], const int index, const int spd) {
  const char* obsName[11] = {"IV ",  "RIV ",  "NIVL", "NIIVL", "RIVL", "RIIVL",
                             "NIVR", "NIIVR", "RIVR", "RIIVR", "ego "};
  strcpy(str[0], obsName[index]);
  char szSpd[3];
  itoa(spd, szSpd, 10);
  strcpy(str[1], szSpd);
  strcat(str[1], " m/s");
}

void drawCar(Point* car,
             const char str[2][8],
             int carType,
             const float yaw,
             const int index) {
  // ObjType of ME: 0=UNFILLED, 1=CAR, 2=TRUCK, 3=MOTORBIKE, 4=BICYCLE,
  // 5=PEDESTRIAN, 6=GENERAL_OBJECT, 7=ANIMAL 8=UNCERTAIN_VCL
  if (carType >= 7)
    carType = 0;

  float carLen = car_len_tbl[carType];
  float carWid = car_wid_tbl[carType];
  // display: left-hand system. control: right-hand system
  // vertice order: upper-right -> lower-right-> lower-left -> upper-left
  float halfLenCos = carLen / 2.0f * cosf(yaw),
        halfLenSin = carLen / 2.0f * sinf(yaw);
  float halfWidCos = carWid / 2.0f * cosf(yaw),
        halfWidSin = carWid / 2.0f * sinf(yaw);
  Point vertices[4] = {
      {car->x + halfLenCos - halfWidSin, car->y + halfLenSin + halfWidCos},
      {car->x + halfLenCos + halfWidSin, car->y + halfLenSin - halfWidCos},
      {car->x - halfLenCos + halfWidSin, car->y - halfLenSin - halfWidCos},
      {car->x - halfLenCos - halfWidSin, car->y - halfLenSin + halfWidCos}};
  coordinateTrans2(car);
  for (int i = 0; i < 4; i++)
    coordinateTrans2(&vertices[i]);
  POINT vertices_show[4];
  for (int i = 0; i < 4; i++) {
    vertices_show[i].x = vertices[i].x;
    vertices_show[i].y = vertices[i].y;
  }

  fillpolygon(vertices_show, 4);

  if (carType >= 3 && carType <= 6) {  // motorbike/pedestrian outline
    rectangle(car->x - car_wid_tbl[1] / 2.0f * s_xScale2,
              car->y - car_len_tbl[1] / 2.0f * s_yScale2,
              car->x + car_wid_tbl[1] / 2.0f * s_xScale2,
              car->y + car_len_tbl[1] / 2.0f * s_yScale2);
  }
  if (carType == 5) {
    solidcircle(car->x, car->y, 5);
  }

  if (index == 0 || index == 1 || index == 10) {
    outtextxy(car->x - textwidth(str[0]), car->y + textheight(str[0]) / 2,
              str[0]);
    outtextxy(car->x, car->y + textheight(str[1]) / 2, str[1]);
  } else if (index <= 5) {
    outtextxy(car->x - 20 - textwidth(str[0]), car->y - textheight(str[0]),
              str[0]);
    outtextxy(car->x - 20 - textwidth(str[1]), car->y, str[1]);
  } else {
    outtextxy(car->x + 20, car->y - textheight(str[0]), str[0]);
    outtextxy(car->x + 20, car->y, str[1]);
  }
}

void drawPolygon(const Point* center, const int num, const float rotateDegree) {
  int r = 15;
  int vertex_x = r, vertex_y = 0;
  POINT vertices_show[num];
  for (int i = 0; i < num; i++) {
    vertex_x = center->x + r * cosf(2 * M_PI / num * i + rotateDegree);
    vertex_y = center->y + r * sinf(2 * M_PI / num * i + rotateDegree);
    vertices_show[i] = {vertex_x, vertex_y};
  }
  polygon(vertices_show, num);
}

void drawTsrSign(const TsrInfo* tsrInfo) {
  // TSR status display
  char tsr_disp[10] = "TSR: ";
  char spd_val[10];
  int tsr_spd = tsrInfo->tsr_spd;
  bool tsr_spd_warn = tsrInfo->tsr_spd_warn;
  itoa(tsr_spd, spd_val, 10);
  strcat(tsr_disp, spd_val);
  char tsi_disp[10];
  if (tsrInfo->tsr_tsi[0] == 5)
    strcpy(tsi_disp, "stop");
  else if (tsrInfo->tsr_tsi[0] == 6)
    strcpy(tsi_disp, "yield");
  else if (tsrInfo->tsr_tsi[1] == 5)
    strcpy(tsi_disp, "no entry");
  if (tsr_spd_warn)
    settextcolor(RED);
  outtextxy(s_infoAreaBoundary, s_origin2.y, tsr_disp);
  settextcolor(BLACK);
  outtextxy(s_infoAreaBoundary, s_origin2.y + textheight(tsr_disp), tsi_disp);

  // TSR original input

  for (int i = 0; i < 3; i++) {
    if (!tsrInfo->tsr_signs[i].valid || tsrInfo->tsr_signs[i].type < 0 ||
        tsrInfo->tsr_signs[i].type > 210)
      continue;
    Point tsr_pos = {tsrInfo->tsr_signs[i].pos_x, tsrInfo->tsr_signs[i].pos_y};
    coordinateTrans2(&tsr_pos);

    // TSI (traffic sign indication). 210 = stop, 168 = yield, 199 = no entry
    if (tsrInfo->tsr_signs[i].type == 168) {
      drawPolygon(&tsr_pos, 3, M_PI / 2);
    } else if (tsrInfo->tsr_signs[i].type == 210) {
      drawPolygon(&tsr_pos, 8, M_PI / 8);
      char stop_sign[] = "stop";
      outtextxy(tsr_pos.x - textwidth(stop_sign) / 2,
                tsr_pos.y - textheight(stop_sign) / 2, stop_sign);
    } else if (tsrInfo->tsr_signs[i].type == 199) {
      setfillcolor(RED);
      solidcircle(tsr_pos.x, tsr_pos.y, 14);
      setfillcolor(WHITE);
      solidrectangle(tsr_pos.x - 8, tsr_pos.y - 3, tsr_pos.x + 8,
                     tsr_pos.y + 3);
    }

    // TSR Speed limit: 10~140, 5~145, cancel sign
    char tsr_sign[4];
    if (tsrInfo->tsr_signs[i].type <= 13) {
      itoa(tsr_spd_table1[tsrInfo->tsr_signs[i].type], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsrInfo->tsr_signs[i].type >= 100 &&
               tsrInfo->tsr_signs[i].type <= 114) {
      itoa(tsr_spd_table2[tsrInfo->tsr_signs[i].type - 100], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsrInfo->tsr_signs[i].type == 79 ||
               tsrInfo->tsr_signs[i].type == 80) {
      strcpy(tsr_sign, "Cnl");
      int r = textheight(tsr_sign) * 0.75f;
      int r_x = (double)r / sqrt(2);
      setlinecolor(BLACK);
      line(tsr_pos.x - r_x, tsr_pos.y + r_x, tsr_pos.x + r_x, tsr_pos.y - r_x);
      circle(tsr_pos.x, tsr_pos.y, r);
    }
  }
}

void drawMotionInfo(const MotionInfo* motionInfo) {
  // set spd display
  char spd_title[10] = "Spd: ";
  char set_title[10] = "SET: ";
  char inner_set_title[10] = "Inn: ";
  char spec_case_title[10] = "Spc: ";
  char scenario_title[10] = "Sce: ";

  char str_cur_spd[5];
  char str_disp_set_spd[5];
  char str_inner_spd_lmt[5];

  int cur_spd = round(motionInfo->egoSpd * 1.03f * 3.6f);
  int spd_lmt = round(motionInfo->spdLmt);
  int inner_spd_lmt = round(motionInfo->innerSpdLmt);

  itoa(cur_spd, str_cur_spd, 10);
  itoa(spd_lmt, str_disp_set_spd, 10);
  itoa(inner_spd_lmt, str_inner_spd_lmt, 10);
  strcat(spd_title, str_cur_spd);
  strcat(set_title, str_disp_set_spd);
  strcat(inner_set_title, str_inner_spd_lmt);

  // spd plan inner value
  switch (motionInfo->specCaseFlg) {
    case 1:
    case 11:
      strcat(spec_case_title, "v-30");
      break;
    case 2:
    case 12:
      strcat(spec_case_title, "a-1");
      break;
    case 3:
    case 13:
      strcat(spec_case_title, "cut");
      break;
    default:
      break;
  }

  switch (motionInfo->scenarioFlg) {
    case 3:
    case 13:
      strcat(scenario_title, "curv");
      break;
    case 10:
      strcat(scenario_title, "Ndg");
      break;
    case 4:
    case 14:
      strcat(scenario_title, "ramp");
      break;
    case 5:
      strcat(scenario_title, "dec");
      break;
    case 15:
      strcat(scenario_title, "decN");
      break;
    case 6:
      strcat(scenario_title, "hold");
      break;
    case 16:
      strcat(scenario_title, "hldN");
      break;
    case 7:
      strcat(scenario_title, "acc");
      break;
    case 17:
      strcat(scenario_title, "accN");
      break;
    case 8:
      strcat(scenario_title, "gapf");
      break;
    case 18:
      strcat(scenario_title, "gapfN");
      break;
    case 9:
      strcat(scenario_title, "gap");
      break;
    case 19:
      strcat(scenario_title, "gapN");
      break;
    default:
      break;
  }
  outtextxy(s_infoAreaBoundary, s_origin2.y - 7 * textheight(spd_title),
            inner_set_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - 6 * textheight(spd_title),
            spec_case_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - 5 * textheight(spd_title),
            scenario_title);

  // ACC mode: 3-stand still, 4-stand active, 5-active, 6-override
  if (motionInfo->accMode <= 5 && motionInfo->accMode >= 3)
    settextcolor(GREEN);
  else if (motionInfo->accMode == 6)
    settextcolor(RED);
  else
    settextcolor(BLACK);

  outtextxy(s_infoAreaBoundary, s_origin2.y - textheight(spd_title), spd_title);
  if (motionInfo->accMode <= 6 && motionInfo->accMode >= 3) {
    outtextxy(s_infoAreaBoundary, s_origin2.y - 2 * textheight(spd_title),
              set_title);
  }

  char alc_side[8];
  memset(alc_side, '\0', sizeof(alc_side));
  if (motionInfo->alcBehav.AutoLaneChgSide == 1)
    strcpy(alc_side, "ALC_L");
  else if (motionInfo->alcBehav.AutoLaneChgSide == 2)
    strcpy(alc_side, "ALC_R");
  // alc sts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving,
  // 4-in target line, 5-finished,6-Back to Ego, 8-takeover, 9-popMsgReq
  char alc_sts[12];
  memset(alc_sts, '\0', sizeof(alc_sts));
  if (motionInfo->alcBehav.AutoLaneChgSts == 2)
    strcpy(alc_sts, "Hold");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 3)
    strcpy(alc_sts, "Chg1");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 4)
    strcpy(alc_sts, "Chg2");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 6)
    strcpy(alc_sts, "Back");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 8)
    strcpy(alc_sts, "Takeover");
  if (motionInfo->alcBehav.AutoLaneChgSide == 1 ||
      motionInfo->alcBehav.AutoLaneChgSide == 2) {
    outtextxy(s_infoAreaBoundary, s_origin2.y - 200, alc_side);
    outtextxy(s_infoAreaBoundary, s_origin2.y - 200 + textheight(alc_side),
              alc_sts);
  }
}

void drawQuinticPolyTraj(const float* coeffs,
                         const int color,
                         const float startX,
                         const float lengthX,
                         const float lengthS,
                         Point* predictPosn) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  Point last = {0.0f, 0.0f};
  float restLen = lengthS;
  for (float x = startX; x < lengthX || restLen > 0; x += 3.0f) {
    float y = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x +
              coeffs[3] * x * x * x + coeffs[4] * x * x * x * x +
              coeffs[5] * x * x * x * x * x;
    if (restLen > 0) {
      if (last.x != 0.0f) {
        float delta_s = hypotf(x - last.x, y - last.y);
        restLen -= delta_s;
      }
      last.x = x, last.y = y;
    }
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != 0.0f && x < lengthX)
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    lastDrawPoint = curDrawPoint;
  }
  predictPosn->x = last.x;
  predictPosn->y = last.y;
}

void drawPiecewiseCubicPolyTraj(const EgoPathVcc* egoPath,
                                const int color,
                                const float startX,
                                Point* predictPosn) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  Point last = {0.0f, 0.0f};

  float totalLen = egoPath->Len[0] + egoPath->Len[1] + egoPath->Len[2];
  for (float x = startX; x < totalLen; x += 3.0f) {
    float x_prime = x, y = 0;
    float c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    if (x <= egoPath->Len[0]) {
      c0 = egoPath->C0[0], c1 = egoPath->C1[0], c2 = egoPath->C2[0],
      c3 = egoPath->C3[0];
    } else if (x <= egoPath->Len[0] + egoPath->Len[1]) {
      x_prime = x - egoPath->Len[0];
      c0 = egoPath->C0[1], c1 = egoPath->C1[1], c2 = egoPath->C2[1],
      c3 = egoPath->C3[1];
    } else {
      x_prime = x - egoPath->Len[0] - egoPath->Len[1];
      c0 = egoPath->C0[2], c1 = egoPath->C1[2], c2 = egoPath->C2[2],
      c3 = egoPath->C3[2];
    }
    y = c0 + c1 * x_prime + c2 * x_prime * x_prime +
        c3 * x_prime * x_prime * x_prime;
    last.x = x, last.y = y;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != startX)
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    lastDrawPoint = curDrawPoint;
  }

  predictPosn->x = last.x;
  predictPosn->y = last.y;
}

void drawObstacles(const SsmObjType* ssmObjs,
                   const EgoPathVcc* egoPath,
                   const float cur_spd) {
  for (int i = 0; i < ssmObjs->obj_num; i++) {
    if (!ssmObjs->obj_lists[i].valid_flag)
      continue;

    const SsmObsType* obs = &ssmObjs->obj_lists[i];
    Point obs_cur = {obs->pos_x, obs->pos_y};
    char str_obs_cur[2][8] = {};
    strCompletion(str_obs_cur, i, obs->speed_x);
    setlinecolor(BLACK);
    setfillcolor(DARKGRAY);
    drawCar(&obs_cur, str_obs_cur, obs->type, obs->pos_yaw, i);

    // obs latspd not stable, use ego centre line offset
    Point obs_pred, obs_pred_path[6];
    float objPosnLgt[6], objPosnLat[6];

    for (int j = 0; j < 6; j++) {
      obs_pred_path[j].x = obs->pos_x + obs->speed_x * j;
      float predLatOffset = obs->speed_y * j;
      if (cur_spd > 40.0f / 3.6f && obs->speed_x > 1.0f) {
        objPosnLgt[j] = obs->pos_x + obs->speed_x * j;

        float x_prime = objPosnLgt[j];
        float c0 = 0, c1 = 0, c2 = 0, c3 = 0;
        if (objPosnLgt[j] <= egoPath->Len[0]) {
          c0 = egoPath->C0[0], c1 = egoPath->C1[0], c2 = egoPath->C2[0],
          c3 = egoPath->C3[0];
        } else if (objPosnLgt[j] <= egoPath->Len[0] + egoPath->Len[1]) {
          x_prime = objPosnLgt[j] - egoPath->Len[0];
          c0 = egoPath->C0[1], c1 = egoPath->C1[1], c2 = egoPath->C2[1],
          c3 = egoPath->C3[1];
        } else {
          x_prime = objPosnLgt[j] - egoPath->Len[0] - egoPath->Len[1];
          c0 = egoPath->C0[2], c1 = egoPath->C1[2], c2 = egoPath->C2[2],
          c3 = egoPath->C3[2];
        }
        objPosnLat[j] = c0 + c1 * x_prime + c2 * x_prime * x_prime +
                        c3 * x_prime * x_prime * x_prime;

        float roadCurveOffset = objPosnLat[j] - objPosnLat[0];

        if (fabsf(roadCurveOffset) > fabsf(predLatOffset))
          predLatOffset = roadCurveOffset;
      }
      obs_pred_path[j].y = obs->pos_y + predLatOffset;
    }

    obs_pred = obs_pred_path[5];
    // cipv, considier 0->1s const acc, 1->5s const spd
    if (obs->lane_index == 3 && obs->pos_x > 0) {
      float const_acc_time = 1.0f;
      obs_pred.x = obs->pos_x +
                   (obs->speed_x + obs->acc_x * const_acc_time) * 5.0f -
                   0.5f * obs->acc_x * const_acc_time * const_acc_time;
      obs_pred.x = fmax(obs->pos_x, obs_pred.x);
    }

    if (show_predict_swt && fabsf(obs_pred.x - obs->pos_x) > 5.0f) {
      char str_obs_pred[2][8] = {};
      strcpy(str_obs_pred[1], "Pred");
      setfillcolor(LIGHTGRAY);
      strCompletion(str_obs_pred, i, obs->speed_x);
      drawCar(&obs_pred, str_obs_pred, obs->type, obs->pos_yaw, i);
      setlinecolor(LIGHTGRAY);
      setlinestyle(PS_DASH);

      coordinateTrans2(&obs_pred_path[0]);
      for (int j = 0; j < 5; j++) {
        coordinateTrans2(&obs_pred_path[j + 1]);
        line(obs_pred_path[j].x, obs_pred_path[j].y, obs_pred_path[j + 1].x,
             obs_pred_path[j + 1].y);
      }
    }
  }
}

void drawBEVRuler(const float zeroOffsetX) {
  settextcolor(BLACK);
  float coordinate_x_of_ruler_y = fmin(-6.0f, -zeroOffsetX);
  Point ruler_x[4] = {
      {-30.0f, -6.0f}, {0.0f, -6.0f}, {50.0f, -6.0f}, {100.0f, -6.0f}};
  Point ruler_y[2] = {{coordinate_x_of_ruler_y, -5.0f},
                      {coordinate_x_of_ruler_y, 5.0f}};
  char str[7];
  for (int i = 0; i < 4; i++) {
    str[0] = '\0';
    itoa(ruler_x[i].x, str, 10);
    strcat(str, " m");
    coordinateTrans2(&ruler_x[i]);
    line(ruler_x[i].x, ruler_x[i].y, ruler_x[i].x - 10, ruler_x[i].y);
    outtextxy(ruler_x[i].x, ruler_x[i].y - textheight(str) / 2, str);
  }
  for (int i = 0; i < 2; i++) {
    str[0] = '\0';
    itoa(ruler_y[i].y, str, 10);
    strcat(str, " m");
    coordinateTrans2(&ruler_y[i]);
    line(ruler_y[i].x, ruler_y[i].y, ruler_y[i].x, ruler_y[i].y + 10);
    outtextxy(ruler_y[i].x - textwidth(str) / 2, ruler_y[i].y + 10, str);
  }
}

void drawBasicGraph(const int len,
                    const int wid,
                    const float rangeX,
                    const float rangeY,
                    const float offsetY) {
  // boundary
  setlinecolor(BLACK);
  rectangle(s_origin1.x, s_origin1.y, s_origin1.x + len, s_origin1.y - wid);

  setlinecolor(LIGHTGRAY);
  settextcolor(BLACK);
  settextstyle(20, 0, "Calibri");
  // horizontal lines
  float intervalX = rangeX / 5.0f, intervalY = rangeY / 6.0f;
  for (int i = 1; i <= 6; i++) {
    Point point1 = {0.0f, i * intervalY};
    Point point2 = {rangeX, i * intervalY};
    coordinateTrans1(&point1);
    coordinateTrans1(&point2);
    line(point1.x, point1.y, point2.x, point2.y);
    point1 = {0.0f, i * intervalY};
    char str[4] = {0};
    itoa(i * intervalY - offsetY, str, 10);
    coordinateTrans1(&point1);
    outtextxy(point1.x - textwidth(str) - textwidth(" "),
              point1.y - textheight(str) / 2, str);
  }
  // vertical lines
  for (int i = 1; i <= 5; i++) {
    Point point1 = {i * intervalX, 0.0f};
    Point point2 = {i * intervalX, rangeY};
    coordinateTrans1(&point1);
    coordinateTrans1(&point2);
    line(point1.x, point1.y, point2.x, point2.y);
    point1 = {i * intervalX, 0.0f};
    char str[4] = {0};
    itoa(i * intervalX, str, 10);
    coordinateTrans1(&point1);
    outtextxy(point1.x - textwidth(str) / 2, point1.y, str);
  }
}

void showXYGraph(const GraphConfig* config,
                 const float zeroOffsetY,
                 const char* title,
                 const int pointColor,
                 Point* points,
                 const int startIndex,
                 const int pointNums,
                 Point* ctrlPoint,
                 const float quinticPoly[6]) {
  // float zeroOffsetY = 4.0f;  // positive means: 0 moves up
  float len = config->length - 2 * config->offset;
  float wid = config->width - 2 * config->offset;

  // x = horizental, y = vertical
  s_origin1 = {(float)(config->oriX + config->offset),
               (float)(config->oriY + config->offset + wid)};
  s_xScale1 = len / config->rangeX;
  s_yScale1 = wid / config->rangeY;

  drawBasicGraph(len, wid, config->rangeX, config->rangeY, zeroOffsetY);

  //  titles
  settextcolor(BLACK);
  settextstyle(25, 0, "Calibri");
  outtextxy(s_origin1.x + len / 2 - textwidth(title) / 2,
            s_origin1.y - wid - textheight(title), title);
  settextstyle(20, 0, "Calibri", 900, 900, 0, 0, 0, 0);

  char titleY[10];
  if (title[0] == 'A')
    strcpy(titleY, "A (m/s2)");
  else if (title[0] == 'V')
    strcpy(titleY, "V (m/s)");
  else if (title[0] == 'S')
    strcpy(titleY, "S (m)");
  else
    strcpy(titleY, "y");

  outtextxy(s_origin1.x - 50, s_origin1.y - wid / 2, titleY);
  settextstyle(20, 0, "Calibri", 0, 0, 0, 0, 0, 0);
  char titleX[] = "T (s)";
  outtextxy(s_origin1.x + (len - textwidth(titleX)) / 2,
            s_origin1.y + textheight(titleX), titleX);

  Point curDrawP = {0, 0};
  Point lastDrawP = points[0];
  coordinateTrans1(&lastDrawP);
#if CURVE_FITTING_TYPE == 1
  float drawP[6][2];
  // bezier curve from result point
  if (title[0] == 'S') {
    memset(drawP, 0, sizeof(drawP));
    for (int i = 0; i <= 5; i++) {
      drawP[i][0] = points[i].x;
      drawP[i][1] = points[i].y;
    }
  }

  for (float i = 0.0f; i < 5; i += 0.4f) {
    if (title[0] == 'S') {
      bezierPoint(i, 5.0f, drawP, &curDrawP.x, &curDrawP.y);
    } else if (title[0] == 'V') {
      bezierDerivative(i, 5.0f, drawP, &curDrawP.x, &curDrawP.y);
      curDrawP.y = curDrawP.y / curDrawP.x;
      curDrawP.x = i;
      // printf("v, x =%.2f , y =%.2f\n ", curDrawP.x, curDrawP.y);
    } else if (title[0] == 'A') {
      bezierSecDerivative(i, 5.0f, drawP, &curDrawP.x, &curDrawP.y);
      curDrawP.y = curDrawP.y;
      curDrawP.y += zeroOffsetY;
      curDrawP.x = i;
      // printf("a, x =%.2f , y =%.2f\n ", curDrawP.x, curDrawP.y);
    }
    // bezierPoint(i, 5.0f, drawP, &curDrawP.x, &curDrawP.y);
  }
#elif CURVE_FITTING_TYPE == 2
  for (float i = 0.0f; i < 5; i += 0.2f) {
    if (title[0] == 'S') {
      curDrawP = {i, quinticPoly[0] + quinticPoly[1] * i +
                         quinticPoly[2] * i * i + quinticPoly[3] * i * i * i +
                         quinticPoly[4] * i * i * i * i +
                         quinticPoly[5] * i * i * i * i * i};
    } else if (title[0] == 'V') {
      curDrawP = {i, quinticPoly[1] + 2 * quinticPoly[2] * i +
                         3 * quinticPoly[3] * i * i +
                         4 * quinticPoly[4] * i * i * i +
                         5 * quinticPoly[5] * i * i * i * i};
    } else if (title[0] == 'A') {
      curDrawP = {i, 2 * quinticPoly[2] + 6 * quinticPoly[3] * i +
                         12 * quinticPoly[4] * i * i +
                         20 * quinticPoly[5] * i * i * i};
    }
    curDrawP.y += zeroOffsetY;
    coordinateTrans1(&curDrawP);
    line(lastDrawP.x, lastDrawP.y, curDrawP.x, curDrawP.y);
    lastDrawP = curDrawP;
  }
#endif
  // result points
  setfillcolor(pointColor);
  setlinecolor(pointColor);
  for (int i = startIndex; i < startIndex + pointNums; i++) {
    float val = points[i].y;
    points[i].y += zeroOffsetY;
    coordinateTrans1(&points[i]);
    if (i > 0)
      line(points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
    if (title[0] == 'A' || title[0] == 'S' || title[0] == 'V') {
      solidcircle(points[i].x, points[i].y, 5);
      char str[4] = "";
      sprintf(str, "%.1f", val);
      outtextxy(points[i].x + 5, points[i].y + 5, str);
    }
  }
  if (title[0] == 'A') {
    float val = ctrlPoint->y;
    ctrlPoint->y += zeroOffsetY;
    coordinateTrans1(ctrlPoint);
    solidcircle(ctrlPoint->x, ctrlPoint->y, 5);
    char str[4] = "";
    sprintf(str, "%.1f", val);
    outtextxy(ctrlPoint->x - textwidth(str) / 2, ctrlPoint->y - textheight(str),
              str);
  }
}

void initBEVGraph(const GraphConfig* config, const float zeroOffsetX) {
  float len = config->length - 2.0f * config->offset;
  float wid = config->width - 2.0f * config->offset;
  // vehicle frame: front-left-up, FLU. eg. x-longitudinal, y-lateral
  s_origin2 = {config->oriX + config->length * 0.5f,
               config->oriY + config->offset + wid};
  s_xScale2 = len / config->rangeY;
  s_yScale2 = wid / config->rangeX;
  s_origin2.y -= s_yScale2 * zeroOffsetX;
  s_infoAreaBoundary = config->oriX + config->length - 65;

  // title
  settextcolor(BLACK);
  settextstyle(25, 0, "Calibri");
  char title[] = " ";
  outtextxy(s_origin2.x - textwidth(title) / 2,
            config->offset - textheight(title), title);
  settextstyle(20, 0, "Calibri");
}

void showBEVGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const SsmObjType* ssmObjs,
                  const LinesInfo* linesInfo,
                  const TsrInfo* tsrInfo,
                  const MotionInfo* motionInfo) {
  initBEVGraph(config, zeroOffsetX);

  // tsr info
  drawTsrSign(tsrInfo);

  // road lines
  // lineType: 0-unknown, 1-solid, 2-dash, 32-Dash(inner)_Solid(outer),
  setlinestyle(PS_DASHDOT);
  Point lineEnd;
  int leftBoundaryColor = (motionInfo->alcBehav.LeftBoundaryType == 2 ||
                           motionInfo->alcBehav.LeftBoundaryType == 32)
                              ? GREEN
                              : RGB(0, 87, 55);
  int rightBoundaryColor = (motionInfo->alcBehav.RightBoundaryType == 2 ||
                            motionInfo->alcBehav.RightBoundaryType == 32)
                               ? GREEN
                               : RGB(0, 87, 55);
  drawQuinticPolyTraj(linesInfo->left_coeffs, leftBoundaryColor,
                      linesInfo->left_coeffs[6], linesInfo->left_coeffs[7],
                      linesInfo->left_coeffs[7], &lineEnd);
  drawQuinticPolyTraj(
      linesInfo->leftleft_coeffs, DARKGRAY, linesInfo->leftleft_coeffs[6],
      linesInfo->leftleft_coeffs[7], linesInfo->leftleft_coeffs[7], &lineEnd);
  drawQuinticPolyTraj(linesInfo->right_coeffs, rightBoundaryColor,
                      linesInfo->right_coeffs[6], linesInfo->right_coeffs[7],
                      linesInfo->right_coeffs[7], &lineEnd);
  drawQuinticPolyTraj(linesInfo->rightright_coeffs, DARKGRAY,
                      linesInfo->rightright_coeffs[6],
                      linesInfo->rightright_coeffs[7],
                      linesInfo->rightright_coeffs[7], &lineEnd);

  // obstacles
  drawObstacles(ssmObjs, &linesInfo->ego_coeffs, motionInfo->egoSpd);

  // navigation path, ego c7 as end point
  float naviRange = linesInfo->alc_coeffs[7];
  Point predictPosn = {0.0f, 0.0f};
  // ego lane path, c0 ~ c3
  drawPiecewiseCubicPolyTraj(&linesInfo->ego_coeffs, MAGENTA, 0.0f,
                             &predictPosn);

  drawQuinticPolyTraj(linesInfo->alc_coeffs, LIGHTRED, naviRange,
                      fmax(50.0f, naviRange), fmax(50.0f, naviRange),
                      &predictPosn);
  drawQuinticPolyTraj(linesInfo->alc_coeffs, RED, 0.0f, naviRange, naviRange,
                      &predictPosn);

  // ego car
  setfillcolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, 0};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  strCompletion(str_ego, 10, motionInfo->egoSpd);
  drawCar(&ego, str_ego, 1, 0, 10);

  if (show_predict_swt && predictPosn.x > 2.0f) {
    // setfillstyle(BS_HATCHED, HS_DIAGCROSS);
    Point ego_pred = {predictPosn.x, predictPosn.y};
    char str_ego_pred[2][8] = {};
    strcpy(str_ego_pred[0], "ego_pred");
    strCompletion(str_ego_pred, 10, motionInfo->egoPredSpd);
    drawCar(&ego_pred, str_ego_pred, 1, 0, 10);
    // setfillstyle(BS_SOLID);
  }
  // ego spd info and lane change status
  drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);
}

void drawRadarObj(const RadarObjInfo* radar_info) {
  for (int j = 0; j < 32; j++) {
    if (radar_info->iObjectId[j] == 0)
      continue;

    Point obj_posn = {radar_info->fDistX[j], radar_info->fDistY[j]};
    char obj_id[10] = "";
    snprintf(obj_id + strlen(obj_id), sizeof(obj_id) - strlen(obj_id), "%d", j);
    coordinateTrans2(&obj_posn);

    setlinecolor(BLACK);
    setfillcolor(DARKGRAY);
    solidcircle(obj_posn.x, obj_posn.y, 5);
    outtextxy(obj_posn.x - textwidth(obj_id) / 2,
              obj_posn.y + textheight(obj_id) / 2, obj_id);
  }
}

void showRadarGraph(const GraphConfig* config,
                    const float zeroOffsetX,
                    const RadarObjInfo* radar_info) {
  initBEVGraph(config, zeroOffsetX);

  drawRadarObj(radar_info);

  // ego car
  setfillcolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, 0.0f};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  drawCar(&ego, str_ego, 1, 0.0f, 10);

  drawBEVRuler(zeroOffsetX);
}

// following functions beckup
// delay 100ms to eliminate misoperation caused by long press
bool inArea(int mx, int my, int x, int y, int w, int h) {
  return (mx > x && mx < x + w && my > y && my < y + h);
}
bool button(ExMessage* msg, int x, int y, int w, int h, bool* swt) {
  bool ans =
      msg->message == WM_LBUTTONDOWN && inArea(msg->x, msg->y, x, y, w, h);
  if (ans) {
    *swt = !(*swt);
    Sleep(100);
  }
  const char* text = (*swt) ? "Pred On" : "Pred Off";
  if (inArea(msg->x, msg->y, x, y, w, h))
    setfillcolor(CYAN);
  else
    setfillcolor(RGB(255, 255, 255));
  setlinecolor(BLACK);
  fillroundrect(x, y, x + w, y + h, 5, 5);
  settextcolor(BLACK);
  outtextxy(x + (w - textwidth(text)) / 2, y + (h - textheight(text)) / 2,
            text);
  return ans;
}
bool functionButton(ExMessage msg) {
  // while (1) {
  return button(&msg, s_infoAreaBoundary - 2, 50, 60, 30, &show_predict_swt);

  // BeginBatchDraw();
  //  cleardevice();

  // EndBatchDraw();
  // msg.message = 0;
  // return;
}

void keyboardTest() {
  int x = 200, y = 200;
  BeginBatchDraw();
  ExMessage msg;
  while (1) {
    cleardevice();
    solidcircle(x, y, 10);
    FlushBatchDraw();
    if (GetAsyncKeyState(VK_DOWN))
      y++;
    if (GetAsyncKeyState(VK_UP))
      y--;
    if (GetAsyncKeyState(VK_LEFT))
      x--;
    if (GetAsyncKeyState(VK_RIGHT))
      x++;
  }
  if (peekmessage(&msg, EX_MOUSE)) {
    FlushBatchDraw();
    switch (msg.message) {
      case WM_LBUTTONDOWN:
        printf("left Button pressed\n");
        break;
      case WM_RBUTTONDOWN:
        printf("right Button pressed\n");
        break;
      default:
        break;
    }
  }
}