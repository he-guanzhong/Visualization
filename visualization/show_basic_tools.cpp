#include "visualization/show_basic_tools.h"

#define CURVE_FITTING_TYPE 2
bool g_showPredictSwt = false;
extern uint8 gScenarioFlg;
extern float gTempMeasureVal[4];

// Draw graph origins
static Point s_origin1 = {0.0f, 0.0f};
static float s_xScale1 = 0, s_yScale1 = 0;
static Point s_origin2 = {0.0f, 0.0f};
static float s_xScale2 = 0, s_yScale2 = 0;
static int s_infoAreaBoundary;

const static float kObsLengthTbl[10] = {2.6f, 5.4f, 10.4f, 2.5f, 2.5f,
                                        1.0f, 2.0f, 2.0f,  2.0f, 2.5f};
const static float kObsWidthTbl[10] = {1.4f, 1.9f, 2.0f, 0.6f, 0.6f,
                                       1.0f, 2.0f, 2.0f, 2.0f, 0.6f};
const static int kTsrSpdTbl1[14] = {10, 20, 30,  40,  50,  60,  70,
                                    80, 90, 100, 110, 120, 130, 140};
const static int kTsrSpdTbl2[15] = {5,  15, 25,  35,  45,  55,  65, 75,
                                    85, 95, 105, 115, 125, 135, 145};

void coordinateTrans1(Point* point) {
  point->x = s_origin1.x + point->x * s_xScale1;
  point->y = s_origin1.y - point->y * s_yScale1;
}

void coordinateTrans2(Point* point) {
  const float tmp = point->x;
  point->x = s_origin2.x - point->y * s_xScale2;
  point->y = s_origin2.y - tmp * s_yScale2;
}

void strCompletion(char str[2][8], const int index, const int spd) {
  const char* obsName[15] = {"IV ",   "RIV ",  "NIVL",  "NIIVL", "RIVL",
                             "RIIVL", "NIVR",  "NIIVR", "RIVR",  "RIIVR",
                             "NNIVL", "NNIVR", "NRIVL", "NRIVR", "Ego "};
  strcpy(str[0], obsName[index]);
  snprintf(&str[1][0], 8, "%d m/s", spd);
}

float getPiecewiseCubicPolynomial(const float x,
                                  const EgoPathVcc* egoPathVcc,
                                  const int order) {
  float x_prime = x;
  float c0 = 0, c1 = 0, c2 = 0, c3 = 0;
  if (x <= egoPathVcc->Len[0]) {
    c0 = egoPathVcc->C0[0], c1 = egoPathVcc->C1[0], c2 = egoPathVcc->C2[0],
    c3 = egoPathVcc->C3[0];
  } else if (x <= egoPathVcc->Len[0] + egoPathVcc->Len[1]) {
    x_prime = x - egoPathVcc->Len[0];
    c0 = egoPathVcc->C0[1], c1 = egoPathVcc->C1[1], c2 = egoPathVcc->C2[1],
    c3 = egoPathVcc->C3[1];
  } else {
    x_prime = x - egoPathVcc->Len[0] - egoPathVcc->Len[1];
    c0 = egoPathVcc->C0[2], c1 = egoPathVcc->C1[2], c2 = egoPathVcc->C2[2],
    c3 = egoPathVcc->C3[2];
  }
  if (0 == order) {
    return c0 + c1 * x_prime + c2 * x_prime * x_prime +
           c3 * x_prime * x_prime * x_prime;
  } else if (1 == order) {
    return c1 + 2.0f * c2 * x_prime + 3.0f * c3 * x_prime * x_prime;
  } else if (2 == order) {
    return 2.0f * c2 + 6.0f * c3 * x_prime;
  } else if (3 == order) {
    return 6.0f * c3;
  } else {
    return 0;
  }
}

void drawCar(Point* car,
             const char str[2][8],
             int carType,
             const float len,
             const float wid,
             const float yaw,
             const int index) {
  // ObjType of ME: 0=UNFILLED, 1=CAR, 2=TRUCK, 3=MOTORBIKE, 4=BICYCLE,
  // 5=PEDESTRIAN, 6=GENERAL_OBJECT, 7=ANIMAL 8=UNCERTAIN_VCL, 9=TWO_WHEELER
  if (carType >= 10) {
    carType = 0;
  }
  const float carLen = kObsLengthTbl[carType];
  float carWid = kObsWidthTbl[carType];
  /* limited trust in width of large truck, max legal width 2.6m */
  carWid = (carType == 2 && wid > 2.0f) ? fmin(wid, 2.6f) : carWid;

  // display: left-hand system. control: right-hand system
  // vertice order: upper-right -> lower-right-> lower-left -> upper-left
  const float halfLenCos = carLen / 2.0f * cosf(yaw),
              halfLenSin = carLen / 2.0f * sinf(yaw);
  const float halfWidCos = carWid / 2.0f * cosf(yaw),
              halfWidSin = carWid / 2.0f * sinf(yaw);
  Point vertices[4] = {
      {car->x + halfLenCos - halfWidSin, car->y + halfLenSin + halfWidCos},
      {car->x + halfLenCos + halfWidSin, car->y + halfLenSin - halfWidCos},
      {car->x - halfLenCos + halfWidSin, car->y - halfLenSin - halfWidCos},
      {car->x - halfLenCos - halfWidSin, car->y - halfLenSin + halfWidCos}};
  coordinateTrans2(car);
  for (int i = 0; i < 4; ++i)
    coordinateTrans2(&vertices[i]);
  POINT vertices_show[4];
  for (int i = 0; i < 4; ++i) {
    vertices_show[i].x = vertices[i].x;
    vertices_show[i].y = vertices[i].y;
  }

  fillpolygon(vertices_show, 4);

  if (carType >= 4 && carType <= 9) {  // pedestrian/others outline
    rectangle(car->x - kObsWidthTbl[1] / 2.0f * s_xScale2,
              car->y - kObsLengthTbl[1] / 2.0f * s_yScale2,
              car->x + kObsWidthTbl[1] / 2.0f * s_xScale2,
              car->y + kObsLengthTbl[1] / 2.0f * s_yScale2);
  } else if (carType == 3) {  // motorbike
    rectangle(car->x - 1.0f / 2.0f * s_xScale2,
              car->y - kObsLengthTbl[1] / 2.0f * s_yScale2,
              car->x + 1.0f / 2.0f * s_xScale2,
              car->y + kObsLengthTbl[1] / 2.0f * s_yScale2);
  }
  if (carType == 5) {
    solidcircle(car->x, car->y, 5);
  }

  if (index == 0 || index == 1 || index >= 14) {
    const int y_offset =
        carType == 2 ? textheight(str[0]) : textheight(str[0]) / 2;
    outtextxy(car->x - textwidth(str[0]), car->y + y_offset, str[0]);
    outtextxy(car->x, car->y + y_offset, str[1]);
  } else if (index <= 5 || index == 10 || index == 12) {
    outtextxy(car->x - 20 - textwidth(str[0]), car->y - textheight(str[0]),
              str[0]);
    outtextxy(car->x - 20 - textwidth(str[1]), car->y, str[1]);
  } else {
    outtextxy(car->x + 20, car->y - textheight(str[0]), str[0]);
    outtextxy(car->x + 20, car->y, str[1]);
  }
  return;
}

void drawPolygon(const Point* center, const int num, const float rotateDegree) {
  const int r = 15;
  int vertex_x = r, vertex_y = 0;
  POINT vertices_show[num];
  for (int i = 0; i < num; i++) {
    vertex_x = center->x + r * cosf(2 * M_PI / num * i + rotateDegree);
    vertex_y = center->y + r * sinf(2 * M_PI / num * i + rotateDegree);
    vertices_show[i] = {vertex_x, vertex_y};
  }
  polygon(vertices_show, num);
  return;
}

void drawTsrSign(const TsrInfo* tsrInfo) {
  // TSR status display
  char tsr_disp[] = "TSR: ";
  char spd_val[10] = "";
  const int tsr_spd = tsrInfo->tsr_spd;
  const bool tsr_spd_warn = tsrInfo->tsr_spd_warn;
  if (tsr_spd == 166) {
    strcpy(spd_val, "Cnl");
  } else {
    itoa(tsr_spd, spd_val, 10);
  }
  strcat(tsr_disp, spd_val);
  char tsi_disp[10] = "TSI ";
  if (tsrInfo->tsr_tsi[0] == 5) {
    strcpy(tsi_disp, "stop");
  } else if (tsrInfo->tsr_tsi[0] == 6) {
    strcpy(tsi_disp, "yield");
  } else if (tsrInfo->tsr_tsi[1] == 5) {
    strcpy(tsi_disp, "noEnt");
  }
  if (tsr_spd_warn) {
    settextcolor(RED);
  }
  outtextxy(s_infoAreaBoundary, s_origin2.y, tsr_disp);
  settextcolor(BLACK);
  if (tsrInfo->tsr_tsi[0] || tsrInfo->tsr_tsi[1]) {
    outtextxy(s_infoAreaBoundary, s_origin2.y + textheight(tsr_disp), tsi_disp);
  }

  // TSR original input
  for (int i = 0; i < 3; i++) {
    if (!tsrInfo->tsr_signs[i].valid || tsrInfo->tsr_signs[i].type < 0 ||
        tsrInfo->tsr_signs[i].type > 210) {
      continue;
    }
    Point tsr_pos = {tsrInfo->tsr_signs[i].pos_x, tsrInfo->tsr_signs[i].pos_y};
    coordinateTrans2(&tsr_pos);

    // TSI (traffic sign indication). 210 = stop, 168 = yield, 199 = no entry
    if (tsrInfo->tsr_signs[i].type == 168) {
      drawPolygon(&tsr_pos, 3, M_PI / 2);
    } else if (tsrInfo->tsr_signs[i].type == 210) {
      drawPolygon(&tsr_pos, 8, M_PI / 8);
      const char stop_sign[] = "stop";
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
    char tsr_sign[4] = "";
    if (tsrInfo->tsr_signs[i].type <= 13) {
      itoa(kTsrSpdTbl1[tsrInfo->tsr_signs[i].type], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsrInfo->tsr_signs[i].type >= 28 &&
               tsrInfo->tsr_signs[i].type <= 41) {
      itoa(kTsrSpdTbl1[tsrInfo->tsr_signs[i].type - 28], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(LIGHTRED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsrInfo->tsr_signs[i].type >= 100 &&
               tsrInfo->tsr_signs[i].type <= 114) {
      itoa(kTsrSpdTbl2[tsrInfo->tsr_signs[i].type - 100], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsrInfo->tsr_signs[i].type == 79 ||
               tsrInfo->tsr_signs[i].type == 80) {
      strcpy(tsr_sign, "Cnl");
      const int r = textheight(tsr_sign) * 0.75f;
      const int r_x = (float)r / sqrtf(2.0f);
      setlinecolor(BLACK);
      line(tsr_pos.x - r_x, tsr_pos.y + r_x, tsr_pos.x + r_x, tsr_pos.y - r_x);
      circle(tsr_pos.x, tsr_pos.y, r);
    }
  }
  return;
}

void drawMotionInfo(const MotionInfo* motionInfo) {
  // set spd display, only 5 characters in title is acceptable
  char spd_title[10] = "Spd: ";
  char set_title[10] = "SET: ";
  char tau_title[10] = "Thw: ";
  char inner_set_title[10] = "Inn: ";
  char spec_case_title[10] = "Spc: ";
  char scenario_title[10] = "Sce: ";
  char maxDecel_title[10] = "Dcl: ";

  const int cur_spd = round(motionInfo->egoSpd * 1.04f * 3.6f);
  const int spd_lmt = round(motionInfo->spdLmt);
  const int inner_spd_lmt = round(motionInfo->innerSpdLmt);

  const int tle_len = strlen(spd_title);
  snprintf(spd_title + tle_len, sizeof(spd_title) - tle_len, "%d", cur_spd);
  snprintf(set_title + tle_len, sizeof(set_title) - tle_len, "%d", spd_lmt);
  snprintf(inner_set_title + tle_len, sizeof(inner_set_title) - tle_len, "%d",
           inner_spd_lmt);
  snprintf(tau_title + tle_len, sizeof(tau_title) - tle_len, "%d",
           motionInfo->tauGap);
  snprintf(maxDecel_title + tle_len, sizeof(maxDecel_title) - tle_len, "%.2f",
           motionInfo->maxDecel);

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
    case 4:
    case 14:
      strcat(spec_case_title, "mgBk");  // merge brake
      break;
    case 5:
    case 15:
      strcat(spec_case_title, "mgAc");  // merge active
      break;
    default:
      break;
  }

  switch (motionInfo->scenarioFlg) {
    case 8:
    case 18:
      strcat(scenario_title, "curv");
      break;
    case 10:
      strcat(scenario_title, "Ndg");
      break;
    case 9:
    case 19:
      strcat(scenario_title, "dec");
      break;
    case 5:
      strcat(scenario_title, "r2m");
      break;
    case 15:
      strcat(scenario_title, "r2mN");
      break;
    case 6:
      strcat(scenario_title, "m2r");
      break;
    case 16:
      strcat(scenario_title, "m2rN");
      break;
    case 7:
      strcat(scenario_title, "Ext");
      break;
    case 17:
      strcat(scenario_title, "ExtN");
      break;
    case 1:
      strcat(scenario_title, "gA");
      break;
    case 11:
      strcat(scenario_title, "gAN");
      break;
    case 2:
      strcat(scenario_title, "gC");
      break;
    case 12:
      strcat(scenario_title, "gCN");
      break;
    case 3:
      strcat(scenario_title, "gB");
      break;
    case 13:
      strcat(scenario_title, "gBN");
      break;
    case 4:
      strcat(scenario_title, "gF");
      break;
    case 14:
      strcat(scenario_title, "gFN");
      break;
    default:
      break;
  }
  outtextxy(s_infoAreaBoundary, s_origin2.y - 8 * textheight(spd_title),
            inner_set_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - 7 * textheight(spd_title),
            spec_case_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - 6 * textheight(spd_title),
            scenario_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - 5 * textheight(spd_title),
            maxDecel_title);

  // ACC mode: 3-stand still, 4-stand active, 5-active, 6-override
  if (motionInfo->accMode <= 5 && motionInfo->accMode >= 3) {
    settextcolor(GREEN);
  } else if (motionInfo->accMode == 6) {
    settextcolor(RED);
  } else {
    settextcolor(BLACK);
  }

  outtextxy(s_infoAreaBoundary, s_origin2.y - textheight(spd_title) * 3,
            tau_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y - textheight(spd_title), spd_title);
  if (motionInfo->accMode <= 6 && motionInfo->accMode >= 3) {
    outtextxy(s_infoAreaBoundary, s_origin2.y - 2 * textheight(spd_title),
              set_title);
  }

  char alc_side[8] = "";
  if (motionInfo->alcBehav.AutoLaneChgSide == 1) {
    strcpy(alc_side, "ALC_L");
  } else if (motionInfo->alcBehav.AutoLaneChgSide == 2) {
    strcpy(alc_side, "ALC_R");
  }
  // alc sts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving,
  // 4-in target line, 5-finished, 6-Back to Ego, 8-takeover, 9-popMsgReq
  char alc_sts[12] = "";
  if (motionInfo->alcBehav.AutoLaneChgSts == 2)
    strcpy(alc_sts, "Hold");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 3)
    strcpy(alc_sts, "Chg1");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 4)
    strcpy(alc_sts, "Chg2");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 5)
    strcpy(alc_sts, "Fini");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 6)
    strcpy(alc_sts, "Back");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 8)
    strcpy(alc_sts, "Takeover");
  else if (motionInfo->alcBehav.AutoLaneChgSts == 9)
    strcpy(alc_sts, "PopMsg");
  if (motionInfo->alcBehav.AutoLaneChgSide == 1 ||
      motionInfo->alcBehav.AutoLaneChgSide == 2) {
    outtextxy(s_infoAreaBoundary, s_origin2.y - 10 * textheight(alc_side),
              alc_side);
    outtextxy(s_infoAreaBoundary, s_origin2.y - 9 * textheight(alc_side),
              alc_sts);
  }

  settextcolor(BLACK);
  if (motionInfo->gapIndex < 5 && motionInfo->gapTarV) {
    const float tarS = motionInfo->gapTarS;
    const float tarL = motionInfo->alcBehav.AutoLaneChgSide == 1 ? 3.4f : -3.4f;
    Point tarPoint = {tarS, tarL};
    coordinateTrans2(&tarPoint);
    solidcircle(tarPoint.x, tarPoint.y, 5);
    char str_tar1[10] = "Gap:";
    char str_tar2[10] = "";
    const int tar1_len = strlen(str_tar1);
    const int tar2_len = strlen(str_tar2);
    snprintf(str_tar1 + tar1_len, sizeof(str_tar1) - tar1_len, " %d",
             motionInfo->gapIndex);
    snprintf(str_tar2 + tar2_len, sizeof(str_tar2) - tar2_len, "%.1f m/s",
             motionInfo->gapTarV);
    const int offset = motionInfo->alcBehav.AutoLaneChgSide == 1
                           ? -textwidth(str_tar2)
                           : textwidth(str_tar2) / 4.0f;
    outtextxy(tarPoint.x + offset, tarPoint.y - textheight(str_tar1), str_tar1);
    outtextxy(tarPoint.x + offset, tarPoint.y, str_tar2);
  }
  return;
}

void drawQuinticPolyTraj(const float* coeffs,
                         const int color,
                         const float startX,
                         const float lengthX) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  for (float x = startX; x < lengthX; x += 3.0f) {
    const float y = getQuinticPolynomial(x, coeffs, 0);
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != 0.0f && x < lengthX) {
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    }
    lastDrawPoint = curDrawPoint;
  }
  return;
}

void getEgoPredictPose(const float* path1,
                       const float* path2,
                       const float lengthS,
                       Point* egoPredPosn,
                       float* egoPredYaw) {
  Point last = {0.0f, 0.0f};
  float restLen = lengthS;
  const float startX1 = path1[6], len1 = path1[7], len2 = path2[7];
  for (float x = startX1; restLen > 0; x += 1.0f) {
    const float y = (x < len1 || len2 < 1.0f)
                        ? getQuinticPolynomial(x, path1, 0)
                        : getQuinticPolynomial(x - len1, path2, 0);
    if (last.x != 0.0f) {
      float delta_s = hypotf(x - last.x, y - last.y);
      restLen -= delta_s;
    }
    last.x = x, last.y = y;
  }
  egoPredPosn->x = last.x;
  egoPredPosn->y = last.y;
  *egoPredYaw = (last.x < len1 || len2 < 1.0f)
                    ? getQuinticPolynomial(last.x, path1, 1)
                    : getQuinticPolynomial(last.x - len1, path2, 1);
  return;
}

void drawPiecewiseCubicPolyTraj(const EgoPathVcc* egoPath,
                                const int color,
                                const float startX) {
  setlinecolor(color);
  Point lastDrawPoint = {0.0f, 0.0f};
  Point last = {0.0f, 0.0f};
  float totalLen = egoPath->Len[0] + egoPath->Len[1] + egoPath->Len[2];
  for (float x = startX; x < totalLen; x += 3.0f) {
    const float y = getPiecewiseCubicPolynomial(x, egoPath, 0);
    last.x = x, last.y = y;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (lastDrawPoint.x != startX) {
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    }
    lastDrawPoint = curDrawPoint;
  }
  return;
}

void drawPiecewisQuinticPolyTraj(const float* coeffs1,
                                 const float* coeffs2,
                                 const int color1,
                                 const int color2) {
  setlinecolor(color1);
  Point lastDrawPoint = {0.0f, 0.0f};
  Point last = {0.0f, 0.0f};
  const float startX1 = coeffs1[6], len1 = coeffs1[7], len2 = coeffs2[7];
  for (float x = startX1; x < len1 + len2; x += 3.0f) {
    const float y = (x < len1 || len2 < 1.0f)
                        ? getQuinticPolynomial(x, coeffs1, 0)
                        : getQuinticPolynomial(x - len1, coeffs2, 0);
    last.x = x, last.y = y;
    Point curDrawPoint = {x, y};
    coordinateTrans2(&curDrawPoint);
    if (x > len1) {
      setlinecolor(color2);
    }
    if (lastDrawPoint.x != startX1) {
      line(curDrawPoint.x, curDrawPoint.y, lastDrawPoint.x, lastDrawPoint.y);
    }
    lastDrawPoint = curDrawPoint;
  }
  return;
}

void predictionObstacle(Point obs_pred_path[11],
                        float obs_pred_path_yaw[11],
                        const SsmObsType* obs,
                        const int index,
                        const EgoPathVcc* egoPath,
                        const float* LH0,
                        const float* LH1,
                        const float cur_spd,
                        const float* ssmObjSpdY) {
  const float t_interval = 0.5f;
  const float objSpdLatConf = 1.0f;
  float obs_speed_y_cor = 0;
  float objPosnLgt[11], roadPosnLat[11];
  const bool leftRearObsRampOn =
      (index == 4 || index == 5 || index == 10 || index == 12) &&
      (5 == gScenarioFlg || 15 == gScenarioFlg);
  const bool rightRearObsRampOn =
      (index == 8 || index == 9 || index == 11 || index == 13) &&
      (5 == gScenarioFlg || 15 == gScenarioFlg);
  const bool frontObsFlg = index == 0 || index == 2 || index == 3 ||
                           index == 6 || index == 7 || index == 10 ||
                           index == 11;
  /* LatSpd: filtered for front obs, untrustworthy for rear obs */
  if (frontObsFlg) {
    obs_speed_y_cor = ssmObjSpdY[index];
  }

  const float const_acc_time = (index == 0 ? 2.5f : 0.0f);
  for (int j = 0; j <= 10; ++j) {
    if (0 == j) {
      obs_pred_path[j].x = obs->pos_x;
      obs_pred_path[j].y = obs->pos_y;
      obs_pred_path_yaw[j] = obs->pos_yaw;
      objPosnLgt[j] = obs->pos_x;
      roadPosnLat[j] =
          leftRearObsRampOn
              ? getCubicPolynomial(objPosnLgt[j], LH0, 0)
              : (rightRearObsRampOn
                     ? getCubicPolynomial(objPosnLgt[j], LH1, 0)
                     : getPiecewiseCubicPolynomial(objPosnLgt[j], egoPath, 0));
      continue;
    }

    const float t = t_interval * j;
    if (t <= const_acc_time) {
      obs_pred_path[j].x =
          obs->pos_x + obs->speed_x * t + 0.5f * obs->acc_x * t * t;
    } else {
      obs_pred_path[j].x = obs->pos_x +
                           (obs->speed_x + obs->acc_x * const_acc_time) * t -
                           0.5f * obs->acc_x * const_acc_time * const_acc_time;
    }
    obs_pred_path[j].x = fmaxf(obs_pred_path[j - 1].x, obs_pred_path[j].x);

    float predHeadingAg = obs->pos_yaw;
    float predLatOffset = obs_speed_y_cor * t_interval;
    objPosnLgt[j] = obs_pred_path[j].x;
    float roadCurveOffset = 0.0f;
    if (cur_spd > 10.0f / 3.6f && obs->speed_x > 1.0f &&
        objPosnLgt[j] < 150.0f) {
      if (leftRearObsRampOn) {
        roadPosnLat[j] = getCubicPolynomial(objPosnLgt[j], LH0, 0);
      } else if (rightRearObsRampOn) {
        roadPosnLat[j] = getCubicPolynomial(objPosnLgt[j], LH1, 0);
      } else {
        roadPosnLat[j] = getPiecewiseCubicPolynomial(objPosnLgt[j], egoPath, 0);
      }
      roadCurveOffset = roadPosnLat[j] - roadPosnLat[j - 1];

      if ((roadCurveOffset > predLatOffset && predLatOffset >= 0) ||
          (roadCurveOffset < predLatOffset && predLatOffset <= 0) ||
          fabsf(roadCurveOffset) > fabsf(predLatOffset)) {
        predLatOffset = roadCurveOffset;
        predHeadingAg = getPiecewiseCubicPolynomial(objPosnLgt[j], egoPath, 1);
      }
    }
    obs_pred_path[j].y = obs_pred_path[j - 1].y + predLatOffset;
    obs_pred_path_yaw[j] = predHeadingAg;
  }
  return;
}

void drawObstacles(const SsmObjType* ssmObjs,
                   const EgoPathVcc* egoPath,
                   const float* LH0,
                   const float* LH1,
                   const float cur_spd,
                   const float* ssmObjSpdY) {
  for (int i = 0; i < ssmObjs->obj_num; i++) {
    if (!ssmObjs->obj_lists[i].valid_flag) {
      continue;
    }
    const SsmObsType* obs = &ssmObjs->obj_lists[i];
    Point obs_cur = {obs->pos_x, obs->pos_y};
    char str_obs_cur[2][8] = {};
    strCompletion(str_obs_cur, i, obs->speed_x);
    setlinecolor(BLACK);
    setfillcolor(DARKGRAY);
    drawCar(&obs_cur, str_obs_cur, obs->type, obs->length, obs->width,
            obs->pos_yaw, i);

    // obs latspd not stable, use ego centre line offset
    Point obs_pred_path[11];
    float obs_pred_path_yaw[11];
    predictionObstacle(obs_pred_path, obs_pred_path_yaw, obs, i, egoPath, LH0,
                       LH1, cur_spd, ssmObjSpdY);

    Point obs_pred = obs_pred_path[10];
    float obs_pred_yaw = obs_pred_path_yaw[10];

    if (g_showPredictSwt && fabsf(obs_pred.x - obs->pos_x) > 5.0f) {
      char str_obs_pred[2][8] = {};
      strcpy(str_obs_pred[1], "Pred");
      setfillcolor(LIGHTGRAY);
      strCompletion(str_obs_pred, i, obs->speed_x);
      drawCar(&obs_pred, str_obs_pred, obs->type, obs->length, obs->width,
              obs_pred_yaw, i);
      setlinecolor(LIGHTGRAY);
      setlinestyle(PS_DASH);

      coordinateTrans2(&obs_pred_path[0]);
      for (int j = 0; j < 10; j++) {
        coordinateTrans2(&obs_pred_path[j + 1]);
        line(obs_pred_path[j].x, obs_pred_path[j].y, obs_pred_path[j + 1].x,
             obs_pred_path[j + 1].y);
      }
    }
  }
  return;
}

void drawBEVRuler(const float zeroOffsetX) {
  settextcolor(BLACK);
  const float coordinate_x_of_ruler_y = fminf(-6.0f, -zeroOffsetX);
  Point ruler_x[4] = {
      {-30.0f, -6.0f}, {0.0f, -6.0f}, {50.0f, -6.0f}, {100.0f, -6.0f}};
  Point ruler_y[2] = {{coordinate_x_of_ruler_y, -5.0f},
                      {coordinate_x_of_ruler_y, 5.0f}};
  char str[7] = "";
  for (int i = 0; i < 4; ++i) {
    snprintf(str, sizeof(str), "%d m", (int)ruler_x[i].x);
    coordinateTrans2(&ruler_x[i]);
    line(ruler_x[i].x, ruler_x[i].y, ruler_x[i].x - 10, ruler_x[i].y);
    outtextxy(ruler_x[i].x, ruler_x[i].y - textheight(str) / 2, str);
  }
  for (int i = 0; i < 2; i++) {
    snprintf(str, sizeof(str), "%d m", (int)ruler_y[i].y);
    coordinateTrans2(&ruler_y[i]);
    line(ruler_y[i].x, ruler_y[i].y, ruler_y[i].x, ruler_y[i].y + 10);
    outtextxy(ruler_y[i].x - textwidth(str) / 2, ruler_y[i].y + 10, str);
  }
  return;
}

void drawBasicGraph(const int len,
                    const int wid,
                    const float rangeX,
                    const float rangeY,
                    const float offsetY,
                    const int gridNumsX,
                    const int gridNumsY) {
  // boundary
  setlinecolor(BLACK);
  rectangle(s_origin1.x, s_origin1.y, s_origin1.x + len, s_origin1.y - wid);

  setlinecolor(LIGHTGRAY);
  settextcolor(BLACK);
  settextstyle(20, 0, "Calibri");
  // horizontal lines
  const float intervalX = rangeX / gridNumsX, intervalY = rangeY / gridNumsY;
  for (int i = 1; i <= gridNumsY; i++) {
    Point point1 = {0.0f, i * intervalY};
    Point point2 = {rangeX, i * intervalY};
    coordinateTrans1(&point1);
    coordinateTrans1(&point2);
    line(point1.x, point1.y, point2.x, point2.y);
    point1 = {0.0f, i * intervalY};
    char str[8] = "";
    const float val = i * intervalY - offsetY;
    if (intervalY > 0.9f) {
      itoa(i * intervalY - offsetY, str, 10);
    } else if (intervalY < 0.02f) {
      snprintf(str, sizeof(str), "%.4f", val);
    } else {
      snprintf(str, sizeof(str), "%.2f", val);
    }
    coordinateTrans1(&point1);
    outtextxy(point1.x - textwidth(str) - textwidth(" "),
              point1.y - textheight(str) / 2, str);
  }
  // vertical lines
  for (int i = 1; i <= gridNumsX; i++) {
    Point point1 = {i * intervalX, 0.0f};
    Point point2 = {i * intervalX, rangeY};
    coordinateTrans1(&point1);
    coordinateTrans1(&point2);
    line(point1.x, point1.y, point2.x, point2.y);
    point1 = {i * intervalX, 0.0f};
    char str[8] = {0};
    const float val = i * intervalX;
    if (intervalX > 0.9f) {
      itoa(i * intervalX, str, 10);
    } else if (intervalX < 0.02f) {
      snprintf(str, sizeof(str), "%.4f", val);
    } else {
      snprintf(str, sizeof(str), "%.2f", val);
    }
    coordinateTrans1(&point1);
    outtextxy(point1.x - textwidth(str) / 2, point1.y, str);
  }
  return;
}

void showXYGraph(const GraphConfig* config,
                 const float zeroOffsetY,
                 const PlotInfo* plot) {
  // float zeroOffsetY = 4.0f;  // positive means: 0 moves up
  const float len = config->length - 2 * config->offset;
  const float wid = config->width - 2 * config->offset;

  // x = horizental, y = vertical
  s_origin1 = {(float)(config->oriX + config->offset),
               (float)(config->oriY + config->offset + wid)};
  s_xScale1 = len / config->rangeX;
  s_yScale1 = wid / config->rangeY;

  drawBasicGraph(len, wid, config->rangeX, config->rangeY, zeroOffsetY,
                 config->gridNumsX, config->gridNumsY);

  //  titles
  settextcolor(BLACK);
  settextstyle(25, 0, "Calibri");
  outtextxy(s_origin1.x + len / 2 - textwidth(plot->title) / 2,
            s_origin1.y - wid - textheight(plot->title), plot->title);
  settextstyle(20, 0, "Calibri", 900, 900, 0, 0, 0, 0);

  char titleY[10] = "";
  if (plot->title[0] == 'A') {
    strcpy(titleY, "A (m/s2)");
  } else if (plot->title[0] == 'V') {
    strcpy(titleY, "V (m/s)");
  } else if (plot->title[0] == 'S') {
    strcpy(titleY, "S (m)");
  } else {
    strcpy(titleY, "");
  }

  outtextxy(s_origin1.x - 50, s_origin1.y - wid / 2, titleY);
  settextstyle(20, 0, "Calibri", 0, 0, 0, 0, 0, 0);
  char titleX[] = "T (s)";
  outtextxy(s_origin1.x + (len - textwidth(titleX)) / 2,
            s_origin1.y + textheight(titleX), titleX);

  Point curDrawP = {0, 0};
  Point lastDrawP = plot->points[0];
  coordinateTrans1(&lastDrawP);
#if CURVE_FITTING_TYPE == 1
  float drawP[6][2];
  // bezier curve from result point
  if (title[0] == 'S') {
    memset(drawP, 0, sizeof(drawP));
    for (int i = 0; i <= 5; i++) {
      drawP[i][0] = plot->points[i].x;
      drawP[i][1] = plot->points[i].y;
    }
  }

  for (float i = 0.0f; i < 5; i += 0.4f) {
    if (title[0] == 'S') {
      bezierPoint(i, 5, drawP, &curDrawP.x, &curDrawP.y);
    } else if (title[0] == 'V') {
      bezierDerivative(i, 5, drawP, &curDrawP.x, &curDrawP.y);
      curDrawP.y = curDrawP.y / curDrawP.x;
      curDrawP.x = i;
      // printf("v, x =%.2f , y =%.2f\n ", curDrawP.x, curDrawP.y);
    } else if (title[0] == 'A') {
      bezierSecDerivative(i, 5, drawP, &curDrawP.x, &curDrawP.y);
      curDrawP.y = curDrawP.y;
      curDrawP.y += zeroOffsetY;
      curDrawP.x = i;
      // printf("a, x =%.2f , y =%.2f\n ", curDrawP.x, curDrawP.y);
    }
    // bezierPoint(i, 5.0f, drawP, &curDrawP.x, &curDrawP.y);
  }
#endif
  // quintic polynominal curve
  if (plot->showPoly) {
    for (float i = 0.0f; i < 5.0f; i += 0.2f) {
      if (plot->title[0] == 'S') {
        curDrawP = {i, plot->quinticPoly[0] + plot->quinticPoly[1] * i +
                           plot->quinticPoly[2] * i * i +
                           plot->quinticPoly[3] * i * i * i +
                           plot->quinticPoly[4] * i * i * i * i +
                           plot->quinticPoly[5] * i * i * i * i * i};
      } else if (plot->title[0] == 'V') {
        curDrawP = {i, plot->quinticPoly[1] + 2 * plot->quinticPoly[2] * i +
                           3 * plot->quinticPoly[3] * i * i +
                           4 * plot->quinticPoly[4] * i * i * i +
                           5 * plot->quinticPoly[5] * i * i * i * i};
      } else if (plot->title[0] == 'A') {
        curDrawP = {i, 2 * plot->quinticPoly[2] + 6 * plot->quinticPoly[3] * i +
                           12 * plot->quinticPoly[4] * i * i +
                           20 * plot->quinticPoly[5] * i * i * i};
      }
      curDrawP.y += zeroOffsetY;
      coordinateTrans1(&curDrawP);
      if (i > 0) {
        line(lastDrawP.x, lastDrawP.y, curDrawP.x, curDrawP.y);
      }
      lastDrawP = curDrawP;
    }
  }

  // result points
  setfillcolor(plot->pointColor);
  setlinecolor(plot->pointColor);
  for (int i = plot->startIndex; i < plot->startIndex + plot->pointNums; ++i) {
    const float val = plot->points[i].y;
    plot->points[i].y += zeroOffsetY;
    coordinateTrans1(&plot->points[i]);
    if (i > 0) {
      line(plot->points[i - 1].x, plot->points[i - 1].y, plot->points[i].x,
           plot->points[i].y);
    }
    if (plot->title[0] == 'A' || plot->title[0] == 'S' ||
        plot->title[0] == 'V') {
      solidcircle(plot->points[i].x, plot->points[i].y, 5);
      char str[4] = "";
      sprintf(str, "%.1f", val);
      outtextxy(plot->points[i].x + 5, plot->points[i].y + 5, str);
    }
  }
  if (plot->title[0] == 'A') {
    const float val = plot->ctrlPoint->y;
    plot->ctrlPoint->y += zeroOffsetY;
    coordinateTrans1(plot->ctrlPoint);
    solidcircle(plot->ctrlPoint->x, plot->ctrlPoint->y, 5);
    char str[4] = "";
    sprintf(str, "%.1f", val);
    outtextxy(plot->ctrlPoint->x - textwidth(str) / 2,
              plot->ctrlPoint->y - textheight(str), str);
  }
  return;
}

void initBEVGraph(const GraphConfig* config, const float zeroOffsetX) {
  const float len = config->length - 2.0f * config->offset;
  const float wid = config->width - 2.0f * config->offset;
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
  return;
}

void showBEVGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const SsmObjType* ssmObjs,
                  const LinesInfo* linesInfo,
                  const TsrInfo* tsrInfo,
                  const MotionInfo* motionInfo,
                  const float* ssmObjSpdY,
                  const ReservedInfo* reservedInfo) {
  initBEVGraph(config, zeroOffsetX);
  // Note: Origin slightly further left, for aesthetic purpose
  s_origin2.x *= 0.9f;

  // tsr info
  drawTsrSign(tsrInfo);

  /* road line type:
    0-unknown, 1-solid, 2-dash, 3-Double Lane(Near Dashed,FarSolid)
    9- DECELERATION_Dashed */
  setlinestyle(PS_DASHDOT);
  Point lineEnd;
  const int leftBoundaryColor = (2 == motionInfo->alcBehav.LeftBoundaryType ||
                                 3 == motionInfo->alcBehav.LeftBoundaryType ||
                                 9 == motionInfo->alcBehav.LeftBoundaryType)
                                    ? GREEN
                                    : RGB(0, 87, 55);
  const int rightBoundaryColor = (2 == motionInfo->alcBehav.RightBoundaryType ||
                                  3 == motionInfo->alcBehav.RightBoundaryType ||
                                  9 == motionInfo->alcBehav.RightBoundaryType)
                                     ? GREEN
                                     : RGB(0, 87, 55);
  drawQuinticPolyTraj(linesInfo->left_coeffs, leftBoundaryColor,
                      linesInfo->left_coeffs[6], linesInfo->left_coeffs[7]);
  drawQuinticPolyTraj(linesInfo->leftleft_coeffs, DARKGRAY,
                      linesInfo->leftleft_coeffs[6],
                      linesInfo->leftleft_coeffs[7]);
  drawQuinticPolyTraj(linesInfo->right_coeffs, rightBoundaryColor,
                      linesInfo->right_coeffs[6], linesInfo->right_coeffs[7]);
  drawQuinticPolyTraj(linesInfo->rightright_coeffs, DARKGRAY,
                      linesInfo->rightright_coeffs[6],
                      linesInfo->rightright_coeffs[7]);

  // obstacles
  drawObstacles(ssmObjs, &linesInfo->ego_coeffs, linesInfo->left_coeffs,
                linesInfo->right_coeffs, motionInfo->egoSpd, ssmObjSpdY);

  // navigation path, ego c7 as end point
  const float naviRange = linesInfo->alc_coeffs[7];

  // ego lane path, c0 ~ c3
  setlinestyle(PS_DASHDOT, 1);
  drawPiecewiseCubicPolyTraj(&linesInfo->ego_coeffs, MAGENTA, 0.0f);

  // alc path,
  /*   drawQuinticPolyTraj(linesInfo->alc2_coeffs, LIGHTRED,
                        linesInfo->alc_coeffs[7],
                        linesInfo->alc_coeffs[7] + linesInfo->alc2_coeffs[7]);
    drawQuinticPolyTraj(linesInfo->alc_coeffs, RED, linesInfo->alc_coeffs[6],
                        linesInfo->alc_coeffs[7]); */
  drawPiecewisQuinticPolyTraj(linesInfo->alc_coeffs, linesInfo->alc2_coeffs,
                              RED, LIGHTRED);

  Point egoPredPosn = {0.0f, 0.0f};
  float egoPredYaw = 0.0f;
  getEgoPredictPose(linesInfo->alc_coeffs, linesInfo->alc2_coeffs,
                    motionInfo->egoPredPos, &egoPredPosn, &egoPredYaw);

  // ego car
  const int ego_index = ssmObjs->obj_num;
  if (1 == motionInfo->enblSts) {
    setfillcolor(LIGHTRED);
  } else if (2 == motionInfo->enblSts) {
    setfillcolor(MAGENTA);
  } else {
    setfillcolor(RED);
  }
  setlinecolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, 0};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  strCompletion(str_ego, ego_index, motionInfo->egoSpd);
  drawCar(&ego, str_ego, 1, kObsLengthTbl[1], kObsWidthTbl[1], 0, ego_index);

  if (g_showPredictSwt && egoPredPosn.x > 4.0f) {
    char str_ego_pred[2][8] = {};
    strcpy(str_ego_pred[0], "ego_pred");
    strCompletion(str_ego_pred, ego_index, motionInfo->egoPredSpd);
    drawCar(&egoPredPosn, str_ego_pred, 1, kObsLengthTbl[1], kObsWidthTbl[1],
            egoPredYaw, ego_index);
    // setfillstyle(BS_SOLID);
  }
  // ego spd info and lane change status
  drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);

  /* To meet temporary signal display requirements, could be customized */
  drawReservedInfo(reservedInfo);
  return;
}

void showLineChart(GraphConfig* chartConfig,
                   float zeroOffsetY,
                   const int curFrame,
                   const int totalFrame,
                   const int winFrames,
                   const float* time_data,
                   const float* original_data,
                   const float* loopback_data,
                   const char* title,
                   const bool fixedRangeYFlg) {
  const int startFrame = fmax(0, curFrame - winFrames / 2);
  const int endFrame = fmin(totalFrame - 1, curFrame + winFrames / 2);

  Point original_arr[winFrames];
  Point loopback_arr[winFrames];
  float min_arr_y = 100.0f, max_arr_y = -100.0f;
  for (int i = 0; i < winFrames; i++) {
    original_arr[i].x = time_data[i + startFrame] - time_data[startFrame];
    original_arr[i].y = original_data[i + startFrame];
    loopback_arr[i].x = original_arr[i].x;
    loopback_arr[i].y = loopback_data[i + startFrame];
    min_arr_y = fmin(min_arr_y, original_arr[i].y);
    max_arr_y = fmax(max_arr_y, original_arr[i].y);
  }

  chartConfig->rangeX = original_arr[winFrames - 1].x - original_arr[0].x;
  if (!fixedRangeYFlg) {
    int interval_tmp = chartConfig->gridNumsY / chartConfig->rangeY;
    zeroOffsetY = fmax(zeroOffsetY, fmax(fabsf(min_arr_y), fabsf(max_arr_y)));
    zeroOffsetY = ceilf(zeroOffsetY * interval_tmp) / interval_tmp;
    chartConfig->rangeY = fmax(chartConfig->rangeY, zeroOffsetY * 2);
  }
  char title1[25] = "";
  strcpy(title1, title);
  float* curveCoeff = NULL;
  Point keyPoint;
  PlotInfo plot_info1 = {title1,    original_arr, &keyPoint, 0,
                         winFrames, BLUE,         false,     curveCoeff};
  char title2[1] = "";
  PlotInfo plot_info2 = {title2,    loopback_arr, &keyPoint, 0,
                         winFrames, RED,          false,     curveCoeff};
  showXYGraph(chartConfig, zeroOffsetY, &plot_info1);
  showXYGraph(chartConfig, zeroOffsetY, &plot_info2);
  return;
}

// following functions beckup
// delay 100ms to eliminate misoperation caused by long press
bool inArea(int mx, int my, int x, int y, int w, int h) {
  return (mx > x && mx < x + w && my > y && my < y + h);
}
bool buttonStateSwitch(const ExMessage* msg,
                       int x,
                       int y,
                       int w,
                       int h,
                       bool* swt,
                       const char* txtOn,
                       const char* txtOff) {
  bool ans =
      msg->message == WM_LBUTTONDOWN && inArea(msg->x, msg->y, x, y, w, h);
  if (ans) {
    *swt = !(*swt);
    Sleep(100);
  }
  const char* text = (*swt) ? txtOn : txtOff;
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

bool buttonOneStep(ExMessage* msg,
                   int x,
                   int y,
                   int w,
                   int h,
                   const char* text) {
  bool ans =
      msg->message == WM_LBUTTONDOWN && inArea(msg->x, msg->y, x, y, w, h);
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

int functionButton(ExMessage msg) {
  if (buttonStateSwitch(&msg, s_infoAreaBoundary - 2, 50, 60, 30,
                        &g_showPredictSwt, "Pred On", "Pred Off")) {
    return 1;
  } else if (buttonOneStep(&msg, s_infoAreaBoundary - 2, 80, 60, 30, "Prev")) {
    return 2;
  } else if (buttonOneStep(&msg, s_infoAreaBoundary - 2, 110, 60, 30, "Next")) {
    return 3;
  } else {
    return 0;
  }
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

/// @brief display reserved or backup signals to screen, could be customized
/// @param reservedInfo
void drawReservedInfo(const ReservedInfo* reservedInfo) {
  // TODO: name and type of signals could be customized
  // set switch to TRUE to show reserved signals
  bool displayReserveInfoSwitch = false;
  if (!displayReserveInfoSwitch) {
    return;
  }
  // str length should not exeed 11
  char reseved1_title[12] = "R1: ";
  char reseved2_title[12] = "R2: ";
  char reseved3_title[12] = "R3: ";
  char reseved4_title[12] = "R4: ";

  snprintf(reseved1_title + strlen(reseved1_title),
           sizeof(reseved1_title) - strlen(reseved1_title), "%.2f",
           reservedInfo->reserved1);
  outtextxy(s_infoAreaBoundary, s_origin2.y + 2 * textheight(reseved1_title),
            reseved1_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y + 3 * textheight(reseved2_title),
            reseved2_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y + 4 * textheight(reseved3_title),
            reseved3_title);
  outtextxy(s_infoAreaBoundary, s_origin2.y + 5 * textheight(reseved4_title),
            reseved4_title);
  return;
}