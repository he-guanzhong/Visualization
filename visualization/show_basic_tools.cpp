#include "visualization/show_basic_tools.h"

#define CURVE_FITTING_TYPE 0
bool show_predict_swt = false;

// Draw graph origins
Point g_origin1 = {0.0f, 0.0f};
float g_xScale1 = 0, g_yScale1 = 0;
Point g_origin2 = {0.0f, 0.0f};
float g_xScale2 = 0, g_yScale2 = 0;

void coordinateTrans1(Point* point) {
  point->x = g_origin1.x + point->x * g_xScale1;
  point->y = g_origin1.y - point->y * g_yScale1;
}

void coordinateTrans2(Point* point) {
  float tmp = point->x;
  point->x = g_origin2.x - point->y * g_xScale2;
  point->y = g_origin2.y - tmp * g_yScale2;
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
  if (carType > 3)
    carType = 0;
  float car_len_tbl[4] = {2.2f + 0.4f, 5.0f + 0.4f, 6.8f + 0.4f, 2.5f};
  float car_wid_tbl[4] = {1.0f + 0.4f, 1.5f + 0.4f, 1.6f + 0.4f, 0.6f};
  float carLen = car_len_tbl[carType];
  float carWid = car_wid_tbl[carType];
  // display: left-hand system. control: right-hand system
  // vertice order: upper-right -> lower-right-> lower-left -> upper-left
  float halfLenCos = carLen / 2.0f * cos(yaw),
        halfLenSin = carLen / 2.0f * sin(yaw);
  float halfWidCos = carWid / 2.0f * cos(yaw),
        halfWidSin = carWid / 2.0f * sin(yaw);
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

  if (carType == 3) {  // motorbike outline
    rectangle(car->x - car_wid_tbl[1] / 2.0f * g_xScale2,
              car->y - car_len_tbl[1] / 2.0f * g_yScale2,
              car->x + car_wid_tbl[1] / 2.0f * g_xScale2,
              car->y + car_len_tbl[1] / 2.0f * g_yScale2);
    /*     int dir[5] = {1, 1, -1, -1, 1};
        for (int i = 0; i < 4; i++) {
          vertices_show[i].x +=
              car_wid_tbl[1] / car_wid_tbl[3] * g_yScale2 * dir[i];
          vertices_show[i].y +=
              car_len_tbl[1] / car_len_tbl[3] * g_xScale2 * dir[i + 1];
        }
        polygon(vertices_show, 4); */
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
    vertex_x = center->x + r * cos(2 * M_PI / num * i + rotateDegree);
    vertex_y = center->y + r * sin(2 * M_PI / num * i + rotateDegree);
    vertices_show[i] = {vertex_x, vertex_y};
  }
  polygon(vertices_show, num);
}

void drawTsrSign(const TsrInfo* tsr_info) {
  // TSR status display
  char tsr_disp[10] = "TSR: ";
  char spd_val[10];
  int tsr_spd = tsr_info->tsr_spd;
  bool tsr_spd_warn = tsr_info->tsr_spd_warn;
  itoa(tsr_spd, spd_val, 10);
  strcat(tsr_disp, spd_val);
  char tsi_disp[10];
  if (tsr_info->tsr_tsi[0] == 5)
    strcpy(tsi_disp, "stop");
  else if (tsr_info->tsr_tsi[0] == 6)
    strcpy(tsi_disp, "yield");
  else if (tsr_info->tsr_tsi[1] == 5)
    strcpy(tsi_disp, "no entry");
  if (tsr_spd_warn)
    settextcolor(RED);
  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y, tsr_disp);
  settextcolor(BLACK);
  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y + textheight(tsr_disp),
            tsi_disp);

  // TSR original input
  int tsr_spd_table1[14] = {10, 20, 30,  40,  50,  60,  70,
                            80, 90, 100, 110, 120, 130, 140};
  int tsr_spd_table2[15] = {5,  15, 25,  35,  45,  55,  65, 75,
                            85, 95, 105, 115, 125, 135, 145};
  for (int i = 0; i < 3; i++) {
    if (!tsr_info->tsr_signs[i].valid || tsr_info->tsr_signs[i].type < 0 ||
        tsr_info->tsr_signs[i].type > 210)
      continue;
    Point tsr_pos = {tsr_info->tsr_signs[i].pos_x,
                     tsr_info->tsr_signs[i].pos_y};
    coordinateTrans2(&tsr_pos);

    // TSI (traffic sign indication). 210 = stop, 168 = yield, 199 = no entry
    if (tsr_info->tsr_signs[i].type == 168) {
      drawPolygon(&tsr_pos, 3, M_PI / 2);
    } else if (tsr_info->tsr_signs[i].type == 210) {
      drawPolygon(&tsr_pos, 8, M_PI / 8);
      char stop_sign[] = "stop";
      outtextxy(tsr_pos.x - textwidth(stop_sign) / 2,
                tsr_pos.y - textheight(stop_sign) / 2, stop_sign);
    } else if (tsr_info->tsr_signs[i].type == 199) {
      setfillcolor(RED);
      solidcircle(tsr_pos.x, tsr_pos.y, 14);
      setfillcolor(WHITE);
      solidrectangle(tsr_pos.x - 8, tsr_pos.y - 3, tsr_pos.x + 8,
                     tsr_pos.y + 3);
    }

    // TSR Speed limit: 10~140, 5~145, cancel sign
    char tsr_sign[4];
    if (tsr_info->tsr_signs[i].type <= 13) {
      itoa(tsr_spd_table1[tsr_info->tsr_signs[i].type], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsr_info->tsr_signs[i].type >= 100 &&
               tsr_info->tsr_signs[i].type <= 114) {
      itoa(tsr_spd_table2[tsr_info->tsr_signs[i].type - 100], tsr_sign, 10);
      outtextxy(tsr_pos.x - textwidth(tsr_sign) / 2,
                tsr_pos.y - textheight(tsr_sign) / 2, tsr_sign);
      setlinecolor(RED);
      circle(tsr_pos.x, tsr_pos.y, textheight(tsr_sign) * 0.75f);
    } else if (tsr_info->tsr_signs[i].type == 79 ||
               tsr_info->tsr_signs[i].type == 80) {
      strcpy(tsr_sign, "Cnl");
      int r = textheight(tsr_sign) * 0.75f;
      int r_x = (double)r / sqrt(2);
      setlinecolor(BLACK);
      line(tsr_pos.x - r_x, tsr_pos.y + r_x, tsr_pos.x + r_x, tsr_pos.y - r_x);
      circle(tsr_pos.x, tsr_pos.y, r);
    }
  }
}

void drawMotionInfo(const SpdInfo* spd_info) {
  // set spd display
  char spd_title[10] = "Spd: ";
  char set_title[10] = "SET: ";
  char actual_set_title[10] = "Inn: ";
  char spec_case_title[10] = "Spc: ";

  char str_cur_spd[5];
  char str_disp_set_spd[5];
  char str_actual_set_spd[5];

  int cur_spd = round(spd_info->cur_spd * 1.03f * 3.6f);
  int disp_set_spd = round(spd_info->disp_set_spd);
  int actual_set_spd = round(spd_info->actual_set_spd);

  itoa(cur_spd, str_cur_spd, 10);
  itoa(disp_set_spd, str_disp_set_spd, 10);
  itoa(actual_set_spd, str_actual_set_spd, 10);
  strcat(spd_title, str_cur_spd);
  strcat(set_title, str_disp_set_spd);
  strcat(actual_set_title, str_actual_set_spd);

  // spd plan inner value
  switch (spd_info->spec_case_flg) {
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
      strcat(spec_case_title, "curv");
      break;
    case 4:
      strcat(spec_case_title, "ndg");
      break;
    case 5:
    case 15:
      strcat(spec_case_title, "dec");
      break;
    case 6:
    case 16:
      strcat(spec_case_title, "hold");
      break;
    case 7:
    case 17:
      strcat(spec_case_title, "acc");
      break;
    default:
      break;
  }

  outtextxy(g_origin2.x + 12 * g_xScale2,
            g_origin2.y - 4 * textheight(spd_title), spec_case_title);
  outtextxy(g_origin2.x + 12 * g_xScale2,
            g_origin2.y - 5 * textheight(spd_title), actual_set_title);

  // ACC mode: 3-stand still, 4-stand active, 5-active, 6-override
  if (spd_info->acc_mode <= 5 && spd_info->acc_mode >= 3)
    settextcolor(GREEN);
  else if (spd_info->acc_mode == 6)
    settextcolor(RED);
  else
    settextcolor(BLACK);

  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y - textheight(spd_title),
            spd_title);
  if (spd_info->acc_mode <= 6 && spd_info->acc_mode >= 3) {
    outtextxy(g_origin2.x + 12 * g_xScale2,
              g_origin2.y - 2 * textheight(spd_title), set_title);
  }

  char alc_side[8];
  memset(alc_side, '\0', sizeof(alc_side));
  if (spd_info->alc_side == 1)
    strcpy(alc_side, "ALC_L");
  else if (spd_info->alc_side == 2)
    strcpy(alc_side, "ALC_R");
  // alc sts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving,
  // 4-in target line, 5-finished,6-Back to Ego, 8-takeover, 9-popMsgReq
  char alc_sts[12];
  memset(alc_sts, '\0', sizeof(alc_sts));
  if (spd_info->alc_sts == 2)
    strcpy(alc_sts, "Hold");
  else if (spd_info->alc_sts == 3 || spd_info->alc_sts == 4)
    strcpy(alc_sts, "Changing");
  else if (spd_info->alc_sts == 6)
    strcpy(alc_sts, "Back");
  else if (spd_info->alc_sts == 8)
    strcpy(alc_sts, "Takeover");

  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y - 150, alc_side);
  outtextxy(g_origin2.x + 12 * g_xScale2,
            g_origin2.y - 150 + textheight(alc_side), alc_sts);
}

void drawTrajectory(const float* coeffs,
                    const int color,
                    const float startX,
                    const float lengthS,
                    Point* predictPosn) {
  setlinecolor(color);
  Point lastPoint = {0.0f, 0.0f};
  Point last = {0.0f, 0.0f};
  float restLen = lengthS;
  for (int i = startX; i <= lengthS; i++) {
    float j = coeffs[5] + coeffs[4] * i + coeffs[3] * i * i +
              coeffs[2] * i * i * i + coeffs[1] * i * i * i * i +
              coeffs[0] * i * i * i * i * i;
    if (restLen > 0) {
      if (last.x != 0.0f) {
        float delta_s = hypotf((float)i - last.x, (float)j - last.y);
        restLen -= delta_s;
      }
      last.x = i, last.y = j;
    }
    float curPoint_x = i, curPoint_y = j;
    Point curPoint = {curPoint_x, curPoint_y};
    coordinateTrans2(&curPoint);
    if (lastPoint.x != 0.0f)
      line(curPoint.x, curPoint.y, lastPoint.x, lastPoint.y);
    lastPoint = curPoint;
  }
  predictPosn->x = last.x;
  predictPosn->y = last.y;
}

void drawBEVRuler() {
  settextcolor(BLACK);
  Point ruler_x[4] = {
      {-30.0f, -6.0f}, {0.0f, -6.0f}, {50.0f, -6.0f}, {100.0f, -6.0f}};
  Point ruler_y[2] = {{-30.0f, -5.0f}, {-30.0f, 5.0f}};
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
  rectangle(g_origin1.x, g_origin1.y, g_origin1.x + len, g_origin1.y - wid);

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

/// @brief show bacis x-y graph
/// @param config       basic graph configuration
/// @param zeroOffsetY  vertical distance from origin to lower-left corner
/// @param title        title string
/// @param pointColor   macro name of color
/// @param points       point coordinates to be displayed
/// @param ctrlPoint    a-t graph only, accelerations sent to control
void showXYGraph(const GraphConfig* config,
                 const float zeroOffsetY,
                 const char* title,
                 const int pointColor,
                 Point* points,
                 const int startIndex,
                 const int pointNums,
                 Point* ctrlPoint) {
  // float zeroOffsetY = 4.0f;  // positive means: 0 moves up
  float len = config->length - 2 * config->offset;
  float wid = config->width - 2 * config->offset;

  // x = horizental, y = vertical
  g_origin1 = {(float)(config->oriX + config->offset),
               (float)(config->oriY + config->offset + wid)};
  g_xScale1 = len / config->rangeX;
  g_yScale1 = wid / config->rangeY;

  drawBasicGraph(len, wid, config->rangeX, config->rangeY, zeroOffsetY);

  //  titles
  settextcolor(BLACK);
  settextstyle(25, 0, "Calibri");
  outtextxy(g_origin1.x + len / 2 - textwidth(title) / 2,
            g_origin1.y - wid - textheight(title), title);
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

  outtextxy(g_origin1.x - 50, g_origin1.y - wid / 2, titleY);
  settextstyle(20, 0, "Calibri", 0, 0, 0, 0, 0, 0);
  char titleX[] = "T (s)";
  outtextxy(g_origin1.x + (len - textwidth(titleX)) / 2,
            g_origin1.y + textheight(titleX), titleX);

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
  for (float i = 0.0f; i < 2; i += 0.1f) {
    if (title[0] == 'S') {
      curDrawP = {i, fit_coeffi[0] + fit_coeffi[1] * i + fit_coeffi[2] * i * i +
                         fit_coeffi[3] * powf(i, 3) +
                         fit_coeffi[4] * powf(i, 4) +
                         fit_coeffi[5] * powf(i, 5)};
    } else if (title[0] == 'V') {
      curDrawP = {i, fit_coeffi[1] + 2 * fit_coeffi[2] * i +
                         3 * fit_coeffi[3] * i * i +
                         4 * fit_coeffi[4] * powf(i, 3) +
                         5 * fit_coeffi[5] * powf(i, 4)};
    } else if (title[0] == 'A') {
      curDrawP = {i, 2 * fit_coeffi[2] + 6 * fit_coeffi[3] * i +
                         12 * fit_coeffi[4] * i * i +
                         20 * fit_coeffi[5] * powf(i, 3)};
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

/// @brief BEV graph, x-axis_forward_vertical, y-axis_lateral_horizontal
/// @param config         basic configuration
/// @param zeroOffsetX    distance behind ego vehicle to be displayed
/// @param tsr_info       environment TSR info and ego TSR status
/// @param g_ssmFrameType obstacles info
/// @param lines_info     lane lines info
/// @param spd_info       ego vehicle motion status
void showBEVGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const TsrInfo* tsr_info,
                  const SsmFrameType* g_ssmFrameType,
                  const LinesInfo* lines_info,
                  const SpdInfo* spd_info) {
  float len = config->length - 2.0f * config->offset;
  float wid = config->width - 2.0f * config->offset;
  // vehicle frame: front-left-up, FLU. eg. x-longitudinal, y-lateral
  g_origin2 = {config->oriX + config->length * 0.5f,
               config->oriY + config->offset + wid};
  g_xScale2 = len / config->rangeY;
  g_yScale2 = wid / config->rangeX;
  g_origin2.y -= g_yScale2 * zeroOffsetX;

  // title
  settextcolor(BLACK);
  settextstyle(25, 0, "Calibri");
  char title[] = " ";
  outtextxy(g_origin2.x - textwidth(title) / 2,
            config->offset - textheight(title), title);
  settextstyle(20, 0, "Calibri");

  // tsr info
  drawTsrSign(tsr_info);

  // road lines
  setlinestyle(PS_DASHDOT);
  Point lineEnd;
  drawTrajectory(lines_info->left_coeffs, BLACK, lines_info->left_coeffs[6],
                 lines_info->left_coeffs[7], &lineEnd);
  drawTrajectory(lines_info->leftleft_coeffs, BLACK,
                 lines_info->leftleft_coeffs[6], lines_info->leftleft_coeffs[7],
                 &lineEnd);
  drawTrajectory(lines_info->right_coeffs, BLACK, lines_info->right_coeffs[6],
                 lines_info->right_coeffs[7], &lineEnd);
  drawTrajectory(lines_info->rightright_coeffs, BLACK,
                 lines_info->rightright_coeffs[6],
                 lines_info->rightright_coeffs[7], &lineEnd);

  drawTrajectory(lines_info->left_coeffs_me, GREEN,
                 lines_info->left_coeffs_me[6], lines_info->left_coeffs_me[7],
                 &lineEnd);
  drawTrajectory(lines_info->leftleft_coeffs_me, GREEN,
                 lines_info->leftleft_coeffs_me[6],
                 lines_info->leftleft_coeffs_me[7], &lineEnd);
  drawTrajectory(lines_info->right_coeffs_me, GREEN,
                 lines_info->right_coeffs_me[6], lines_info->right_coeffs_me[7],
                 &lineEnd);
  drawTrajectory(lines_info->rightright_coeffs_me, GREEN,
                 lines_info->rightright_coeffs_me[6],
                 lines_info->rightright_coeffs_me[7], &lineEnd);

  // obstacles
  for (int i = 0; i < g_ssmFrameType->Ssm_Objs_Frame_st.obj_num; i++) {
    if (!g_ssmFrameType->Ssm_Objs_Frame_st.obj_lists[i].valid_flag)
      continue;

    SsmObsType obs = g_ssmFrameType->Ssm_Objs_Frame_st.obj_lists[i];
    Point obs_cur = {obs.pos_x, obs.pos_y};
    char str_obs_cur[2][8] = {};
    strCompletion(str_obs_cur, i, obs.speed_x);
    setlinecolor(BLACK);
    setfillcolor(DARKGRAY);
    drawCar(&obs_cur, str_obs_cur, obs.type, obs.pos_yaw, i);

    Point obs_pred = {obs.pos_x + obs.speed_x * 5.0f,
                      obs.pos_y + obs.speed_y * 5.0f};

    // cipv, considier 0->1s const acc, 1->5s const spd
    if (obs.lane_index == 3 && obs.pos_x > 0) {
      float const_acc_time = 1.0f;
      obs_pred.x = obs.pos_x +
                   (obs.speed_x + obs.acc_x * const_acc_time) * 5.0f -
                   0.5f * obs.acc_x * const_acc_time * const_acc_time;
    }

    if (show_predict_swt && obs.speed_x > 1.0f) {
      char str_obs_pred[2][8] = {};
      strcpy(str_obs_pred[1], "Pred");
      setfillcolor(LIGHTGRAY);
      strCompletion(str_obs_pred, i, obs.speed_x);
      drawCar(&obs_pred, str_obs_pred, obs.type, obs.pos_yaw, i);
      setlinecolor(LIGHTGRAY);
      setlinestyle(PS_DASH);
      line(obs_cur.x, obs_cur.y, obs_pred.x, obs_pred.y);
    }
  }

  // navigation path, ego c7 as end point
  float naviRange = lines_info->ego_coeffs[7];
  Point predictPosn = {0.0f, 0.0f};
  drawTrajectory(lines_info->ego_coeffs, LIGHTRED, naviRange,
                 fmax(50.0f, naviRange), &predictPosn);
  drawTrajectory(lines_info->ego_coeffs, RED, 0.0f, naviRange, &predictPosn);

  // ego car
  setfillcolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, lines_info->ego_coeffs[5]};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  strCompletion(str_ego, 10, spd_info->cur_spd);
  drawCar(&ego, str_ego, 1, 0.0f, 10);

  if (show_predict_swt && predictPosn.x > 2.0f) {
    // setfillstyle(BS_HATCHED, HS_DIAGCROSS);
    Point ego_pred = {predictPosn.x, predictPosn.y};
    char str_ego_pred[2][8] = {};
    strcpy(str_ego_pred[0], "ego_pred");
    strCompletion(str_ego_pred, 10, spd_info->pred_spd);
    drawCar(&ego_pred, str_ego_pred, 1, 0, 10);
    // setfillstyle(BS_SOLID);
  }
  // ego spd info and lane change status
  drawMotionInfo(spd_info);
  drawBEVRuler();
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
  return button(&msg, g_origin2.x + 11 * g_xScale2, 50, 60, 30,
                &show_predict_swt);

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