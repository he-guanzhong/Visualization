#include "visualization/show_basic_tools.h"

#define BEZIER_SWITCH 0
bool show_predict_swt = false;
float drawP[6][2];
float fit_coeffi[6];

// Draw graph origins
Point g_origin1 = {0.0f, 0.0f};
float g_xScale1 = 0, g_yScale1 = 0;
Point g_origin2 = {0.0f, 0.0f};
float g_xScale2 = 0, g_yScale2 = 0;

int combination(int n, int k) {
  float result = 1.0f;
  for (int i = 1; i <= k; i++)
    result *= (n - i + 1.0f) / i;
  return result;
}
void bezierPoint(float tau, int n, float points[][2], float* x, float* y) {
  *x = 0, *y = 0;
  if (n < 1)
    return;
  float t = tau / (float)n;
  for (int i = 0; i <= n; i++) {
    *x += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][0];
    *y += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][1];
  }
}
void bezierDerivative(float tau,
                      int n,
                      float points[][2],
                      float* dx,
                      float* dy) {
  *dx = 0, *dy = 0;
  if (n < 2)
    return;
  float t = tau / (float)n;
  for (int i = 0; i < n; i++) {
    *dx += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][0] - points[i][0]);
    *dy += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][1] - points[i][1]);
  }
  printf("t = %.2f, dx = %.2f, dy = %.2f\n", t, *dx, *dy);
}
void bezierSecDerivative(float tau,
                         int n,
                         float points[][2],
                         float* ddx,
                         float* ddy) {
  *ddx = 0, *ddy = 0;
  if (n < 3)
    return;
  float t = tau / (float)n;
  for (int i = 0; i < n - 1; i++) {
    float dx = n * (points[i + 2][0] - 2 * points[i + 1][0] + points[i][0]);

    float dy = n * (points[i + 2][1] - 2 * points[i + 1][1] + points[i][1]);
    *ddx += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dx;
    *ddy += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dy;
  }
  printf("t = %.2f, ddx = %.6f, ddy = %.6f\n", t, *ddx, *ddy);
}

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]) {
  int n = MAT_SIZE;
  for (int i = 0; i < n; i++) {
    // find largest absolute value of a[i][i] as main element
    int maxElem = i;
    for (int k = i + 1; k < n; k++) {
      if (fabs(a[k][i]) > fabs(a[maxElem][i]))
        maxElem = k;
    }
    // swap row of main element to first row
    if (maxElem != i) {
      for (int j = i; j <= n; j++) {
        float tmp = a[i][j];
        a[i][j] = a[maxElem][j];
        a[maxElem][j] = tmp;
      }
    }
    // simplify elements of [i+1,n) to 0. Upper triangular matrix converting
    for (int k = i + 1; k < n; k++) {
      float factor = a[k][i] / a[i][i];
      for (int j = i; j <= n; j++) {
        a[k][j] -= factor * a[i][j];
      }
    }
  }
  // regression process
  for (int i = n - 1; i >= 0; i--) {
    a[i][n] /= a[i][i];
    for (int k = i - 1; k >= 0; k--) {
      a[k][n] -= a[k][i] * a[i][n];
    }
  }
}

void quinticPolyFit(float T,
                    float s0,
                    float v0,
                    float a0,
                    float s1,
                    float v1,
                    float a1,
                    float coeffi[MAT_SIZE]) {
  if (MAT_SIZE != 6)
    return;
  // fifth degree polynominal fitting, set augmented matrix
  // s = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  float mat_A[MAT_SIZE][MAT_SIZE + 1] = {
      {1, 0, 0, 0, 0, 0, s0},
      {0, 1, 0, 0, 0, 0, v0},
      {0, 0, 2, 0, 0, 0, a0},
      {1, T, T * T, T * T * T, T * T * T * T, T * T * T * T * T, s1},
      {0, 1, 2 * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T, v1},
      {0, 0, 2, 6 * T, 12 * T * T, 20 * T * T * T, a1}};

  gaussianElimination(mat_A);
  for (int i = 0; i < MAT_SIZE; i++) {
    fit_coeffi[i] = mat_A[i][MAT_SIZE];
  }
  /*   printf("ST coeffi: ");
    for (int i = 0; i < MAT_SIZE; i++) {
      printf("a[%d] = %.2f\t", i, mat_A[i][MAT_SIZE]);
    }
    printf("\n"); */
  return;
}

void coordinateTrans1(Point* point) {
  point->x = g_origin1.x + point->x * g_xScale1;
  point->y = g_origin1.y - point->y * g_yScale1;
}

void coordinateTrans2(Point* point) {
  float tmp = point->x;
  point->x = g_origin2.x - point->y * g_xScale2;
  point->y = g_origin2.y - tmp * g_yScale2;
}

void strCompletion(char* str, const int index, const int spd) {
  char szIndex[2];
  itoa(index, szIndex, 10);
  strcat(str, szIndex);
  strcat(str, ": ");
  char szSpd[3];
  itoa(spd, szSpd, 10);
  strcat(str, szSpd);
  strcat(str, " m/s");
}

void drawCar(Point* car, const char* str, int carType, const float yaw) {
  // ObjType of ME: 0=UNFILLED, 1=CAR, 2=TRUCK, 3=MOTORBIKE, 4=BICYCLE,
  // 5=PEDESTRIAN, 6=GENERAL_OBJECT, 7=ANIMAL 8=UNCERTAIN_VCL
  if (carType > 3)
    carType = 0;
  float carLen_table[4] = {5.0f, 5.0f, 7.6f, 2.5f};
  float carWid_table[4] = {1.8f, 1.8f, 2.1f, 0.6f};
  float carLen = carLen_table[carType];
  float carWid = carWid_table[carType];
  // display: left-hand system. control: right-hand system
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
  /*   fillrectangle(car->x - carWid / 2, car->y - carLen / 2, car->x + carWid /
     2, car->y + carLen / 2); */
  fillpolygon(vertices_show, 4);
  outtextxy(car->x - textwidth(str) / 2, car->y + textheight(str) / 2, str);
}

void drawPolygon(const Point* center, const int num, const float rotateDegree) {
  int r = 15;
  int vertex_x = r, vertex_y = 0;
  POINT vertices[num];
  for (int i = 0; i < num; i++) {
    vertex_x = center->x + r * cos(2 * M_PI / num * i + rotateDegree);
    vertex_y = center->y + r * sin(2 * M_PI / num * i + rotateDegree);
    vertices[i] = {vertex_x, vertex_y};
  }
  polygon(vertices, num);
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
  char str_spd[5];
  char str_set[5];
  int vDis = round(spd_info->cur_spd * 3.6f);
  int vSetDis = round(spd_info->set_spd * 3.6f);
  itoa(vDis, str_spd, 10);
  itoa(vSetDis, str_set, 10);
  strcat(spd_title, str_spd);
  strcat(set_title, str_set);
  // ACC mode: 3-stand still, 4-stand active, 5-active, 6-override
  if (spd_info->acc_mode <= 5 && spd_info->acc_mode >= 3)
    settextcolor(GREEN);
  else if (spd_info->acc_mode == 6)
    settextcolor(RED);
  else
    settextcolor(BLACK);

  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y - textheight(spd_title),
            spd_title);
  if (spd_info->acc_mode <= 6 && spd_info->acc_mode >= 3)
    outtextxy(g_origin2.x + 12 * g_xScale2,
              g_origin2.y - 2 * textheight(spd_title), set_title);

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

  outtextxy(g_origin2.x + 12 * g_xScale2, g_origin2.y - 100, alc_side);
  outtextxy(g_origin2.x + 12 * g_xScale2,
            g_origin2.y - 100 + textheight(alc_side), alc_sts);
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
                 Point* ctrlPoint) {
  // float zeroOffsetY = 4.0f;  // positive means: 0 moves up
  float len = length - 2 * offset;
  float wid = width - 2 * offset;

  // x = horizental, y = vertical
  g_origin1 = {(float)(oriX + offset), (float)(oriY + offset + wid)};
  g_xScale1 = len / rangeX;
  g_yScale1 = wid / rangeY;

  drawBasicGraph(len, wid, rangeX, rangeY, zeroOffsetY);

  //  titles
  settextcolor(BLACK);
  settextstyle(30, 0, "Calibri");
  outtextxy(g_origin1.x + len / 2 - textwidth(title) / 2,
            g_origin1.y - wid - textheight(title), title);
  settextstyle(20, 0, "Calibri", 900, 900, 0, 0, 0, 0);

  char titleY[10];
  if (title[0] == 'A')
    strcpy(titleY, "A (m/s2)");
  else if (title[0] == 'V')
    strcpy(titleY, "V (m/s)");
  else
    strcpy(titleY, "S (m)");

  outtextxy(g_origin1.x - 50, g_origin1.y - wid / 2, titleY);
  settextstyle(20, 0, "Calibri", 0, 0, 0, 0, 0, 0);
  char titleX[] = "T (s)";
  outtextxy(g_origin1.x + (len - textwidth(titleX)) / 2,
            g_origin1.y + textheight(titleX), titleX);

#if BEZIER_SWITCH == 1
  // bezier curve from result point
  /*   memset(drawP, 0, sizeof(drawP));
    for (int i = 0; i <= 5; i++) {
      drawP[i][0] = points[i].x;
      drawP[i][1] = points[i].y;
    } */
  if (title[0] == 'S') {
    memset(drawP, 0, sizeof(drawP));
    for (int i = 0; i <= 5; i++) {
      drawP[i][0] = points[i].x;
      drawP[i][1] = points[i].y;
    }
  }
  Point curDrawP = {0, 0};
  Point lastDrawP = points[0];
  coordinateTrans1(&lastDrawP);
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

    curDrawP.y += zeroOffsetY;
    coordinateTrans1(&curDrawP);
    line(lastDrawP.x, lastDrawP.y, curDrawP.x, curDrawP.y);
    lastDrawP = curDrawP;
  }
#endif
  Point curDrawP = {0, 0};
  Point lastDrawP = points[0];
  coordinateTrans1(&lastDrawP);
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

  // result points
  setfillcolor(pointColor);
  for (int i = 0; i < 6; i++) {
    float val = points[i].y;
    points[i].y += zeroOffsetY;
    coordinateTrans1(&points[i]);
    solidcircle(points[i].x, points[i].y, 5);
    char str[4] = "";
    sprintf(str, "%.1f", val);
    outtextxy(points[i].x + 5, points[i].y + 5, str);
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
  setlinecolor(pointColor);
#if BEZIER_SWITCH == 0
  for (int i = 0; i < 5; i++) {
    line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
  }
#endif
}

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
                  const SpdInfo* spd_info) {
  float len = length - 2.0f * offset;
  float wid = width - 2.0f * offset;
  // vehicle frame: front-left-up, FLU. eg. x-longitudinal, y-lateral
  g_origin2 = {oriX + length * 0.5f, oriY + offset + wid};
  g_xScale2 = len / (3.4f * 5.0f);
  g_yScale2 = wid / rangeX;
  g_origin2.y -= g_yScale2 * zeroOffsetX;

  // title
  settextcolor(BLACK);
  settextstyle(30, 0, "Calibri");
  char title[] = " ";
  outtextxy(g_origin2.x - textwidth(title) / 2, offset - textheight(title),
            title);
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
    char str_obs_cur[20] = "obs";
    strCompletion(str_obs_cur, i, obs.speed_x);
    setlinecolor(BLACK);
    setfillcolor(DARKGRAY);
    drawCar(&obs_cur, str_obs_cur, obs.type, obs.pos_yaw);

    Point obs_pred = {obs.pos_x + obs.speed_x * 5.0f,
                      obs.pos_y + obs.speed_y * 5.0f};
    if (show_predict_swt && obs.speed_x > 1.0f) {
      char str_obs_pred[20] = "obs_pred";
      setfillcolor(LIGHTGRAY);
      strCompletion(str_obs_pred, i, obs.speed_x);
      drawCar(&obs_pred, str_obs_pred, obs.type, obs.pos_yaw);
      setlinecolor(LIGHTGRAY);
      setlinestyle(PS_DASH);
      line(obs_cur.x, obs_cur.y, obs_pred.x, obs_pred.y);
    }
  }

  // navigation path, ego c7 as end point
  float naviRange = lines_info->ego_coeffs[7];
  Point predictPosn = {0.0f, 0.0f};
  drawTrajectory(lines_info->ego_coeffs, RED, 0.0f, naviRange, &predictPosn);

  // ego car
  setfillcolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, lines_info->ego_coeffs[5]};
  char str_ego[20] = "ego";
  strCompletion(str_ego, 0, spd_info->cur_spd);
  drawCar(&ego, str_ego, 1, 0.0f);

  if (show_predict_swt && predictPosn.x > 2.0f) {
    // setfillstyle(BS_HATCHED, HS_DIAGCROSS);
    Point ego_pred = {predictPosn.x, predictPosn.y};
    char str_ego_pred[20] = "ego_pred";
    strCompletion(str_ego_pred, 0, spd_info->pred_spd);
    drawCar(&ego_pred, str_ego_pred, 1, 0);
    // setfillstyle(BS_SOLID);
  }
  // ego spd info and lane change status
  drawMotionInfo(spd_info);
  drawBEVRuler();
}

// following functions beckup
bool inArea(int mx, int my, int x, int y, int w, int h) {
  return (mx > x && mx < x + w && my > y && my < y + h);
}
bool button(ExMessage* msg, int x, int y, int w, int h, bool* swt) {
  bool ans =
      msg->message == WM_LBUTTONDOWN && inArea(msg->x, msg->y, x, y, w, h);
  if (ans) {
    *swt = !(*swt);
  }
  const char* text = (*swt) ? "Pred On" : "Pred Off";
  if (inArea(msg->x, msg->y, x, y, w, h))
    setfillcolor(CYAN);
  else
    setfillcolor(RGB(255, 255, 255));
  fillroundrect(x, y, x + w, y + h, 5, 5);
  settextcolor(BLACK);
  outtextxy(x + (w - textwidth(text)) / 2, y + (h - textheight(text)) / 2,
            text);
  return ans;
}
void functionButton(ExMessage msg) {
  setlinecolor(BLACK);
  // while (1) {
  button(&msg, g_origin2.x + 11 * g_xScale2, 50, 60, 30, &show_predict_swt);

  // BeginBatchDraw();
  //  cleardevice();

  // EndBatchDraw();
  // msg.message = 0;
  //}
  return;
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