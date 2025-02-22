#include "visualization/extension_package/ext_rem_tools.h"

void drawDpPoints(const Point* pointsInfo, const int nums, const int color) {
  for (int j = 0; j < nums; j++) {
    Point obj_posn = {pointsInfo[j].x, pointsInfo[j].y};
    coordinateTrans2(&obj_posn);
    setlinecolor(BLACK);
    setfillcolor(color);
    solidcircle(obj_posn.x, obj_posn.y, 2);
  }
  return;
}

void drawRemDpInfo(const GraphConfig* config,
                   const RemInfo* remInfo,
                   const int egoOrgColor,
                   const int tarOrgColor,
                   const int egoOffColor,
                   const int tarOffColor) {
  const int infoAreaLeftBdy[2] = {config->oriX + config->length - 180,
                                  config->oriX + config->length - 80};
  const int infoAreaUpBdy[2] = {160, 280};
  const int infoAreaUpUsage[2] = {400, 420};

  // set spd display, only 5 characters in title is acceptable
  const char ego_org_title[10] = "Ego Org";
  const char ego_off_title[10] = "Ego Off";
  const char tar_org_title[10] = "Tar Org";
  const char tar_off_title[10] = "Tar Off";

  char ego_org_coeff[4][15] = {"C0: ", "C1: ", "C2: ", "C3: "};
  char tar_org_coeff[4][15] = {"C0: ", "C1: ", "C2: ", "C3: "};
  char ego_off_coeff[4][15] = {"", "", "", ""};
  char tar_off_coeff[4][15] = {"", "", "", ""};
  char ego_usage_title[15] = "Ego Usage: ";
  char tar_usage_title[15] = "Tar Usage: ";
  const int coeff_len1 = strlen(ego_org_coeff[0]);
  const int coeff_len2 = strlen(ego_off_coeff[0]);
  const int text_height = textheight(ego_org_coeff[0]);
  settextcolor(egoOrgColor);
  outtextxy(infoAreaLeftBdy[0], infoAreaUpBdy[0], ego_org_title);
  settextcolor(tarOrgColor);
  outtextxy(infoAreaLeftBdy[0], infoAreaUpBdy[1], tar_org_title);
  settextcolor(egoOffColor);
  outtextxy(infoAreaLeftBdy[1], infoAreaUpBdy[0], ego_off_title);
  settextcolor(tarOffColor);
  outtextxy(infoAreaLeftBdy[1], infoAreaUpBdy[1], tar_off_title);

  snprintf(ego_usage_title + 10, sizeof(ego_usage_title), "%d",
           remInfo->dp_usage[0]);
  snprintf(tar_usage_title + 10, sizeof(tar_usage_title), "%d",
           remInfo->dp_usage[1]);
  int usageColor = BLACK;
  settextcolor(usageColor);
  outtextxy(infoAreaLeftBdy[0], infoAreaUpUsage[0], ego_usage_title);
  settextcolor(usageColor);
  outtextxy(infoAreaLeftBdy[0], infoAreaUpUsage[1], tar_usage_title);

  for (int i = 0; i < 4; i++) {
    snprintf(ego_org_coeff[i] + coeff_len1,
             sizeof(ego_org_coeff[i]) - coeff_len1, "%.6f",
             remInfo->ego_dp_org[i]);
    snprintf(tar_org_coeff[i] + coeff_len1,
             sizeof(tar_org_coeff[i]) - coeff_len1, "%.6f",
             remInfo->tar_dp_org[i]);
    snprintf(ego_off_coeff[i] + coeff_len2,
             sizeof(ego_off_coeff[i]) - coeff_len2, "%.6f",
             remInfo->ego_dp_off[i]);
    snprintf(tar_off_coeff[i] + coeff_len2,
             sizeof(tar_off_coeff[i]) - coeff_len2, "%.6f",
             remInfo->tar_dp_off[i]);
    settextcolor(egoOrgColor);
    outtextxy(infoAreaLeftBdy[0], infoAreaUpBdy[0] + text_height * (i + 1),
              ego_org_coeff[i]);
    settextcolor(tarOrgColor);
    outtextxy(infoAreaLeftBdy[0], infoAreaUpBdy[1] + text_height * (i + 1),
              tar_org_coeff[i]);
    settextcolor(egoOffColor);
    outtextxy(infoAreaLeftBdy[1], infoAreaUpBdy[0] + text_height * (i + 1),
              ego_off_coeff[i]);
    settextcolor(tarOffColor);
    outtextxy(infoAreaLeftBdy[1], infoAreaUpBdy[1] + text_height * (i + 1),
              tar_off_coeff[i]);
  }
  settextcolor(BLACK);
}

void drawRemRuler(const float zeroOffsetX,
                  const float rangeX,
                  const float rangeY) {
  settextcolor(BLACK);
  const float coordinate_x_of_ruler_y = fminf(-4.0f, -zeroOffsetX);
  Point ruler_x[4] = {{-rangeX * 0.2f, -rangeY / 2.0f},
                      {0.0f, -rangeY / 2.0f},
                      {rangeX * 0.2f, -rangeY / 2.0f},
                      {rangeX, -rangeY / 2.0f}};
  Point ruler_y[2] = {{coordinate_x_of_ruler_y, -rangeY / 2.0f},
                      {coordinate_x_of_ruler_y, rangeY / 2.0f}};
  char str[7] = "";
  for (int i = 0; i < 4; ++i) {
    snprintf(str, sizeof(str), "%.1f m", ruler_x[i].x);
    coordinateTrans2(&ruler_x[i]);
    line(ruler_x[i].x, ruler_x[i].y, ruler_x[i].x - 10, ruler_x[i].y);
    outtextxy(ruler_x[i].x, ruler_x[i].y - textheight(str) / 2, str);
  }
  for (int i = 0; i < 2; i++) {
    snprintf(str, sizeof(str), "%.1f m", ruler_y[i].y);
    coordinateTrans2(&ruler_y[i]);
    line(ruler_y[i].x, ruler_y[i].y, ruler_y[i].x, ruler_y[i].y + 10);
    outtextxy(ruler_y[i].x - textwidth(str) / 2, ruler_y[i].y + 10, str);
  }
}

void showRemGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const LinesInfo* linesInfo,
                  const MotionInfo* motionInfo,
                  const RemInfo* remInfo) {
  initBEVGraph(config, zeroOffsetX);

  // ego car
  setlinecolor(RED);
  setlinestyle(PS_SOLID);
  Point ego = {0.0f, 0};
  char str_ego[2][8] = {};
  strcpy(str_ego[0], "ego");
  strCompletion(str_ego, 10, motionInfo->egoSpd);
  setfillcolor(WHITE);
  drawCar(&ego, str_ego, 1, 0, 10);
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

  // navigation path, ego c7 as end point
  const float naviRange = linesInfo->alc_coeffs[7];
  Point predictPosn = {0.0f, 0.0f};

  //  REM origin ego / tar DP lines
  const int egoOrgColor = BLUE;
  const int tarOrgColor = BROWN;
  const int egoOffColor = GREEN;
  const int tarOffColor = RGB(208, 159, 126);

  // if (remInfo->ego_dp_org[0]) {
  //   setlinestyle(PS_SOLID, 2);
  //   drawQuinticPolyTraj(remInfo->ego_dp_org, egoOrgColor,
  //                       remInfo->ego_dp_org[6], remInfo->ego_dp_org[7],
  //                       remInfo->ego_dp_org[7], &lineEnd);
  // }
  // if (remInfo->tar_dp_org[0]) {
  //   setlinestyle(PS_SOLID, 2);
  //   drawQuinticPolyTraj(remInfo->tar_dp_org, tarOrgColor,
  //                       remInfo->tar_dp_org[6], remInfo->tar_dp_org[7],
  //                       remInfo->tar_dp_org[7], &lineEnd);
  // }

  // REM offline ego / tar DP lines
  if (remInfo->ego_dp_off[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(remInfo->ego_dp_off, egoOffColor,
                        remInfo->ego_dp_off[8], remInfo->ego_dp_off[9],
                        remInfo->ego_dp_off[9], &lineEnd);
  }
  if (remInfo->tar_dp_off[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(remInfo->tar_dp_off, tarOffColor,
                        remInfo->tar_dp_off[8], remInfo->tar_dp_off[9],
                        remInfo->tar_dp_off[9], &lineEnd);
  }

  // REM dp lines text info
  drawRemDpInfo(config, remInfo, egoOrgColor, tarOrgColor, egoOffColor,
                tarOffColor);

  // REM dp points
  setlinestyle(PS_SOLID, 1);
  drawDpPoints(remInfo->ego_dp_point, remInfo->ego_point_nums, egoOrgColor);
  drawDpPoints(remInfo->tar_dp_point, remInfo->tar_point_nums, egoOffColor);

  // PA path, c0 ~ c3
  // setlinestyle(PS_DASHDOT, 1);
  // drawPiecewiseCubicPolyTraj(&linesInfo->ego_coeffs, MAGENTA, 0.0f,
  //                            &predictPosn);
  // drawQuinticPolyTraj(linesInfo->alc_coeffs, LIGHTRED, naviRange,
  //                     fmaxf(50.0f, naviRange), fmaxf(50.0f, naviRange),
  //                     &predictPosn);
  // drawQuinticPolyTraj(linesInfo->alc_coeffs, RED, 0.0f, naviRange, 120.0f,
  //                     &predictPosn);

  // ego spd info and lane change status
  // drawMotionInfo(motionInfo);
  drawRemRuler(zeroOffsetX, config->rangeX, config->rangeY);
  return;
}

void ShowRemInterface(const GraphConfig* config,
                      const float zeroOffsetX,
                      const LinesInfo* linesInfo,
                      const MotionInfo* motionInfo,
                      const RemInfo* remInfo,
                      const ExMessage* msg,
                      const float* time_data,
                      const int curFrame,
                      const int totalFrame) {
#ifdef REM_DEMO_TEST
  static bool s_ego_tar_swt = true;

  GraphConfig RemBevConfig = *config;
  RemBevConfig.length /= 2;
  // RemBevConfig.width -= 150;
  showRemGraph(&RemBevConfig, 0, linesInfo, motionInfo, remInfo);

  /* Min RangeY of Coefficients. e.g. 0.6 = [-0.3, 0.3] */
  const float rangeY_c[4] = {0.6f, 0.06f, 0.003f, 0.0001f};
  const int winFrames = 61;

  GraphConfig chartConfig = {config->length / 4,
                             config->width / 2,
                             config->offset / 2,
                             config->length / 2,
                             0,
                             0,
                             rangeY_c[0],
                             4,
                             6};

  int infoAreaBoundary = config->oriX + config->length - 65;
  buttonStateSwitch(msg, 10, 50, 60, 30, &s_ego_tar_swt, "Ego", "Tar");

  if (s_ego_tar_swt) {
    showLineChart(&chartConfig, rangeY_c[0] / 2, curFrame, totalFrame,
                  winFrames, time_data, ego_dp_org_data[0], ego_dp_off_data[0],
                  "Ego C0", false);

    chartConfig.oriX = config->length * 0.75f;
    chartConfig.rangeY = rangeY_c[1];
    showLineChart(&chartConfig, rangeY_c[1] / 2, curFrame, totalFrame,
                  winFrames, time_data, ego_dp_org_data[1], ego_dp_off_data[1],
                  "Ego C1", false);

    chartConfig.oriX = config->length * 0.5f;
    chartConfig.oriY = chartConfig.width;
    chartConfig.rangeY = rangeY_c[2];
    showLineChart(&chartConfig, rangeY_c[2] / 2, curFrame, totalFrame,
                  winFrames, time_data, ego_dp_org_data[2], ego_dp_off_data[2],
                  "Ego C2", false);

    chartConfig.oriX = config->length * 0.75f;
    chartConfig.rangeY = rangeY_c[3];
    showLineChart(&chartConfig, rangeY_c[3] / 2, curFrame, totalFrame,
                  winFrames, time_data, ego_dp_org_data[3], ego_dp_off_data[3],
                  "Ego C3", false);

  } else {
    showLineChart(&chartConfig, rangeY_c[0] / 2, curFrame, totalFrame,
                  winFrames, time_data, tar_dp_org_data[0], tar_dp_off_data[0],
                  "Tar C0", false);

    chartConfig.oriX = config->length * 0.75f;
    chartConfig.rangeY = rangeY_c[1];
    showLineChart(&chartConfig, rangeY_c[1] / 2, curFrame, totalFrame,
                  winFrames, time_data, tar_dp_org_data[1], tar_dp_off_data[1],
                  "Tar C1", false);

    chartConfig.oriX = config->length * 0.5f;
    chartConfig.oriY = chartConfig.width;
    chartConfig.rangeY = rangeY_c[2];
    showLineChart(&chartConfig, rangeY_c[2] / 2, curFrame, totalFrame,
                  winFrames, time_data, tar_dp_org_data[2], tar_dp_off_data[2],
                  "Tar C2", false);

    chartConfig.oriX = config->length * 0.75f;
    chartConfig.rangeY = rangeY_c[3];
    showLineChart(&chartConfig, rangeY_c[3] / 2, curFrame, totalFrame,
                  winFrames, time_data, tar_dp_org_data[3], tar_dp_off_data[3],
                  "Tar C3", false);
  }

#endif

  return;
}
