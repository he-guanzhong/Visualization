#include "visualization/extension_package/ext_rem_tools.h"

void drawDpPoints(const Point* pointsInfo, const int nums, const int color) {
  for (int j = 0; j < nums; j++) {
    Point obj_posn = {pointsInfo[j].x, pointsInfo[j].y};
    coordinateTrans2(&obj_posn);
    setlinecolor(BLACK);
    setfillcolor(color);
    solidcircle(obj_posn.x, obj_posn.y, 5);
  }
  return;
}

void drawRemDpInfo(const GraphConfig* config,
                   const RemInfo* remInfo,
                   const int egoOrgColor,
                   const int tarOrgColor,
                   const int egoOffColor,
                   const int tarOffColor) {
  const int infoAreaLeftBdy[2] = {config->oriX + config->length - 150,
                                  config->oriX + config->length - 70};
  const int infoAreaUpBdy[2] = {160, 280};

  // set spd display, only 5 characters in title is acceptable
  const char ego_org_title[10] = "Ego Org";
  const char ego_off_title[10] = "Ego Off";
  const char tar_org_title[10] = "Tar Org";
  const char tar_off_title[10] = "Tar Off";

  char ego_org_coeff[4][12] = {"C0: ", "C1: ", "C2: ", "C3: "};
  char tar_org_coeff[4][12] = {"C0: ", "C1: ", "C2: ", "C3: "};
  char ego_off_coeff[4][12] = {"", "", "", ""};
  char tar_off_coeff[4][12] = {"", "", "", ""};
  // char ego_off_coeff[4][10] = {"C0: ", "C1: ", "C2: ", "C3: "};
  // char tar_off_coeff[4][10] = {"C0: ", "C1: ", "C2: ", "C3: "};
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

  for (int i = 0; i < 4; i++) {
    snprintf(ego_org_coeff[i] + coeff_len1,
             sizeof(ego_org_coeff[i]) - coeff_len1, "%.3f",
             remInfo->ego_dp_org[i]);
    snprintf(tar_org_coeff[i] + coeff_len1,
             sizeof(tar_org_coeff[i]) - coeff_len1, "%.3f",
             remInfo->tar_dp_org[i]);
    snprintf(ego_off_coeff[i] + coeff_len2,
             sizeof(ego_off_coeff[i]) - coeff_len2, "%.3f",
             remInfo->ego_dp_off[i]);
    snprintf(tar_off_coeff[i] + coeff_len2,
             sizeof(tar_off_coeff[i]) - coeff_len2, "%.3f",
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
  return;
}

void showRemGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const LinesInfo* linesInfo,
                  const MotionInfo* motionInfo,
                  const RemInfo* remInfo) {
  initBEVGraph(config, zeroOffsetX);

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
  const int egoOffColor = LIGHTBLUE;
  const int tarOffColor = RGB(208, 159, 126);

  if (remInfo->ego_dp_org[0]) {
    setlinestyle(PS_SOLID, 2);
    drawQuinticPolyTraj(remInfo->ego_dp_org, egoOrgColor,
                        remInfo->ego_dp_org[6], remInfo->ego_dp_org[7],
                        remInfo->ego_dp_org[7], &lineEnd);
  }
  if (remInfo->tar_dp_org[0]) {
    setlinestyle(PS_SOLID, 2);
    drawQuinticPolyTraj(remInfo->tar_dp_org, tarOrgColor,
                        remInfo->tar_dp_org[6], remInfo->tar_dp_org[7],
                        remInfo->tar_dp_org[7], &lineEnd);
  }

  // REM offline ego / tar DP lines
  if (remInfo->ego_dp_off[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(remInfo->ego_dp_off, egoOffColor,
                        remInfo->ego_dp_off[6], remInfo->ego_dp_off[7],
                        remInfo->ego_dp_off[7], &lineEnd);
  }
  if (remInfo->tar_dp_off[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(remInfo->tar_dp_off, tarOffColor,
                        remInfo->tar_dp_off[6], remInfo->tar_dp_off[7],
                        remInfo->tar_dp_off[7], &lineEnd);
  }

  // REM dp lines text info
  drawRemDpInfo(config, remInfo, egoOrgColor, tarOrgColor, egoOffColor,
                tarOffColor);

  // REM dp points
  setlinestyle(PS_SOLID, 1);
  drawDpPoints(remInfo->ego_dp_point, remInfo->point_nums, egoOrgColor);
  drawDpPoints(remInfo->tar_dp_point, remInfo->point_nums, egoOffColor);

  // PA path, c0 ~ c3
  setlinestyle(PS_DASHDOT, 1);
  drawPiecewiseCubicPolyTraj(&linesInfo->ego_coeffs, MAGENTA, 0.0f,
                             &predictPosn);
  drawQuinticPolyTraj(linesInfo->alc_coeffs, LIGHTRED, naviRange,
                      fmaxf(50.0f, naviRange), fmaxf(50.0f, naviRange),
                      &predictPosn);
  drawQuinticPolyTraj(linesInfo->alc_coeffs, RED, 0.0f, naviRange, 120.0f,
                      &predictPosn);

  // ego car
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
  strCompletion(str_ego, 10, motionInfo->egoSpd);
  drawCar(&ego, str_ego, 1, 0, 10);

  // ego spd info and lane change status
  // drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);
  return;
}
