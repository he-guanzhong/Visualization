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

void showRemGraph(const GraphConfig* config,
                  const float zeroOffsetX,
                  const LinesInfo* linesInfo,
                  const MotionInfo* motionInfo,
                  const RemPointsInfo* remPointsInfo) {
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

  // REM dp lines
  if (linesInfo->ego_dp[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(linesInfo->ego_dp, BLUE, linesInfo->ego_dp[6],
                        linesInfo->ego_dp[7], linesInfo->ego_dp[7], &lineEnd);
  }
  if (linesInfo->tar_dp[0]) {
    setlinestyle(PS_SOLID, 3);
    drawQuinticPolyTraj(linesInfo->tar_dp, BROWN, linesInfo->tar_dp[6],
                        linesInfo->tar_dp[7], linesInfo->tar_dp[7], &lineEnd);
  }
  // REM dp points
  drawDpPoints(remPointsInfo->egoDpPoint, remPointsInfo->nums, BLUE);
  drawDpPoints(remPointsInfo->tarDpPoint, remPointsInfo->nums, BROWN);

  // ego lane PA path, c0 ~ c3
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
  drawMotionInfo(motionInfo);
  drawBEVRuler(zeroOffsetX);
  return;
}
