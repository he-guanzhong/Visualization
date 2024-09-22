/* Autonomous Driving Data Visualization Tool
 * All rights reserved by hgz */
#include "visualization/show_main.h"

#ifdef SPEED_PLANNING_H_
extern bool show_predict_swt;
extern SsmObjType g_ssmObjType;
extern uint8 g_truncated_col;
extern float gInnerSpdLmt_kph;
extern uint8 gSpecialCaseFlg;
extern uint8 gScenarioFlg;
extern uint8 gGapIndex;
extern float gGapTarS;
extern float gGapTarV;
extern float gAlcStCoeff[6];
extern float gTempMeasureVal1;
extern float gTempMeasureVal2;
#else
SsmObjType g_ssmObjType;
uint8 g_truncated_col;
float gInnerSpdLmt_kph;
uint8 gSpecialCaseFlg;
uint8 gScenarioFlg;
uint8 gGapIndex;
float gGapTarS;
float gGapTarV;
float gAlcStCoeff[6];
float gTempMeasureVal1;
float gTempMeasureVal2;
#endif

// Key display information
MotionInfo motionInfo;
Point s_points[6], v_points[6], a_points[6];
Point ctrlPoint;
int spdPlanEnblSts;
// go straight
// ego_coeff[9]: c0~c2, c31~c32, len1~len2
// changeleft
/* float alc_coeffs[8] = {0, 0, 0, 1.4243e-5, -1.707e-7, 6.0113e-10, 0, 120};
 */
LinesInfo linesInfo = {
    .alc_coeffs = {0.072, -0.0018, 1.98e-6f, 1.07e-7f, 0, 0, 0, 120},
    .ego_coeffs = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 3.4, 1},
    .left_coeffs = {3.4f / 2.0f, 0, 0, 0, 0, 0, -30, 100},
    .leftleft_coeffs = {3.4f * 1.5f, 0, 0, 0, 0, 0, -30, 100},
    .right_coeffs = {-3.4f / 2.0f, 0, 0, 0, 0, 0, -30, 100},
    .rightright_coeffs = {-3.4f * 1.5f, 0, 0, 0, 0, 0, -30, 100}};
TsrInfo tsrInfo;
RadarObjInfo radarInfo;
AgsmLinesInfo agsmLinesInfo;

// temporary storage of log data
PLAYMODE playMode;
int totalFrame = DATA_NUM;
char csvFileName[150];

void ReadOutputData(const int t) {
  // spd plan result
  for (int i = 0; i <= 5; i++) {
    s_points[i].y = s_points_data[i][t];
    v_points[i].y = v_points_data[i][t];
    a_points[i].y = a_points_data[i][t];
    s_points[i].x = t_points_data[i][t];
    v_points[i].x = t_points_data[i][t];
    a_points[i].x = t_points_data[i][t];
  }
  ctrlPoint = {ctrl_point_data[0][t], ctrl_point_data[1][t]};
  spdPlanEnblSts = spdPlanEnblSts_data[t];
  g_truncated_col = truncated_col_data[t];

  gInnerSpdLmt_kph = innerSpdLmt_data[t];
  gSpecialCaseFlg = specialCaseFlg_data[t];
  gScenarioFlg = scenarioFlg_data[t];
  gGapIndex = alcGapIndex_data[t];
  gGapTarS = alcGapTarS_data[t];
  gGapTarV = alcGapTarV_data[t];

  gTempMeasureVal1 = tempMeasureVal1_data[t];
  gTempMeasureVal2 = tempMeasureVal2_data[t];
  for (int k = 0; k < 6; k++) {
    gAlcStCoeff[k] = alcStCoeff_data[k][t];
  }
  // use alc c7 as line end
  linesInfo.alc_coeffs[7] = fmaxf(s_points[4].y, s_points[5].y);
}

void WriteOutputData(const int t) {
  for (int k = 0; k <= 5; k++) {
    s_points_data[k][t] = s_points[k].y;
    v_points_data[k][t] = v_points[k].y;
    a_points_data[k][t] = a_points[k].y;
    t_points_data[k][t] = s_points[k].x;
  }
  ctrl_point_data[0][t] = ctrlPoint.x;
  ctrl_point_data[1][t] = ctrlPoint.y;
  spdPlanEnblSts_data[t] = spdPlanEnblSts;
  truncated_col_data[t] = g_truncated_col;

  innerSpdLmt_data[t] = gInnerSpdLmt_kph;
  specialCaseFlg_data[t] = gSpecialCaseFlg;
  scenarioFlg_data[t] = gScenarioFlg;
  alcGapIndex_data[t] = gGapIndex;
  alcGapTarS_data[t] = gGapTarS;
  alcGapTarV_data[t] = gGapTarV;

  tempMeasureVal1_data[t] = gTempMeasureVal1;
  tempMeasureVal2_data[t] = gTempMeasureVal2;
  for (int k = 0; k < 6; k++) {
    alcStCoeff_data[k][t] = gAlcStCoeff[k];
  }
}

void ReadInputData(const int t) {
  motionInfo.egoSpd = egoSpd_data[t];
  motionInfo.egoAcc = egoAcc_data[t];
  motionInfo.spdLmt = spdLmt_data[t];
  motionInfo.accMode = accMode_data[t];
  motionInfo.alcBehav.AutoLaneChgSide = alcBehav_data[0][t];
  motionInfo.alcBehav.AutoLaneChgSts = alcBehav_data[1][t];
  motionInfo.alcBehav.LeftBoundaryType = alcBehav_data[2][t];
  motionInfo.alcBehav.RightBoundaryType = alcBehav_data[3][t];
  motionInfo.alcBehav.NaviPilot1stRampOnDis = alcBehav_data[4][t];

  // obstacles
  g_ssmObjType.obj_num = 10;
  for (int i = 0; i < g_ssmObjType.obj_num; i++) {
    g_ssmObjType.obj_lists[i].valid_flag = objs_valid_flag_data[i][t];
    g_ssmObjType.obj_lists[i].type = objs_type_data[i][t];
    g_ssmObjType.obj_lists[i].pos_x = objs_pos_x_data[i][t];
    g_ssmObjType.obj_lists[i].pos_y = objs_pos_y_data[i][t];
    g_ssmObjType.obj_lists[i].lane_index = objs_lane_index_data[i][t];
    g_ssmObjType.obj_lists[i].speed_x = objs_speed_x_data[i][t];
    g_ssmObjType.obj_lists[i].speed_y = objs_speed_y_data[i][t];
    g_ssmObjType.obj_lists[i].acc_x = objs_acc_x_data[i][t];
    g_ssmObjType.obj_lists[i].pos_yaw = objs_pos_yaw_data[i][t];
  }

  // ego path, cubic polynominal
  linesInfo.ego_coeffs.C0[0] = ego_path_data[0][t];
  linesInfo.ego_coeffs.C1[0] = ego_path_data[1][t];
  linesInfo.ego_coeffs.C2[0] = ego_path_data[2][t];
  linesInfo.ego_coeffs.C3[0] = ego_path_data[3][t];
  linesInfo.ego_coeffs.C3[1] = ego_path_data[4][t];
  linesInfo.ego_coeffs.C3[2] = ego_path_data[5][t];
  linesInfo.ego_coeffs.Len[0] = ego_path_data[6][t];
  linesInfo.ego_coeffs.Len[1] = ego_path_data[7][t];
  linesInfo.ego_coeffs.Len[2] = ego_path_data[8][t];
  for (int i = 0; i <= 1; i++) {
    linesInfo.ego_coeffs.C0[i + 1] =
        linesInfo.ego_coeffs.C0[i] +
        linesInfo.ego_coeffs.C1[i] * linesInfo.ego_coeffs.Len[i] +
        linesInfo.ego_coeffs.C2[i] * linesInfo.ego_coeffs.Len[i] *
            linesInfo.ego_coeffs.Len[i] +
        linesInfo.ego_coeffs.C3[i] * linesInfo.ego_coeffs.Len[i] *
            linesInfo.ego_coeffs.Len[i] * linesInfo.ego_coeffs.Len[i];
    linesInfo.ego_coeffs.C1[i + 1] =
        linesInfo.ego_coeffs.C1[i] +
        2 * linesInfo.ego_coeffs.C2[i] * linesInfo.ego_coeffs.Len[i] +
        3 * linesInfo.ego_coeffs.C3[i] * linesInfo.ego_coeffs.Len[i] *
            linesInfo.ego_coeffs.Len[i];
    linesInfo.ego_coeffs.C2[i + 1] =
        (2 * linesInfo.ego_coeffs.C2[i] +
         6 * linesInfo.ego_coeffs.C3[i] * linesInfo.ego_coeffs.Len[i]) /
        2.0f;
  }

  // alc_path and Mobileye lines, quintic polynominal
  for (int i = 0; i < 8; i++) {
    linesInfo.alc_coeffs[i] = alc_path_data[i][t];
    linesInfo.left_coeffs[i] = l_path_data[i][t];
    linesInfo.right_coeffs[i] = r_path_data[i][t];
    linesInfo.leftleft_coeffs[i] = ll_path_data[i][t];
    linesInfo.rightright_coeffs[i] = rr_path_data[i][t];
  }

  // TSR info
  tsrInfo.tsr_spd = tsr_spd_data[t];
  tsrInfo.tsr_spd_warn = tsr_spd_warn_data[t];
  tsrInfo.tsr_tsi[0] = tsr_tsi_data[0][t];
  tsrInfo.tsr_tsi[1] = tsr_tsi_data[1][t];
  for (int i = 0; i < 3; i++) {
    tsrInfo.tsr_signs[i].valid = tsr_valid_flag_data[i][t];
    tsrInfo.tsr_signs[i].type = tsr_type_data[i][t];
    tsrInfo.tsr_signs[i].pos_x = tsr_pos_x_data[i][t];
    tsrInfo.tsr_signs[i].pos_y = tsr_pos_y_data[i][t];
  }
  // Radar info
#ifdef RADAR_DEMO_DISP
  for (int j = 0; j < 32; j++) {
    radarInfo.iObjectId[j] = iObjectId_data[j][t];
    radarInfo.fDistX[j] = fDistX_data[j][t];
    radarInfo.fDistY[j] = fDistY_data[j][t];
  }
#endif
  return;
}

void Time2Str(const float time, char* str) {
  char szTime_s[8];
  int tt_m = (int)time / 60;
  float tt_s = fmodf(time, 60);
  if (tt_m) {
    sprintf(str, "%d", tt_m);
    strcat(str, "m ");
    sprintf(szTime_s, "%.2f", tt_s);
    strcat(szTime_s, "s");
    strcat(str, szTime_s);
  } else {
    sprintf(str, "%.2f", tt_s);
    strcat(str, "s");
  }
}

void ShowOutputKeyInfo(const int posY) {
  char szPlanSts[10];
  char szTmpMeas1[20] = "Meas1: ", szTmpMeas2[20] = "Meas2: ";
  itoa(g_truncated_col, szPlanSts, 10);
  const char* str2 = spdPlanEnblSts ? " Enable" : " Fail  ";
  strcat(szPlanSts, str2);
  snprintf(szTmpMeas1 + strlen(szTmpMeas1),
           sizeof(szTmpMeas1) - strlen(szTmpMeas1), "%.3f", gTempMeasureVal1);
  snprintf(szTmpMeas2 + strlen(szTmpMeas2),
           sizeof(szTmpMeas2) - strlen(szTmpMeas2), "%.3f", gTempMeasureVal2);
  outtextxy(0, posY - textheight(szPlanSts) * 3, szPlanSts);
  outtextxy(0, posY - textheight(szPlanSts) * 2, szTmpMeas1);
  outtextxy(0, posY - textheight(szPlanSts), szTmpMeas2);
}

void ShowBasicFrameInfo(int* t, int* cycle, const int length, const int width) {
  // time
  *t = (*t == totalFrame - 1 ? 1 : ++(*t));
  *cycle = (time_data[*t] - time_data[*t - 1]) * 1000;
  char szCycle[12];
  Time2Str(time_data[*t], szCycle);
  char szT[] = " time = ";
  strcat(szT, szCycle);
  outtextxy(0, 0, szT);
  // introduction
  const char** szIntro = (const char**)malloc(sizeof(const char*) * 2);
  szIntro[0] = "Left click:   Play / Pause ";
  szIntro[1] = "Right click: Exit ";
  for (int i = 0; i < 2; i++) {
    outtextxy(length - textwidth(szIntro[0]), i * textheight(szIntro[i]),
              szIntro[i]);
  }
  free(szIntro);
  outtextxy(0, width - 5 - textheight(csvFileName), csvFileName);
  // progress bar
  setfillcolor(GREEN);
  solidrectangle(0, width - 5, (float)(*t) / (float)totalFrame * length, width);
  char szTotalTime[13];
  Time2Str(time_data[totalFrame - 1], szTotalTime);

  outtextxy(length - textwidth(szTotalTime),
            width - 5 - textheight(szTotalTime), szTotalTime);
}

void ShowSpdPlanInterface(const int length,
                          const int width,
                          const int offset,
                          const LinesInfo* linesInfo,
                          const MotionInfo* motionInfo) {
  GraphConfig BEV_cfg = {length / 2, width,  offset,     length / 2,
                         0,          130.0f, 3.4f * 5.0f};
  GraphConfig ST_cfg = {length / 2, (int)(width * 0.44), offset, 0, 0, 5.0f,
                        120.0f};
  GraphConfig VT_cfg = {length / 2, (int)(width * 0.44), offset,
                        0,          (int)(width * 0.28), 5.0f,
                        30.0f};
  GraphConfig AT_cfg = {length / 2, (int)(width * 0.44), offset,
                        0,          (int)(width * 0.56), 5.0f,
                        6.0f};
  char ST_title[10] = "S-T Graph";
  PlotInfo ST_info = {ST_title, s_points, &ctrlPoint, 0,
                      6,        BLUE,     true,       gAlcStCoeff};
  char VT_title[10] = "V-T";
  PlotInfo VT_info = {VT_title, v_points, &ctrlPoint, 0,
                      6,        BLUE,     true,       gAlcStCoeff};
  char AT_title[10] = "A-T";
  PlotInfo AT_info = {AT_title, a_points, &ctrlPoint, 0,
                      6,        RED,      true,       gAlcStCoeff};
  showBEVGraph(&BEV_cfg, 30.0f, &g_ssmObjType, linesInfo, &tsrInfo, motionInfo);
  showXYGraph(&ST_cfg, 0.0f, &ST_info);
  showXYGraph(&VT_cfg, 0.0f, &VT_info);
  showXYGraph(&AT_cfg, 4.0f, &AT_info);
  return;
}

void DisplayLog(const int length, const int width, const int offset) {
  if (totalFrame <= 0)
    return;
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();

  ExMessage msg;
  bool playSwitch = true;
  bool refleshScreen = false;
  int t = 0;
  int cycle = 100;  // unit: ms

  BeginBatchDraw();
  while (1) {
    // mouse clicks to change play status
    if (peekmessage(&msg, EX_MOUSE)) {
      if (functionButton(msg)) {
        refleshScreen = true;
      } else if (msg.message == WM_LBUTTONDOWN && (msg.y < 0.95f * width) &&
                 (msg.y > 0.25f * width)) {
        playSwitch = !playSwitch;
      } else if (msg.message == WM_LBUTTONDOWN && (msg.y > 0.95f * width)) {
        float rate = (float)msg.x / (float)length;
        t = rate * totalFrame;
        refleshScreen = true;
      } else if (msg.message == WM_RBUTTONDOWN) {
        break;
      }

    } else {
      if (playSwitch || refleshScreen) {
        cleardevice();
        refleshScreen = false;
        ShowBasicFrameInfo(&t, &cycle, length, width);

#ifdef AGSM_LOCAL_TEST  // HR
        ReadAgsmInputData(t, &motionInfo, &agsmLinesInfo, &g_ssmObjType);
#else
        ReadInputData(t);
#endif
        ReadOutputData(t);

        motionInfo.egoPredSpd = fmaxf(v_points[4].y, v_points[5].y);
        motionInfo.innerSpdLmt = gInnerSpdLmt_kph;
        motionInfo.specCaseFlg = gSpecialCaseFlg;
        motionInfo.scenarioFlg = gScenarioFlg;
        motionInfo.gapIndex = gGapIndex;
        motionInfo.gapTarS = gGapTarS;
        motionInfo.gapTarV = gGapTarV;

        if (playMode == AGSM) {  // HR
          GraphConfig BEV_cfg = {length, width, offset, 0, 0, 120.0f, 30.0f};
          showAGSMGraph(&BEV_cfg, 0, &g_ssmObjType, &agsmLinesInfo,
                        &motionInfo);
        } else if (playMode == RADAR) {
          GraphConfig BEV_cfg = {length, width,  offset,     0,
                                 0,      100.0f, 3.4f * 5.0f};
          showRadarGraph(&BEV_cfg, 0.0f, &radarInfo);
        } else if (playMode == LOOPBACK) {
          const int chartWidth = 200, charOffset = 50;
          ShowSpdPlanInterface(length, width - chartWidth + charOffset, offset,
                               &linesInfo, &motionInfo);
          DisplayLineChart(length, chartWidth, charOffset, 0,
                           width - chartWidth, t, 120);
          ShowOutputKeyInfo(width - chartWidth);
        } else {
          ShowSpdPlanInterface(length, width, offset, &linesInfo, &motionInfo);
          ShowOutputKeyInfo(width - 50);
        }

        outtextxy(length / 3, 0, "Playing");
        functionButton(msg);
        FlushBatchDraw();
        Sleep(cycle);
      } else {
        functionButton(msg);
        setbkmode(OPAQUE);
        outtextxy(length / 3, 0, "Paused ");
        setbkmode(TRANSPARENT);
        FlushBatchDraw();
        Sleep(500);
      }
    }
  }
  EndBatchDraw();
  closegraph();
  return;
}

#ifdef SPEED_PLANNING_H_
void ExecuteSpdPlan(const AlcPathVcc* alcPathVcc,
                    const AgsmEnvModelPath* agsmEnvModelPath,
                    const SsmObjType* ssmObjs) {
  DpSpeedPoints output;
  SpeedPlanProcessor(motionInfo.egoSpd, motionInfo.egoAcc, motionInfo.spdLmt,
                     &motionInfo.alcBehav, alcPathVcc, agsmEnvModelPath,
                     &ssmObjs->obj_lists[0], &ssmObjs->obj_lists[1],
                     &ssmObjs->obj_lists[2], &ssmObjs->obj_lists[3],
                     &ssmObjs->obj_lists[4], &ssmObjs->obj_lists[5],
                     &ssmObjs->obj_lists[6], &ssmObjs->obj_lists[7],
                     &ssmObjs->obj_lists[8], &ssmObjs->obj_lists[9], &output);

  s_points[0] = {output.Point0.t, output.Point0.s};
  s_points[1] = {output.Point1.t, output.Point1.s};
  s_points[2] = {output.Point2.t, output.Point2.s};
  s_points[3] = {output.Point3.t, output.Point3.s};
  s_points[4] = {output.Point4.t, output.Point4.s};
  s_points[5] = {output.Point5.t, output.Point5.s};
  v_points[0] = {output.Point0.t, output.Point0.v};
  v_points[1] = {output.Point1.t, output.Point1.v};
  v_points[2] = {output.Point2.t, output.Point2.v};
  v_points[3] = {output.Point3.t, output.Point3.v};
  v_points[4] = {output.Point4.t, output.Point4.v};
  v_points[5] = {output.Point5.t, output.Point5.v};
  a_points[0] = {output.Point0.t, output.Point0.a};
  a_points[1] = {output.Point1.t, output.Point1.a};
  a_points[2] = {output.Point2.t, output.Point2.a};
  a_points[3] = {output.Point3.t, output.Point3.a};
  a_points[4] = {output.Point4.t, output.Point4.a};
  a_points[5] = {output.Point5.t, output.Point5.a};

  ctrlPoint = {output.PointCtrl0.t, output.PointCtrl0.a};
  spdPlanEnblSts = output.SpdPlanEnblSts;
  return;
}

void PrintOutputInfo(const DpSpeedPoints* output) {
  /* printf("Direct: %d, Default result: \n", s_laneChangeDirection); */
  printf("AlcLatEnbl: %d, \t SpdPlanEnblSts: %d\n", output->AlcLatCtrlEnbl,
         output->SpdPlanEnblSts);
  printf("Ctrl 0: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
         output->PointCtrl0.t, output->PointCtrl0.s, output->PointCtrl0.v,
         output->PointCtrl0.a);
  printf("Ctrl 1: t = %.3f, s = %.2f, v = %.3f, a = %.3f \n",
         output->PointCtrl1.t, output->PointCtrl1.s, output->PointCtrl1.v,
         output->PointCtrl1.a);
  printf("Ctrl 2: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
         output->PointCtrl2.t, output->PointCtrl2.s, output->PointCtrl2.v,
         output->PointCtrl2.a);
  printf("Ctrl 3: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
         output->PointCtrl3.t, output->PointCtrl3.s, output->PointCtrl3.v,
         output->PointCtrl3.a);
  /* float time = 5;
 float dst_v =
     (ssmObjs.obj_lists[2].speed_x + ssmObjs.obj_lists[3].speed_x) / 2.0f;
 float dst_a = 0;
 float dst_s =
     (ssmObjs.obj_lists[2].pos_x + ssmObjs.obj_lists[3].pos_x) / 2.0f +
     dst_v * time;
    quinticPolyFit(time, s_points[0].y, v_points[0].y, a_points[0].y,
    dst_s, dst_v, dst_a, gAlcStCoeff); */
}

void CalcOneStep() {
  SsmObjType ssmObjs;
  AlcPathVcc alcPathVcc;
  AgsmEnvModelPath agsmEnvModelPath;
  LoadDummyPathData(linesInfo.alc_coeffs, &linesInfo.ego_coeffs,
                    linesInfo.left_coeffs, linesInfo.leftleft_coeffs,
                    linesInfo.right_coeffs, linesInfo.rightright_coeffs,
                    &alcPathVcc, &agsmEnvModelPath);
  memset(&ssmObjs, 0, sizeof(ssmObjs));
  if (playMode == ONESTEP) {
    LoadDummyMotionData(&motionInfo.egoSpd, &motionInfo.egoAcc,
                        &motionInfo.spdLmt, &motionInfo.accMode,
                        &motionInfo.alcBehav);
    LoadDummySSmData(&ssmObjs);
    show_predict_swt = true;
  } else {
    ssmObjs = g_ssmObjType;
  }
  ExecuteSpdPlan(&alcPathVcc, &agsmEnvModelPath, &ssmObjs);

  linesInfo.alc_coeffs[7] = fmaxf(s_points[4].y, s_points[5].y);

  /*   if (playMode == ONESTEP) {
      PrintOutputInfo(&output);
    } */
  return;
}

void DisplayOneStep(const int length, const int width, const int offset) {
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();

  motionInfo.egoPredSpd = fmaxf(v_points[4].y, v_points[5].y);
  motionInfo.innerSpdLmt = gInnerSpdLmt_kph;
  motionInfo.specCaseFlg = gSpecialCaseFlg;
  motionInfo.scenarioFlg = gScenarioFlg;
  motionInfo.gapIndex = gGapIndex;
  motionInfo.gapTarS = gGapTarS;
  motionInfo.gapTarV = gGapTarV;
  ShowSpdPlanInterface(length, width, offset, &linesInfo, &motionInfo);

  system("pause");
  closegraph();
}

void LoopbackCalculation() {
  for (int t = 0; t < totalFrame; t++) {
    ReadInputData(t);
    CalcOneStep();
    WriteOutputData(t);

    linesInfo.alc_coeffs[7] = fmaxf(s_points[4].y, s_points[5].y);
  }
}

void GenerateLocalData() {
  AlcPathVcc alcPathVcc;
  AgsmEnvModelPath agsmEnvModelPath;
  LoadDummyPathData(linesInfo.alc_coeffs, &linesInfo.ego_coeffs,
                    linesInfo.left_coeffs, linesInfo.leftleft_coeffs,
                    linesInfo.right_coeffs, linesInfo.rightright_coeffs,
                    &alcPathVcc, &agsmEnvModelPath);
  LoadDummyMotionData(&motionInfo.egoSpd, &motionInfo.egoAcc,
                      &motionInfo.spdLmt, &motionInfo.accMode,
                      &motionInfo.alcBehav);

  SsmObjType ssmObjs;
  memset(&ssmObjs, 0, sizeof(ssmObjs));
  // DummySsmData(&ssmObjs);
  LoadDummySSmData(&ssmObjs);

  float cycle_s = 0.1f;
  float obs_pos_x[10] = {0};
  float obs_pos_y[10] = {0};
  float obs_speed_x[10] = {0};
  float obs_speed_y[10] = {0};
  float accResponseDelay[5] = {0};
  int accDelay_pos = 0;

  totalFrame = 300;
  for (int t = 0; t < totalFrame; t++) {
    time_data[t] = t * cycle_s;

    motionInfo.egoAcc = accResponseDelay[accDelay_pos];
    motionInfo.egoSpd =
        fmaxf(0, motionInfo.egoSpd + motionInfo.egoAcc * cycle_s);
    spdLmt_data[t] = motionInfo.spdLmt;
    egoAcc_data[t] = motionInfo.egoAcc;
    egoSpd_data[t] = motionInfo.egoSpd;
    alcBehav_data[0][t] = motionInfo.alcBehav.AutoLaneChgSide;
    alcBehav_data[1][t] = motionInfo.alcBehav.AutoLaneChgSts;
    alcBehav_data[2][t] = motionInfo.alcBehav.LeftBoundaryType;
    alcBehav_data[3][t] = motionInfo.alcBehav.RightBoundaryType;
    alcBehav_data[4][t] = motionInfo.alcBehav.NaviPilot1stRampOnDis;
    accMode_data[t] = motionInfo.accMode;

    for (int k = 0; k < 10; k++) {
      if (t == 0) {  // initial obs pos
        obs_pos_x[k] = ssmObjs.obj_lists[k].pos_x;
        obs_pos_y[k] = ssmObjs.obj_lists[k].pos_y;
        obs_speed_x[k] = ssmObjs.obj_lists[k].speed_x;
        obs_speed_y[k] = ssmObjs.obj_lists[k].speed_y;
      }
      obs_pos_x[k] += (obs_speed_x[k] - motionInfo.egoSpd) * cycle_s;
      obs_pos_y[k] += obs_speed_y[k] * cycle_s;
      ssmObjs.obj_lists[k].pos_x = obs_pos_x[k];
      ssmObjs.obj_lists[k].pos_y = obs_pos_y[k];

      objs_valid_flag_data[k][t] = ssmObjs.obj_lists[k].valid_flag;
      objs_type_data[k][t] = ssmObjs.obj_lists[k].type;
      objs_pos_x_data[k][t] = obs_pos_x[k];
      objs_pos_y_data[k][t] = obs_pos_y[k];
      objs_lane_index_data[k][t] = ssmObjs.obj_lists[k].lane_index;
      objs_speed_x_data[k][t] = obs_speed_x[k];
      objs_speed_y_data[k][t] = obs_speed_y[k];
      objs_acc_x_data[k][t] = ssmObjs.obj_lists[k].acc_x;
      objs_pos_yaw_data[k][t] = ssmObjs.obj_lists[k].pos_yaw;
    }
    // simulate auto lane change
    /*       if (objs_pos_x_data[2][t] > 20 && alcBehav_data[0][t] == 1 &&
              alcBehav_data[1][t] == 2) {
            alcBehav_data[1][t] = 3;
            float source_coeffs[] = {0,         0,          0, 1.4243e-5,
                                     -1.707e-7, 6.0113e-10, 0, 120};
            memcpy(alc_coeffs, source_coeffs, 8 * sizeof(float));
            for (int i = 0; i < 6; i++)
              alc_path_data[i][t] = alc_coeffs[i];
          } */
    for (int k = 0; k < 8; k++) {
      alc_path_data[k][t] = linesInfo.alc_coeffs[k];
      l_path_data[k][t] = linesInfo.left_coeffs[k];
      r_path_data[k][t] = linesInfo.right_coeffs[k];
      ll_path_data[k][t] = linesInfo.leftleft_coeffs[k];
      rr_path_data[k][t] = linesInfo.rightright_coeffs[k];
    }
    ego_path_data[0][t] = linesInfo.ego_coeffs.C0[0];
    ego_path_data[1][t] = linesInfo.ego_coeffs.C1[0];
    ego_path_data[2][t] = linesInfo.ego_coeffs.C2[0];
    ego_path_data[3][t] = linesInfo.ego_coeffs.C3[0];
    ego_path_data[4][t] = linesInfo.ego_coeffs.C3[1];
    ego_path_data[5][t] = linesInfo.ego_coeffs.C3[2];
    ego_path_data[6][t] = linesInfo.ego_coeffs.Len[0];
    ego_path_data[7][t] = linesInfo.ego_coeffs.Len[1];
    ego_path_data[8][t] = linesInfo.ego_coeffs.Len[2];

    ExecuteSpdPlan(&alcPathVcc, &agsmEnvModelPath, &ssmObjs);

    // assume inertia delay 0.5s
    accResponseDelay[accDelay_pos] = ctrlPoint.y;
    accDelay_pos = accDelay_pos >= 4 ? 0 : accDelay_pos + 1;

    WriteOutputData(t);
  }
}

void DisplayLoopbackCurve(const int length, const int width, const int offset) {
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();

  memcpy(original_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));
  LoopbackCalculation();
  memcpy(loopback_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));

  DisplayLineChart(length, width, offset, 0, 0, totalFrame / 2, totalFrame);

  system("pause");
  closegraph();
}
#endif

void DisplayLineChart(const int length,
                      const int width,
                      const int offset,
                      const int oriX,
                      const int oriY,
                      const int curFrame,
                      const int frameNums) {
  const int startFrame = fmax(0, curFrame - frameNums / 2);
  const int endFrame = fmin(totalFrame - 1, curFrame + frameNums / 2);

  Point original_arr[frameNums];
  Point loopback_arr[frameNums];
  for (int i = 0; i < frameNums; i++) {
    original_arr[i].x = time_data[i + startFrame] - time_data[startFrame];
    original_arr[i].y = original_data[i + startFrame];
    loopback_arr[i].x = original_arr[i].x;
    loopback_arr[i].y = loopback_data[i + startFrame];
  }
  GraphConfig XY_cfg = {
      length, width, offset,
      oriX,   oriY,  original_arr[frameNums - 1].x - original_arr[0].x,
      6.0f};
  char title1[25] = "Origin(BLUE), New(RED)";
  PlotInfo plot_info1 = {title1,    original_arr, &ctrlPoint, 0,
                         frameNums, BLUE,         false,      gAlcStCoeff};
  char title2[1] = "";
  PlotInfo plot_info2 = {title2,    loopback_arr, &ctrlPoint, 0,
                         frameNums, RED,          false,      gAlcStCoeff};
  showXYGraph(&XY_cfg, 4.0f, &plot_info1);
  showXYGraph(&XY_cfg, 4.0f, &plot_info2);
  return;
}

void ReleaseWrapper() {
  OPENFILENAME ofn;
  char szFile[260];
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.lpstrFile = szFile;
  ofn.lpstrFile[0] = '\0';
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = "CSV Files\0*.csv\0All Files\0*.*\0";
  ofn.nFilterIndex = 1;
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
  if (GetOpenFileNameA(&ofn) == FALSE) {
    return;
  }
  //   printf("Selected file: %s\n", ofn.lpstrFile);
  strcpy(csvFileName, ofn.lpstrFile);
  LoadLog(csvFileName, &totalFrame);  // HR

#ifdef SPEED_PLANNING_H_
  if (playMode == LOOPBACK) {
    memcpy(original_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));
    LoopbackCalculation();
    memcpy(loopback_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));
  } else if (playMode == LINECHART) {
    DisplayLoopbackCurve(1000, 400, 50);
    return;
  }
#endif
  int length = 750;
  int width = playMode == LOOPBACK ? 900 : 750;
#ifdef RADAR_DEMO_DISP
  playMode = fDistX_data[0][0] ? RADAR : playMode;
  length = 400;
#endif
#ifdef AGSM_LOCAL_TEST
  playMode = AGSM;
  length = 400;
#endif
  DisplayLog(length, width, 100);

  return;
}

int main() {
#ifdef SPEED_PLANNING_H_
  // for speed planner, 3 functions: replay, loopback and simulation
  playMode = PLAYMODE(2);
  switch (playMode) {
    case ONESTEP:
      CalcOneStep();
      DisplayOneStep(750, 750, 100);
      break;
    case LOG:
    case LOOPBACK:
    case LINECHART:
      ReleaseWrapper();
      break;
    case SIMULATION:
      GenerateLocalData();
      DisplayLog(750, 750, 100);
      break;
    default:
      ReleaseWrapper();
      break;
  };
  int memoryCost = sizeof(time_data) + sizeof(egoSpd_data) * 5 +
                   sizeof(alc_path_data) + sizeof(ctrl_point_data) +
                   sizeof(truncated_col_data) * 3 + sizeof(s_points_data) * 5 +
                   sizeof(objs_valid_flag_data) + sizeof(objs_lane_index_data) +
                   sizeof(objs_type_data) + sizeof(objs_pos_x_data) * 6 +
                   sizeof(tsr_spd_data) * 3 + sizeof(tsr_spd_warn_data) +
                   sizeof(tsr_valid_flag_data) + sizeof(tsr_type_data) +
                   2 * sizeof(tsr_pos_x_data) + sizeof(ll_path_data) * 8;
#else
  // for customers, play mode depends on whether spd plan data exists
  playMode = LOG;
  ReleaseWrapper();
#endif
  return 0;
}
