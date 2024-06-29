/* Autonomous Driving Data Visualization Tool
 * All rights reserved by hgz */
#include "visualization/show_main.h"

#ifdef SPEED_PLANNING_H_
extern bool show_predict_swt;
extern SsmObjType g_ssmObjType;
extern uint8 g_truncated_col;
extern float gInnerSpdLmt_kph;
extern uint8 gSpecialCaseFlg;
extern float gTempMeasureVal;
#else
SsmObjType g_ssmObjType;
uint8 g_truncated_col;
float gInnerSpdLmt_kph;
uint8 gSpecialCaseFlg;
float gTempMeasureVal;
#endif

// Key display information
// float alc_coeffs[8] =
// {0,0,0,1.4243e-5,-1.707e-7,6.0113e-10,0,120};changeleft
float egoAcc, egoSpd, spdLmt;
float alc_coeffs[8] = {0.072, -0.0018, 1.98e-6f, 1.07e-7f,
                       0,     0,       0,        120};  // go straight

float ego_coeffs[8] = {0, 0, 0, 0, 0, 0, 0, 100};
int accMode;
AlcBehavior alcBehav;
Point s_points[6], v_points[6], a_points[6];
Point ctrlPoint;
bool AlcLgtCtrlEnbl;
float left_coeffs[8] = {3.4f / 2.0f, 0, 0, 0, 0, 0, -30, 100};
float leftleft_coeffs[8] = {3.4f * 1.5f, 0, 0, 0, 0, 0, -30, 100};
float right_coeffs[8] = {-3.4f / 2.0f, 0, 0, 0, 0, 0, -30, 100};
float rightright_coeffs[8] = {-3.4f * 1.5f, 0, 0, 0, 0, 0, -30, 100};
TsrInfo tsr_info;
float left_coeffs_me[8];
float leftleft_coeffs_me[8];
float right_coeffs_me[8];
float rightright_coeffs_me[8];
RadarObjInfo radar_info;

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
  AlcLgtCtrlEnbl = AlcLgtCtrlEnbl_data[t];
  g_truncated_col = truncated_col_data[t];
  gInnerSpdLmt_kph = innerSpdLmt_data[t];
  gSpecialCaseFlg = specialCaseFlg_data[t];
  gTempMeasureVal = tempMeasureVal_data[t];

  // use alc c7 as line end
  alc_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
}

void ReadInputData(const int t) {
  egoSpd = egoSpd_data[t];
  egoAcc = egoAcc_data[t];
  spdLmt = spdLmt_data[t];
  accMode = accMode_data[t];
  alcBehav.AutoLaneChgSide = alcBehav_data[0][t];
  alcBehav.AutoLaneChgSts = alcBehav_data[1][t];
  alcBehav.LeftBoundaryType = alcBehav_data[2][t];
  alcBehav.RightBoundaryType = alcBehav_data[3][t];
  alcBehav.NaviPilot1stRampOnDis = alcBehav_data[4][t];

  // c0->c5 orders of ego and alc_path are opposite
  for (int i = 0; i <= 5; i++)
    alc_coeffs[i] = alc_path_data[i][t];

  for (int i = 0; i <= 3; i++)
    ego_coeffs[i] = ego_path_data[i][t];

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

  // c0->c5 orders of ego and alc_path are opposite
  for (int i = 0; i < 8; i++) {
    left_coeffs[i] = l_path_data[i][t];
    right_coeffs[i] = r_path_data[i][t];
    leftleft_coeffs[i] = ll_path_data[i][t];
    rightright_coeffs[i] = rr_path_data[i][t];

    // show me original lines
    left_coeffs_me[i] = l_path_me_data[i][t];
    right_coeffs_me[i] = r_path_me_data[i][t];
    leftleft_coeffs_me[i] = ll_path_me_data[i][t];
    rightright_coeffs_me[i] = rr_path_me_data[i][t];
  }

  // TSR info
  tsr_info.tsr_spd = tsr_spd_data[t];
  tsr_info.tsr_spd_warn = tsr_spd_warn_data[t];
  tsr_info.tsr_tsi[0] = tsr_tsi_data[0][t];
  tsr_info.tsr_tsi[1] = tsr_tsi_data[1][t];
  for (int i = 0; i < 3; i++) {
    tsr_info.tsr_signs[i].valid = tsr_valid_flag_data[i][t];
    tsr_info.tsr_signs[i].type = tsr_type_data[i][t];
    tsr_info.tsr_signs[i].pos_x = tsr_pos_x_data[i][t];
    tsr_info.tsr_signs[i].pos_y = tsr_pos_y_data[i][t];
  }
  // for inner tsr test
  /*   tsr_info.tsr_signs[0].valid = true;
    tsr_info.tsr_signs[0].type = 2;
    tsr_info.tsr_signs[0].pos_x = 50;
    tsr_info.tsr_signs[0].pos_y = 5;
    tsr_info.tsr_spd_warn = true; */
  // Radar info
  for (int j = 0; j < 32; j++) {
    radar_info.iObjectId[j] = iObjectId_data[j][t];
    radar_info.fDistX[j] = fDistX_data[j][t];
    radar_info.fDistY[j] = fDistY_data[j][t];
  }
  return;
}

void Time2Str(const float time, char* str) {
  char szTime_s[8];
  int tt_m = (int)time / 60;
  float tt_s = fmod(time, 60);
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

void DisplaySpdPlanInterface(const int length,
                             const int width,
                             const int offset,
                             const LinesInfo* lines_info,
                             const SpdInfo* spd_info) {
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
  showBEVGraph(&BEV_cfg, 30.0f, &tsr_info, &g_ssmObjType, lines_info, spd_info);
  showXYGraph(&ST_cfg, 0.0f, "S-T Graph", BLUE, s_points, 0, 6, &ctrlPoint);
  showXYGraph(&VT_cfg, 0.0f, "V-T", BLUE, v_points, 0, 6, &ctrlPoint);
  showXYGraph(&AT_cfg, 4.0f, "A-T", RED, a_points, 0, 6, &ctrlPoint);
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

        // time
        t = (t == totalFrame - 1 ? 1 : ++t);
        cycle = (time_data[t] - time_data[t - 1]) * 1000;
        char szCycle[12];
        Time2Str(time_data[t], szCycle);
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
        solidrectangle(0, width - 5, (float)t / (float)totalFrame * length,
                       width);
        char szTotalTime[13];
        Time2Str(time_data[totalFrame - 1], szTotalTime);

        outtextxy(length - textwidth(szTotalTime),
                  width - 5 - textheight(szTotalTime), szTotalTime);

        ReadInputData(t);
        ReadOutputData(t);

        LinesInfo lines_info = {alc_coeffs,      ego_coeffs,
                                left_coeffs,     leftleft_coeffs,
                                right_coeffs,    rightright_coeffs,
                                left_coeffs_me,  leftleft_coeffs_me,
                                right_coeffs_me, rightright_coeffs_me};
        SpdInfo spd_info = {egoSpd,
                            fmax(v_points[4].y, v_points[5].y),
                            spdLmt,
                            gInnerSpdLmt_kph,
                            gSpecialCaseFlg,
                            accMode,
                            alcBehav.AutoLaneChgSide,
                            alcBehav.AutoLaneChgSts,
                            alcBehav.LeftBoundaryType,
                            alcBehav.RightBoundaryType};

        if (playMode == FUSION) {
          GraphConfig BEV_cfg = {length, width, offset, 0, 0, 100.0f, 30.0f};
          showBEVGraph(&BEV_cfg, 0, &tsr_info, &g_ssmObjType, &lines_info,
                       &spd_info);
        } else if (playMode == RADAR) {
          GraphConfig BEV_cfg = {length, width,  offset,     0,
                                 0,      100.0f, 3.4f * 5.0f};
          showRadarGraph(&BEV_cfg, 30.0f, &radar_info);
        } else if (playMode == LOOPBACK) {
          const int chartWidth = 200, charOffset = 50;
          DisplaySpdPlanInterface(length, width - chartWidth + charOffset,
                                  offset, &lines_info, &spd_info);
          DisplayLineChart(length, chartWidth, charOffset, 0,
                           width - chartWidth, t, 120);
          char szPlanSts[10], szTmpMeas[20] = "Meas: ";
          itoa(g_truncated_col, szPlanSts, 10);
          const char* str2 = AlcLgtCtrlEnbl ? " Enable" : " Fail  ";
          strcat(szPlanSts, str2);
          sprintf(szTmpMeas, "%.5f", gTempMeasureVal);
          outtextxy(0, width - chartWidth - textheight(szPlanSts), szPlanSts);
          outtextxy(0, width - chartWidth, szTmpMeas);
        } else {
          DisplaySpdPlanInterface(length, width, offset, &lines_info,
                                  &spd_info);
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
}

#ifdef SPEED_PLANNING_H_
void CalcOneStep() {
  AlcPathVcc alcPathVcc = {alc_coeffs[0], alc_coeffs[1], alc_coeffs[2],
                           alc_coeffs[3], alc_coeffs[4], alc_coeffs[5],
                           alc_coeffs[7]};
  AlcPathVcc egoPathVcc = {
      ego_coeffs[0], ego_coeffs[1], ego_coeffs[2], ego_coeffs[3], 0, 0,
      ego_coeffs[7]};

  SsmObjType ssmObjs;
  memset(&ssmObjs, 0, sizeof(ssmObjs));
  if (playMode == ONESTEP) {
    egoSpd = 15.0f, egoAcc = 0, spdLmt = 33.3 * 3.6f;  // kph
    accMode = 5;
    alcBehav.AutoLaneChgSide = 0;
    alcBehav.AutoLaneChgSts = 1;
    alcBehav.LeftBoundaryType = 2;
    alcBehav.RightBoundaryType = 2;
    LoadDummySSmData(&ssmObjs);
    show_predict_swt = false;

  } else {
    ssmObjs = g_ssmObjType;
  }

  DpSpeedPoints output = SpeedPlanProcessor(
      egoSpd, egoAcc, spdLmt, &alcBehav, &alcPathVcc, &egoPathVcc,
      &ssmObjs.obj_lists[0], &ssmObjs.obj_lists[1], &ssmObjs.obj_lists[2],
      &ssmObjs.obj_lists[3], &ssmObjs.obj_lists[4], &ssmObjs.obj_lists[5],
      &ssmObjs.obj_lists[6], &ssmObjs.obj_lists[7], &ssmObjs.obj_lists[8],
      &ssmObjs.obj_lists[9]);

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

  ctrlPoint = {output.pointCtrl0.t, output.pointCtrl0.a};
  AlcLgtCtrlEnbl = output.AlcLgtCtrlEnbl;
  alc_coeffs[7] = fmax(s_points[4].y, s_points[5].y);

  /* printf("Direct: %d, Default result: \n", g_laneChangeDirection); */
  if (playMode == ONESTEP) {
    printf("AlcLatEnbl: %d, \t AlcLgtEnble: %d\n", output.AlcLatCtrlEnbl,
           output.AlcLgtCtrlEnbl);
    printf("Ctrl 0: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
           output.pointCtrl0.t, output.pointCtrl0.s, output.pointCtrl0.v,
           output.pointCtrl0.a);
    printf("Ctrl 1: t = %.3f, s = %.2f, v = %.3f, a = %.3f \n",
           output.pointCtrl1.t, output.pointCtrl1.s, output.pointCtrl1.v,
           output.pointCtrl1.a);
    printf("Ctrl 2: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
           output.pointCtrl2.t, output.pointCtrl2.s, output.pointCtrl2.v,
           output.pointCtrl2.a);
    printf("Ctrl 3: t = %.3f, s = %.3f, v = %.3f, a = %.3f \n",
           output.pointCtrl3.t, output.pointCtrl3.s, output.pointCtrl3.v,
           output.pointCtrl3.a);
    float fit_coeffi[6] = {0};
    quinticPolyFit(s_points[1].x, s_points[0].y, v_points[0].y, a_points[0].y,
                   s_points[1].y, v_points[1].y, a_points[1].y, fit_coeffi);
  }
}

void DisplayOneStep(const int length, const int width, const int offset) {
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();
  show_predict_swt = true;
  LinesInfo lines_info = {
      alc_coeffs,      ego_coeffs,          left_coeffs,    leftleft_coeffs,
      right_coeffs,    rightright_coeffs,   left_coeffs_me, leftleft_coeffs_me,
      right_coeffs_me, rightright_coeffs_me};
  SpdInfo spd_info = {v_points[0].y,
                      fmax(v_points[4].y, v_points[5].y),
                      spdLmt,
                      gInnerSpdLmt_kph,
                      gSpecialCaseFlg,
                      accMode,
                      alcBehav.AutoLaneChgSide,
                      alcBehav.AutoLaneChgSts,
                      alcBehav.LeftBoundaryType,
                      alcBehav.RightBoundaryType};
  DisplaySpdPlanInterface(length, width, offset, &lines_info, &spd_info);

  system("pause");
  closegraph();
}

void LoopbackCalculation() {
  for (int t = 0; t < totalFrame; t++) {
    ReadInputData(t);
    CalcOneStep();
    for (int k = 0; k <= 5; k++) {
      s_points_data[k][t] = s_points[k].y;
      v_points_data[k][t] = v_points[k].y;
      a_points_data[k][t] = a_points[k].y;
      t_points_data[k][t] = s_points[k].x;
    }
    AlcLgtCtrlEnbl_data[t] = AlcLgtCtrlEnbl;
    truncated_col_data[t] = g_truncated_col;
    ctrl_point_data[0][t] = ctrlPoint.x;
    ctrl_point_data[1][t] = ctrlPoint.y;

    innerSpdLmt_data[t] = gInnerSpdLmt_kph;
    specialCaseFlg_data[t] = gSpecialCaseFlg;
    tempMeasureVal_data[t] = gTempMeasureVal;

    alc_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
  }
}

void GenerateLocalData() {
  egoSpd = 30.0f / 3.6f, egoAcc = 0.0f, spdLmt = 50.0f;
  accMode = 5;
  alcBehav.AutoLaneChgSide = 0;
  alcBehav.AutoLaneChgSts = 1;
  alcBehav.LeftBoundaryType = 2;
  alcBehav.RightBoundaryType = 2;

  AlcPathVcc alcPathVcc = {alc_coeffs[0], alc_coeffs[1], alc_coeffs[2],
                           alc_coeffs[3], alc_coeffs[4], alc_coeffs[5],
                           alc_coeffs[7]};
  AlcPathVcc egoPathVcc = {
      ego_coeffs[0], ego_coeffs[1], ego_coeffs[2], ego_coeffs[3], 0, 0,
      ego_coeffs[7]};

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

    egoAcc = accResponseDelay[accDelay_pos];
    egoSpd += egoAcc * cycle_s;
    spdLmt_data[t] = spdLmt;
    egoAcc_data[t] = egoAcc;
    egoSpd_data[t] = egoSpd;
    alcBehav_data[0][t] = alcBehav.AutoLaneChgSide;
    alcBehav_data[1][t] = alcBehav.AutoLaneChgSts;
    alcBehav_data[2][t] = alcBehav.LeftBoundaryType;
    alcBehav_data[3][t] = alcBehav.RightBoundaryType;
    alcBehav_data[4][t] = alcBehav.NaviPilot1stRampOnDis;

    accMode_data[t] = accMode;

    for (int k = 0; k < 10; k++) {
      if (t == 0) {  // initial obs pos
        obs_pos_x[k] = ssmObjs.obj_lists[k].pos_x;
        obs_pos_y[k] = ssmObjs.obj_lists[k].pos_y;
        obs_speed_x[k] = ssmObjs.obj_lists[k].speed_x;
        obs_speed_y[k] = ssmObjs.obj_lists[k].speed_y;
      }
      obs_pos_x[k] += (obs_speed_x[k] - egoSpd) * cycle_s;
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

      // simulate auto lane change
      /*       if (objs_pos_x_data[2][t] > 20 && alcBehav_data[0][t] == 1 &&
                alcBehav_data[1][t] == 2) {
              alcBehav_data[1][t] = 3;
              float source_coeffs[] = {0,         0,          0, 1.4243e-5,
                                       -1.707e-7, 6.0113e-10, 0, 120};
              memcpy(alc_coeffs, source_coeffs, 8 * sizeof(float));
              for (int i = 0; i < 6; i++)
                alc_path_data[i][t] = alc_coeffs[i];
              alcPathVcc.FiveCoeff = alc_coeffs[5];
              alcPathVcc.FourCoeff = alc_coeffs[4];
              alcPathVcc.ThrdCoeff = alc_coeffs[3];
              alcPathVcc.SecCoeff = alc_coeffs[2];
              alcPathVcc.FirstCoeff = alc_coeffs[1];
              alcPathVcc.ConCoeff = alc_coeffs[0];
            } */
    }

    for (int k = 0; k < 8; k++) {
      l_path_data[k][t] = left_coeffs[k];
      r_path_data[k][t] = right_coeffs[k];
      ll_path_data[k][t] = leftleft_coeffs[k];
      rr_path_data[k][t] = rightright_coeffs[k];
    }

    DpSpeedPoints output = SpeedPlanProcessor(
        egoSpd, egoAcc, spdLmt, &alcBehav, &alcPathVcc, &egoPathVcc,
        &ssmObjs.obj_lists[0], &ssmObjs.obj_lists[1], &ssmObjs.obj_lists[2],
        &ssmObjs.obj_lists[3], &ssmObjs.obj_lists[4], &ssmObjs.obj_lists[5],
        &ssmObjs.obj_lists[6], &ssmObjs.obj_lists[7], &ssmObjs.obj_lists[8],
        &ssmObjs.obj_lists[9]);
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
    ctrlPoint = {output.pointCtrl0.t, output.pointCtrl0.a};
    AlcLgtCtrlEnbl = output.AlcLgtCtrlEnbl;

    // assume inertia delay 0.5s
    accResponseDelay[accDelay_pos] = ctrlPoint.y;
    accDelay_pos = accDelay_pos >= 4 ? 0 : accDelay_pos + 1;

    for (int k = 0; k <= 5; k++) {
      s_points_data[k][t] = s_points[k].y;
      v_points_data[k][t] = v_points[k].y;
      a_points_data[k][t] = a_points[k].y;
      t_points_data[k][t] = s_points[k].x;
    }
    AlcLgtCtrlEnbl_data[t] = AlcLgtCtrlEnbl;
    truncated_col_data[t] = g_truncated_col;
    ctrl_point_data[0][t] = ctrlPoint.x;
    ctrl_point_data[1][t] = ctrlPoint.y;

    innerSpdLmt_data[t] = gInnerSpdLmt_kph;
    specialCaseFlg_data[t] = gSpecialCaseFlg;
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
  showXYGraph(&XY_cfg, 4.0f, "Origin(BLUE), New(RED)", BLUE, original_arr, 0,
              frameNums, &ctrlPoint);
  showXYGraph(&XY_cfg, 4.0f, "", RED, loopback_arr, 0, frameNums, &ctrlPoint);
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

  if (GetOpenFileNameA(&ofn) == TRUE) {
    //   printf("Selected file: %s\n", ofn.lpstrFile);
    strcpy(csvFileName, ofn.lpstrFile);
    LoadLog();
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
    playMode = t_points_data[1][0] == 0 ? FUSION : playMode;
    playMode = fDistX_data[0][0] ? RADAR : playMode;
    int length = (playMode == FUSION || playMode == RADAR) ? 400 : 750;
    int width = playMode == LOOPBACK ? 900 : 750;
    DisplayLog(length, width, 100);
  }
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