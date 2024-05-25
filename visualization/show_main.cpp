/* Autonomous Driving Data Visualization Tool
 * All rights reserved by hgz */

#include "visualization/show_main.h"

#ifdef SPEED_PLANNING_H_
extern SsmFrameType g_ssmFrameType;
extern uint8 g_truncated_col;
extern bool show_predict_swt;
extern float gInnerSpdLmt_kph;
extern uint8 gSpecialCaseFlg;
#else
SsmFrameType g_ssmFrameType;
uint8 g_truncated_col;
#endif

// Key display information
// float ego_coeffs[8] =
// {6.0113e-10,-1.707e-7,1.4243e-5,0,0,0,0,120};changeleft
float egoAcc, egoSpd, spdLmt;
float ego_coeffs[8] = {0,       0,     1.07e-7f, 1.98e-6f,
                       -0.0018, 0.072, 0,        120};  // go straight
int accMode;
AlcBehavior alcBehav;
Point s_points[6], v_points[6], a_points[6];
Point ctrlPoint;
bool AlcLgtCtrlEnbl;
float innerSpdLmt;
int specCaseFlg;
float left_coeffs[8] = {0, 0, 0, 0, 0, 3.4f / 2.0f, -30, 100};
float leftleft_coeffs[8] = {0, 0, 0, 0, 0, 3.4f * 1.5f, -30, 100};
float right_coeffs[8] = {0, 0, 0, 0, 0, -3.4f / 2.0f, -30, 100};
float rightright_coeffs[8] = {0, 0, 0, 0, 0, -3.4f * 1.5f, -30, 100};
TsrInfo tsr_info;
float left_coeffs_me[8];
float leftleft_coeffs_me[8];
float right_coeffs_me[8];
float rightright_coeffs_me[8];

// temporary storage of log data
PLAYMODE playMode;
int totalFrame = DATA_NUM;
char csvFileName[150];

float time_data[DATA_NUM];
float egoSpd_data[DATA_NUM];
float egoAcc_data[DATA_NUM];
float spdLmt_data[DATA_NUM];
float alc_path_data[6][DATA_NUM];
int alcBehav_data[4][DATA_NUM];
int accMode_data[DATA_NUM];

bool AlcLgtCtrlEnbl_data[DATA_NUM];
int truncated_col_data[DATA_NUM];
float innerSpdLmt_data[DATA_NUM];
int specCaseFlg_data[DATA_NUM];

float ctrl_point_data[2][DATA_NUM];
float s_points_data[6][DATA_NUM];
float v_points_data[6][DATA_NUM];
float a_points_data[6][DATA_NUM];
float t_points_data[6][DATA_NUM];

bool objs_valid_flag_data[10][DATA_NUM];
int objs_lane_index_data[10][DATA_NUM];
int objs_type_data[10][DATA_NUM];
float objs_pos_x_data[10][DATA_NUM];
float objs_pos_y_data[10][DATA_NUM];
float objs_speed_x_data[10][DATA_NUM];
float objs_speed_y_data[10][DATA_NUM];
float objs_acc_x_data[10][DATA_NUM];
float objs_pos_yaw_data[10][DATA_NUM];

int tsr_spd_data[DATA_NUM];
bool tsr_spd_warn_data[DATA_NUM];
int tsr_tsi_data[2][DATA_NUM];

bool tsr_valid_flag_data[3][DATA_NUM];
int tsr_type_data[3][DATA_NUM];
float tsr_pos_x_data[3][DATA_NUM];
float tsr_pos_y_data[3][DATA_NUM];

float ll_path_data[8][DATA_NUM];
float l_path_data[8][DATA_NUM];
float r_path_data[8][DATA_NUM];
float rr_path_data[8][DATA_NUM];
float ll_path_me_data[8][DATA_NUM];
float l_path_me_data[8][DATA_NUM];
float r_path_me_data[8][DATA_NUM];
float rr_path_me_data[8][DATA_NUM];

float original_data[DATA_NUM];
float loopback_data[DATA_NUM];

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
  AlcLgtCtrlEnbl = AlcLgtCtrlEnbl_data[t];
  g_truncated_col = truncated_col_data[t];
  ctrlPoint = {ctrl_point_data[0][t], ctrl_point_data[1][t]};
  innerSpdLmt = innerSpdLmt_data[t];
  specCaseFlg = specCaseFlg_data[t];

  // use alc c7 as line end
  ego_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
}

void ReadInputData(const int t) {
  egoSpd = egoSpd_data[t];
  egoAcc = egoAcc_data[t];
  spdLmt = spdLmt_data[t];
  accMode = accMode_data[t];
  alcBehav.AutoLaneChgSide = alcBehav_data[0][t];
  alcBehav.AutoLaneChgSts = alcBehav_data[1][t];
  alcBehav.LeftBoundaryType = 2;  // always dash
  alcBehav.RightBoundaryType = 2;

  // c0->c5 orders of ego and alc_path are opposite
  for (int i = 1; i <= 5; i++)
    ego_coeffs[i] = alc_path_data[5 - i][t];

  // obstacles
  g_ssmFrameType.Ssm_Objs_Frame_st.obj_num = 10;
  for (int i = 0; i < g_ssmFrameType.Ssm_Objs_Frame_st.obj_num; i++) {
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].valid_flag =
        objs_valid_flag_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].type = objs_type_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].pos_x = objs_pos_x_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].pos_y = objs_pos_y_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].lane_index =
        objs_lane_index_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].speed_x =
        objs_speed_x_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].speed_y =
        objs_speed_y_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].acc_x = objs_acc_x_data[i][t];
    g_ssmFrameType.Ssm_Objs_Frame_st.obj_lists[i].pos_yaw =
        objs_pos_yaw_data[i][t];
  }

  // c0->c5 orders of ego and alc_path are opposite
  for (int i = 0; i < 8; i++) {
    if (i <= 5) {
      left_coeffs[i] = l_path_data[5 - i][t];
      right_coeffs[i] = r_path_data[5 - i][t];
      leftleft_coeffs[i] = ll_path_data[5 - i][t];
      rightright_coeffs[i] = rr_path_data[5 - i][t];
    } else {
      left_coeffs[i] = l_path_data[i][t];
      right_coeffs[i] = r_path_data[i][t];
      leftleft_coeffs[i] = ll_path_data[i][t];
      rightright_coeffs[i] = rr_path_data[i][t];
    }
    // show me original lines
    if (i <= 5) {
      left_coeffs_me[i] = l_path_me_data[5 - i][t];
      right_coeffs_me[i] = r_path_me_data[5 - i][t];
      leftleft_coeffs_me[i] = ll_path_me_data[5 - i][t];
      rightright_coeffs_me[i] = rr_path_me_data[5 - i][t];
    } else {
      left_coeffs_me[i] = l_path_me_data[i][t];
      right_coeffs_me[i] = r_path_me_data[i][t];
      leftleft_coeffs_me[i] = ll_path_me_data[i][t];
      rightright_coeffs_me[i] = rr_path_me_data[i][t];
    }
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

        LinesInfo lines_info = {
            ego_coeffs,         left_coeffs,       leftleft_coeffs,
            right_coeffs,       rightright_coeffs, left_coeffs_me,
            leftleft_coeffs_me, right_coeffs_me,   rightright_coeffs_me};
        SpdInfo spd_info = {egoSpd,
                            fmax(v_points[4].y, v_points[5].y),
                            spdLmt,
                            innerSpdLmt,
                            specCaseFlg,
                            accMode,
                            alcBehav.AutoLaneChgSide,
                            alcBehav.AutoLaneChgSts};

        if (playMode == PLAYMODE::FUSION) {
          GraphConfig BEV_cfg = {length / 2, width,  offset,     0,
                                 0,          100.0f, 3.4f * 5.0f};
          showBEVGraph(&BEV_cfg, 0, &tsr_info, &g_ssmFrameType, &lines_info,
                       &spd_info);
        } else {
          GraphConfig BEV_cfg = {length / 2, width,  offset,     length / 2,
                                 0,          130.0f, 3.4f * 5.0f};
          GraphConfig ST_cfg = {
              length / 2, (int)(width * 0.44), offset, 0, 0, 5.0f, 120.0f};
          GraphConfig VT_cfg = {length / 2, (int)(width * 0.44), offset,
                                0,          (int)(width * 0.28), 5.0f,
                                30.0f};
          GraphConfig AT_cfg = {length / 2, (int)(width * 0.44), offset,
                                0,          (int)(width * 0.56), 5.0f,
                                6.0f};
          showBEVGraph(&BEV_cfg, 30.0f, &tsr_info, &g_ssmFrameType, &lines_info,
                       &spd_info);
          showXYGraph(&ST_cfg, 0.0f, "S-T Graph", BLUE, s_points, 0, 6,
                      &ctrlPoint);
          showXYGraph(&VT_cfg, 0.0f, "V-T", BLUE, v_points, 0, 6, &ctrlPoint);
          showXYGraph(&AT_cfg, 4.0f, "A-T", RED, a_points, 0, 6, &ctrlPoint);

          char szPlanSts[10];
          itoa(g_truncated_col, szPlanSts, 10);
          const char* str2 = AlcLgtCtrlEnbl ? " Enable" : " Fail  ";
          strcat(szPlanSts, str2);
          outtextxy(0, width * 0.75 + 50, szPlanSts);
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
  AlcPathVcc alcPathVcc;
  memset(&alcPathVcc, 0, sizeof(alcPathVcc));
  alcPathVcc.FiveCoeff = ego_coeffs[0];
  alcPathVcc.FourCoeff = ego_coeffs[1];
  alcPathVcc.ThrdCoeff = ego_coeffs[2];
  alcPathVcc.SecCoeff = ego_coeffs[3];
  alcPathVcc.FirstCoeff = ego_coeffs[4];
  alcPathVcc.ConCoeff = ego_coeffs[5];

  SsmFrameType ssmFrame;
  memset(&ssmFrame, 0, sizeof(ssmFrame));
  if (playMode == PLAYMODE::ONESTEP) {
    egoSpd = 25.0f, egoAcc = 0, spdLmt = 33.3 * 3.6f;  // kph
    accMode = 5;
    alcBehav.AutoLaneChgSide = 0;
    alcBehav.AutoLaneChgSts = 1;
    alcBehav.LeftBoundaryType = 2;
    alcBehav.RightBoundaryType = 2;
    DummySsmData(&ssmFrame);
  } else {
    ssmFrame = g_ssmFrameType;
  }

  DpSpeedPoints output =
      SpeedPlanProcessor(egoSpd, egoAcc, spdLmt, &alcBehav, &alcPathVcc,
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[0],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[1],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[2],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[3],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[4],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[5],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[6],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[7],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[8],
                         &ssmFrame.Ssm_Objs_Frame_st.obj_lists[9]);

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
  ego_coeffs[7] = fmax(s_points[4].y, s_points[5].y);

  /*   printf("Memory: %ld Byte \n", g_ALC_MEMORY_SIZE);
    printf("Direct: %d, Default result: \n", g_laneChangeDirection); */
  if (playMode == PLAYMODE::ONESTEP) {
    printf("AlcLatEnbl: %d, \t AlcLgtEnble: %d\n", output.AlcLatCtrlEnbl,
           output.AlcLgtCtrlEnbl);
    for (int i = 0; i <= 5; i++) {
      printf("Point%d: t = %.1f, s = %.1f, v = %.1f, a = %.1f \n", i,
             s_points[i].x, s_points[i].y, v_points[i].y, a_points[i].y);
    }
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

  LinesInfo lines_info = {
      ego_coeffs,         left_coeffs,       leftleft_coeffs,
      right_coeffs,       rightright_coeffs, left_coeffs_me,
      leftleft_coeffs_me, right_coeffs_me,   rightright_coeffs_me};
  SpdInfo spd_info = {v_points[0].y,
                      fmax(v_points[4].y, v_points[5].y),
                      spdLmt,
                      innerSpdLmt,
                      specCaseFlg,
                      accMode,
                      alcBehav.AutoLaneChgSide,
                      alcBehav.AutoLaneChgSts};
  show_predict_swt = true;
  GraphConfig BEV_cfg = {length / 2, width,  offset,     length / 2,
                         0,          130.0f, 3.4f * 5.0f};
  showBEVGraph(&BEV_cfg, 30.0f, &tsr_info, &g_ssmFrameType, &lines_info,
               &spd_info);
  GraphConfig ST_cfg = {length / 2, (int)(width * 0.44), offset, 0, 0, 5.0f,
                        120.0f};
  GraphConfig VT_cfg = {length / 2, (int)(width * 0.44), offset,
                        0,          (int)(width * 0.28), 5.0f,
                        30.0f};
  GraphConfig AT_cfg = {length / 2, (int)(width * 0.44), offset,
                        0,          (int)(width * 0.56), 5.0f,
                        6.0f};
  showXYGraph(&ST_cfg, 0.0f, "S-T Graph", BLUE, s_points, 0, 6, &ctrlPoint);
  showXYGraph(&VT_cfg, 0.0f, "V-T", BLUE, v_points, 0, 6, &ctrlPoint);
  showXYGraph(&AT_cfg, 4.0f, "A-T", RED, a_points, 0, 6, &ctrlPoint);

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
    specCaseFlg_data[t] = gSpecialCaseFlg;

    ego_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
  }
}

void LocalDummySsmData(SsmFrameType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->Ssm_Objs_Frame_st.obj_num = 3;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].pos_x = 80;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].pos_y = 0;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].acc_x = 0.0f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].speed_x = 50.0f / 3.6f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].type = 1;        // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].lane_index = 3;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[0].valid_flag = TRUE;

  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].pos_x = 10;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].pos_y = 3.15f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].acc_x = 0.0f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].speed_x = 40.0f / 3.6f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].speed_y = 0.0f;  // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].type = 1;        // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].lane_index = 2;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[2].valid_flag = FALSE;

  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].pos_x = 1;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].pos_y = 3.15f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].acc_x = 0.0f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].speed_x = 50.0f / 3.6f;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].speed_y = 0.0f;  // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].type = 1;        // hgz
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].lane_index = 2;
  ssmObjs->Ssm_Objs_Frame_st.obj_lists[4].valid_flag = TRUE;
}

void GenerateLocalData() {
  egoSpd = 40.0f / 3.6f, egoAcc = 0.0f, spdLmt = 40.0f;

  alcBehav.AutoLaneChgSide = 1;
  alcBehav.AutoLaneChgSts = 2;
  alcBehav.LeftBoundaryType = 2;
  alcBehav.RightBoundaryType = 2;

  accMode = 5;
  AlcPathVcc alcPathVcc;
  memset(&alcPathVcc, 0, sizeof(alcPathVcc));
  alcPathVcc.FiveCoeff = ego_coeffs[0];
  alcPathVcc.FourCoeff = ego_coeffs[1];
  alcPathVcc.ThrdCoeff = ego_coeffs[2];
  alcPathVcc.SecCoeff = ego_coeffs[3];
  alcPathVcc.FirstCoeff = ego_coeffs[4];
  alcPathVcc.ConCoeff = ego_coeffs[5];

  SsmFrameType ssmFrame;
  memset(&ssmFrame, 0, sizeof(ssmFrame));
  // DummySsmData(&ssmFrame);
  LocalDummySsmData(&ssmFrame);

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
    alcBehav_data[3][t] = alcBehav.LeftBoundaryType;
    alcBehav_data[4][t] = alcBehav.RightBoundaryType;
    accMode_data[t] = accMode;

    for (int k = 0; k < 10; k++) {
      if (t == 0) {  // initial obs pos
        obs_pos_x[k] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_x;
        obs_pos_y[k] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_y;
        obs_speed_x[k] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].speed_x;
        obs_speed_y[k] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].speed_y;
      }
      obs_pos_x[k] += (obs_speed_x[k] - egoSpd) * cycle_s;
      obs_pos_y[k] += obs_speed_y[k] * cycle_s;
      ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_x = obs_pos_x[k];
      ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_y = obs_pos_y[k];

      objs_valid_flag_data[k][t] =
          ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].valid_flag;
      objs_type_data[k][t] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].type;
      objs_pos_x_data[k][t] = obs_pos_x[k];
      objs_pos_y_data[k][t] = obs_pos_y[k];
      objs_lane_index_data[k][t] =
          ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].lane_index;
      objs_speed_x_data[k][t] = obs_speed_x[k];
      objs_speed_y_data[k][t] = obs_speed_y[k];
      objs_acc_x_data[k][t] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].acc_x;
      objs_pos_yaw_data[k][t] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_yaw;

      // simulate auto lane change
      /*       if (objs_pos_x_data[2][t] > 20 && alcBehav_data[0][t] == 1 &&
                alcBehav_data[1][t] == 2) {
              alcBehav_data[1][t] = 3;
              float source_coeffs[] = {6.0113e-10, -1.707e-7, 1.4243e-5, 0,
                                       0,          0,         0,         120};
              memcpy(ego_coeffs, source_coeffs, 8 * sizeof(float));
              for (int i = 0; i < 6; i++)
                alc_path_data[i][t] = ego_coeffs[i];
              alcPathVcc.FiveCoeff = ego_coeffs[0];
              alcPathVcc.FourCoeff = ego_coeffs[1];
              alcPathVcc.ThrdCoeff = ego_coeffs[2];
              alcPathVcc.SecCoeff = ego_coeffs[3];
              alcPathVcc.FirstCoeff = ego_coeffs[4];
              alcPathVcc.ConCoeff = ego_coeffs[5];
            } */
    }

    for (int k = 0; k < 8; k++) {
      if (k <= 5) {
        l_path_data[k][t] = left_coeffs[5 - k];
        r_path_data[k][t] = right_coeffs[5 - k];
        ll_path_data[k][t] = leftleft_coeffs[5 - k];
        rr_path_data[k][t] = rightright_coeffs[5 - k];
      } else {
        l_path_data[k][t] = left_coeffs[k];
        r_path_data[k][t] = right_coeffs[k];
        ll_path_data[k][t] = leftleft_coeffs[k];
        rr_path_data[k][t] = rightright_coeffs[k];
      }
    }

    DpSpeedPoints output =
        SpeedPlanProcessor(egoSpd, egoAcc, spdLmt, &alcBehav, &alcPathVcc,
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[0],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[1],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[2],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[3],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[4],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[5],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[6],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[7],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[8],
                           &ssmFrame.Ssm_Objs_Frame_st.obj_lists[9]);
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
    specCaseFlg_data[t] = gSpecialCaseFlg;
  }
}

void DisplayLineChart(const int length, const int width, const int offset) {
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();

  memcpy(original_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));
  LoopbackCalculation();
  memcpy(loopback_data, ctrl_point_data[1], sizeof(ctrl_point_data[1]));

  const int startFrame = 500;
  const int frameNums = 1000;
  const int endFrame = fmin(startFrame + frameNums - 1, DATA_NUM);
  Point original_arr[frameNums];
  Point loopback_arr[frameNums];
  for (int i = 0; i < frameNums; i++) {
    original_arr[i].x = time_data[i + startFrame] - time_data[startFrame];
    original_arr[i].y = original_data[i + startFrame];
    loopback_arr[i].x = time_data[i + startFrame] - time_data[startFrame];
    loopback_arr[i].y = loopback_data[i + startFrame];
  }

  GraphConfig XY_cfg = {
      length, width, offset,
      0,      0,     original_arr[frameNums - 1].x - original_arr[0].x,
      6.0f};
  showXYGraph(&XY_cfg, 4.0f, "Origin(BLUE), New(RED)", BLUE, original_arr, 0,
              frameNums, &ctrlPoint);
  showXYGraph(&XY_cfg, 4.0f, "Origin(BLUE), New(RED)", RED, loopback_arr, 0,
              frameNums, &ctrlPoint);

  system("pause");
  closegraph();
}
#endif

BOOL GetFileFromUser(char* filePath, int size) {
  OPENFILENAME ofn;  // common dialog box structure
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = NULL;  // no owner window
  ofn.lpstrFilter = "CSV Files (*.csv)\0*.csv\0All Files (*.*)\0*.*\0";
  ofn.lpstrCustomFilter = NULL;
  ofn.nFilterIndex = 1;
  ofn.lpstrFile = filePath;
  ofn.nMaxFile = size;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = NULL;  // default directory
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

  // Display the Open dialog box.
  return GetOpenFileName(&ofn) == TRUE;
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
    if (playMode == PLAYMODE::LOOPBACK) {
      LoopbackCalculation();
    } else if (playMode == PLAYMODE::LINECHART) {
      DisplayLineChart(1000, 400, 50);
      return;
    }
#endif
    playMode = ctrl_point_data[0][0] < 1e-6f ? PLAYMODE::FUSION : playMode;
    int length = playMode == PLAYMODE::FUSION ? 400 : 750;
    DisplayLog(length, 750, 100);
  } else {
    //  printf("No file selected\n");
    //  system("pause");
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
// printf("Memory: %d kB\n", memoryCost / 1024);
#else
  // for customers, play mode depends on whether spd plan data exists
  playMode = PLAYMODE::LOG;
  ReleaseWrapper();
#endif
  return 0;
}