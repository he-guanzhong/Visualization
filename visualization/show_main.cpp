/* Autonomous Driving Data Visualization Tool
 * All rights reserved by hgz */

#include "visualization/show_main.h"

#ifdef SPEED_PLANNING_H_
extern SsmFrameType g_ssmFrameType;
extern uint8 g_truncated_col;
extern float VfPASP_TimeUsed_ms;
#else
SsmFrameType g_ssmFrameType;
uint8 g_truncated_col;
#endif

// Key display information
// float ego_coeffs[8] = {6.0113e-10, -1.707e-7, 1.4243e-5, 0, 0, 0, 0,
// 120};changeleft
float egoAcc, egoSpd, spdLmt;
float ego_coeffs[8] = {0, 0, 0, 0, 0, 0, 0, 120};  // go straight
Point s_points[6], v_points[6], a_points[6];
Point ctrlPoint;
bool AlcLgtCtrlEnbl;
int accMode;
float left_coeffs[8] = {0, 0, 0, 0, 0, 3.4f / 2.0f, -30, 120};
float leftleft_coeffs[8] = {0, 0, 0, 0, 0, 3.4f * 1.5f, -30, 120};
float right_coeffs[8] = {0, 0, 0, 0, 0, -3.4f / 2.0f, -30, 120};
float rightright_coeffs[8] = {0, 0, 0, 0, 0, -3.4f * 1.5f, -30, 120};
TsrInfo tsr_info;
float left_coeffs_me[8];
float leftleft_coeffs_me[8];
float right_coeffs_me[8];
float rightright_coeffs_me[8];

// temporary storage of log data
PLAYMODE playMode;
int totalTime = DATA_NUM;
char csvFileName[150];

float time_data[DATA_NUM];
float egoSpd_data[DATA_NUM];
float egoAcc_data[DATA_NUM];
float spdLmt_data[DATA_NUM];
float Alc_path_data[6][DATA_NUM];
int acc_mode_data[DATA_NUM];

bool AlcLgtCtrlEnbl_data[DATA_NUM];
int truncated_col_data[DATA_NUM];
float ctrl_point_data[2][DATA_NUM];
float s_points_data[6][DATA_NUM];
float v_points_data[6][DATA_NUM];
float a_points_data[6][DATA_NUM];
float t_points_data[6][DATA_NUM];

bool objs_valid_flag_data[10][DATA_NUM] = {0};
int objs_lane_index_data[10][DATA_NUM] = {0};
int objs_type_data[10][DATA_NUM] = {0};
float objs_pos_x_data[10][DATA_NUM] = {0};
float objs_pos_y_data[10][DATA_NUM] = {0};
float objs_speed_x_data[10][DATA_NUM] = {0};
float objs_speed_y_data[10][DATA_NUM] = {0};
float objs_pos_yaw_data[10][DATA_NUM] = {0};

int tsr_spd_data[DATA_NUM];
bool tsr_spd_warn_data[DATA_NUM];
int tsr_tsi_data[2][DATA_NUM];

bool tsr_valid_flag_data[3][DATA_NUM] = {0};
int tsr_type_data[3][DATA_NUM] = {0};
float tsr_pos_x_data[3][DATA_NUM] = {0};
float tsr_pos_y_data[3][DATA_NUM] = {0};

float ll_path_data[8][DATA_NUM] = {0};
float l_path_data[8][DATA_NUM] = {0};
float r_path_data[8][DATA_NUM] = {0};
float rr_path_data[8][DATA_NUM] = {0};
float ll_path_me_data[8][DATA_NUM] = {0};
float l_path_me_data[8][DATA_NUM] = {0};
float r_path_me_data[8][DATA_NUM] = {0};
float rr_path_me_data[8][DATA_NUM] = {0};

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

  // use alc c7 as line end
  ego_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
}

void ReadInputData(const int t) {
  egoSpd = egoSpd_data[t];
  egoAcc = egoAcc_data[t];
  spdLmt = spdLmt_data[t];
  accMode = acc_mode_data[t];

  // c0->c5 orders of ego and alc_path are opposite
  for (int i = 1; i <= 5; i++)
    ego_coeffs[i] = Alc_path_data[5 - i][t];

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
    if (playMode == PLAYMODE::FUSION) {
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
  char szTotalTime_s[8];
  int tt_m = (int)time / 60;
  float tt_s = fmod(time, 60);
  if (tt_m) {
    sprintf(str, "%d", tt_m);
    strcat(str, "m ");
    sprintf(szTotalTime_s, "%.2f", tt_s);
    strcat(szTotalTime_s, "s");
    strcat(str, szTotalTime_s);
  } else {
    sprintf(str, "%.2f", tt_s);
    strcat(str, "s");
  }
}

void DisplayLog(const int length, const int width, const int offset) {
  if (totalTime <= 0)
    return;
  initgraph(length, width);
  setbkcolor(WHITE);
  setbkmode(TRANSPARENT);
  cleardevice();

  ExMessage msg;
  bool playSwitch = true;
  bool refleshScreen = false;
  int t = 0;
  int cycle = 50;  // unit: ms
  char playStatus[10] = "Playing";

  BeginBatchDraw();
  while (1) {
    // mouse clicks to change play status
    if (peekmessage(&msg, EX_MOUSE)) {
      if (msg.message == WM_LBUTTONDOWN && (msg.y < 0.95f * width) &&
          (msg.y > 0.25f * width)) {
        playSwitch = !playSwitch;
        memset(playStatus, '\0', sizeof(playStatus));
        strcpy(playStatus, playSwitch ? "Playing" : "Paused");
        // printf("%s\n", playStatus);
      } else if (msg.message == WM_LBUTTONDOWN && (msg.y > 0.95f * width)) {
        float rate = (float)msg.x / (float)length;
        t = rate * totalTime;
        refleshScreen = true;
      } else if (msg.message == WM_RBUTTONDOWN) {
        break;
      }

    } else {
      if (playSwitch || refleshScreen) {
        cleardevice();
        refleshScreen = false;

        // time
        t = (t == totalTime - 1 ? 1 : ++t);
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
        solidrectangle(0, width - 5, (float)t / (float)totalTime * length,
                       width);
        char szTotalTime[13];
        Time2Str(time_data[totalTime - 1], szTotalTime);

        outtextxy(length - textwidth(szTotalTime),
                  width - 5 - textheight(szTotalTime), szTotalTime);

        ReadInputData(t);
        ReadOutputData(t);

        LinesInfo lines_info = {
            ego_coeffs,         left_coeffs,       leftleft_coeffs,
            right_coeffs,       rightright_coeffs, left_coeffs_me,
            leftleft_coeffs_me, right_coeffs_me,   rightright_coeffs_me};
        SpdInfo spd_info = {egoSpd, fmax(v_points[4].y, v_points[5].y), spdLmt,
                            accMode};

        if (playMode == PLAYMODE::FUSION) {
          showBEVGraph(length, width, offset, 0, 0, 0.0f, 100.0f, &tsr_info,
                       &g_ssmFrameType, &lines_info, &spd_info);
        } else {
          showBEVGraph(length / 2, width, offset, length / 2, 0, 30.0f, 130.0f,
                       &tsr_info, &g_ssmFrameType, &lines_info, &spd_info);
          showSTGraph(length / 2, width / 2, offset, 0, 0, 0.0f, 5.0f, 120.0f,
                      "S-T Graph", BLUE, s_points, &ctrlPoint);
          showSTGraph(length / 2, width * 0.4, offset, 0, width * 0.49 - offset,
                      0.0f, 5.0f, 36.0f, "V-T", BLUE, v_points, &ctrlPoint);
          showSTGraph(length / 2, width * 0.4, offset, 0, width * 0.75 - offset,
                      4.0f, 5.0f, 6.0f, "A-T", RED, a_points, &ctrlPoint);

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
        FlushBatchDraw();
        setbkmode(TRANSPARENT);
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
    egoSpd = 25.0f, egoAcc = 0.0f, spdLmt = 33.3f;
    DummySsmData(&ssmFrame);
  } else {
    ssmFrame = g_ssmFrameType;
  }

  DpSpeedPoints output =
      SpeedPlanProcessor(egoSpd, egoAcc, spdLmt, &alcPathVcc,
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

  s_points[0] = {output.point0.t, output.point0.s};
  s_points[1] = {output.point1.t, output.point1.s};
  s_points[2] = {output.point2.t, output.point2.s};
  s_points[3] = {output.point3.t, output.point3.s};
  s_points[4] = {output.point4.t, output.point4.s};
  s_points[5] = {output.point5.t, output.point5.s};
  v_points[0] = {output.point0.t, output.point0.v};
  v_points[1] = {output.point1.t, output.point1.v};
  v_points[2] = {output.point2.t, output.point2.v};
  v_points[3] = {output.point3.t, output.point3.v};
  v_points[4] = {output.point4.t, output.point4.v};
  v_points[5] = {output.point5.t, output.point5.v};
  a_points[0] = {output.point0.t, output.point0.a};
  a_points[1] = {output.point1.t, output.point1.a};
  a_points[2] = {output.point2.t, output.point2.a};
  a_points[3] = {output.point3.t, output.point3.a};
  a_points[4] = {output.point4.t, output.point4.a};
  a_points[5] = {output.point5.t, output.point5.a};

  ctrlPoint = {output.pointCtrl.t, output.pointCtrl.a};
  AlcLgtCtrlEnbl = output.AlcLgtCtrlEnbl;
  /*   printf("Time: %.2f ms. \t Memory: %ld Byte \n", VfPASP_TimeUsed_ms,
           g_ALC_MEMORY_SIZE);
    printf("Direct: %d, Default result: \n", g_laneChangeDirection); */
  if (playMode == PLAYMODE::ONESTEP) {
    printf("Time: %.2f ms, g_truncated_col: %d\n", VfPASP_TimeUsed_ms,
           g_truncated_col);
    printf("AlcLatEnbl: %d, \t AlcLgtEnble: %d\n", output.AlcLatCtrlEnbl,
           output.AlcLgtCtrlEnbl);
    for (int i = 0; i <= 5; i++) {
      printf("Point%d: t = %.1f, s = %.1f, v = %.1f, a = %.1f \n", i,
             s_points[i].x, s_points[i].y, v_points[i].y, a_points[i].y);
    }
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
  SpdInfo spd_info = {v_points[0].y, fmax(v_points[4].y, v_points[5].y), spdLmt,
                      accMode};

  showBEVGraph(length / 2, width, offset, length / 2, 0, 30.0f, 150.0f,
               &tsr_info, &g_ssmFrameType, &lines_info, &spd_info);
  showSTGraph(length / 2, width / 2, offset, 0, 0, 0.0f, 5.0f, 120.0f,
              "S-T Graph", BLUE, s_points, &ctrlPoint);
  showSTGraph(length / 2, width * 0.4, offset, 0, width * 0.49 - offset, 0.0f,
              5.0f, 36.0f, "V-T", BLUE, v_points, &ctrlPoint);
  showSTGraph(length / 2, width * 0.4, offset, 0, width * 0.75 - offset, 4.0f,
              5.0f, 6.0f, "A-T", RED, a_points, &ctrlPoint);

  system("pause");
  closegraph();
}

void LoopbackCalculation() {
  for (int t = 0; t < totalTime; t++) {
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

    ego_coeffs[7] = fmax(s_points[4].y, s_points[5].y);
  }
}

void GenerateLocalData() {
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
  DummySsmData(&ssmFrame);

  egoSpd = 25.0f, egoAcc = 0.0f, spdLmt = 33.3f;
  float cycle_s = 0.1f;
  float obs_pos_x[10] = {0};
  float obs_pos_y[10] = {0};
  float obs_speed_x[10] = {0};
  float obs_speed_y[10] = {0};
  float accResponseDelay[5] = {0};
  int accDelay_pos = 0;

  totalTime = 300;
  for (int t = 0; t < totalTime; t++) {
    time_data[t] = t * cycle_s;

    egoAcc = accResponseDelay[accDelay_pos];
    egoSpd += egoAcc * cycle_s;
    spdLmt_data[t] = spdLmt;
    egoAcc_data[t] = egoAcc;
    egoSpd_data[t] = egoSpd;

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
      objs_pos_yaw_data[k][t] = ssmFrame.Ssm_Objs_Frame_st.obj_lists[k].pos_yaw;
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
        SpeedPlanProcessor(egoSpd, egoAcc, spdLmt, &alcPathVcc,
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
    s_points[0] = {output.point0.t, output.point0.s};
    s_points[1] = {output.point1.t, output.point1.s};
    s_points[2] = {output.point2.t, output.point2.s};
    s_points[3] = {output.point3.t, output.point3.s};
    s_points[4] = {output.point4.t, output.point4.s};
    s_points[5] = {output.point5.t, output.point5.s};
    v_points[0] = {output.point0.t, output.point0.v};
    v_points[1] = {output.point1.t, output.point1.v};
    v_points[2] = {output.point2.t, output.point2.v};
    v_points[3] = {output.point3.t, output.point3.v};
    v_points[4] = {output.point4.t, output.point4.v};
    v_points[5] = {output.point5.t, output.point5.v};
    a_points[0] = {output.point0.t, output.point0.a};
    a_points[1] = {output.point1.t, output.point1.a};
    a_points[2] = {output.point2.t, output.point2.a};
    a_points[3] = {output.point3.t, output.point3.a};
    a_points[4] = {output.point4.t, output.point4.a};
    a_points[5] = {output.point5.t, output.point5.a};
    a_points[5] = {output.point5.t, output.point5.a};
    ctrlPoint = {output.pointCtrl.t, output.pointCtrl.a};
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
  }
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
    printf("Selected file: %s\n", ofn.lpstrFile);
    strcpy(csvFileName, ofn.lpstrFile);
    LoadLog();
#ifdef SPEED_PLANNING_H_
    if (playMode == PLAYMODE::LOOPBACK)
      LoopbackCalculation();
#endif
    int length = playMode == PLAYMODE::FUSION ? 400 : 750;
    DisplayLog(length, 750, 100);
  } else {
    printf("No file selected\n");
    system("pause");
  }
  return;
}

int main() {
#ifdef SPEED_PLANNING_H_
  // for speed planner, 3 functions: replay, loopback and simulation
  playMode = PLAYMODE(1);
  switch (playMode) {
    case ONESTEP:
      CalcOneStep();
      DisplayOneStep(750, 750, 100);
      break;
    case LOG:
    case LOOPBACK:
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
  int memoryCost = sizeof(time_data) + sizeof(egoSpd_data) * 3 +
                   sizeof(Alc_path_data) + sizeof(ctrl_point_data) +
                   sizeof(truncated_col_data) * 2 + sizeof(s_points_data) * 4 +
                   sizeof(objs_valid_flag_data) + sizeof(objs_lane_index_data) +
                   sizeof(objs_type_data) + sizeof(objs_pos_x_data) * 5 +
                   sizeof(tsr_spd_data) * 3 + sizeof(tsr_spd_warn_data) +
                   sizeof(tsr_valid_flag_data) + sizeof(tsr_type_data) +
                   2 * sizeof(tsr_pos_x_data) + sizeof(ll_path_data) * 8;
// printf("Memory: %d kB\n", memoryCost / 1024);
#else
  // for customers, fix play mode to FUSION
  playMode = PLAYMODE::FUSION;
  ReleaseWrapper();
#endif
  return 0;
}