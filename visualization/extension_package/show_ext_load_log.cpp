#include "visualization/extension_package/show_ext_load_log.h"

#ifdef AGSM_DEMO_TEST
extern float time_data[DATA_NUM];

float VehSpd_data[DATA_NUM];
float LeftLine_data[8][DATA_NUM];
float RightLine_data[8][DATA_NUM];
float NextLeftLine_data[8][DATA_NUM];
float NextRightLine_data[8][DATA_NUM];

float ConftPathTypC0_data[DATA_NUM];
float ConftPathTypC1_data[DATA_NUM];
float ConftPathTypC2_data[DATA_NUM];
float ConftPathTypC31_data[DATA_NUM];
float ConftPathTypC32_data[DATA_NUM];
float ConftPathTypC33_data[DATA_NUM];
float ConftPathLength1_data[DATA_NUM];
float ConftPathLength2_data[DATA_NUM];
float ConftPathLength3_data[DATA_NUM];
int ConftPathValid_data[DATA_NUM];

bool Obstacles_valid_flag_data[10][DATA_NUM];
int Obstacles_lane_index_data[10][DATA_NUM];
int Obstacles_type_data[10][DATA_NUM];
float Obstacles_pos_x_data[10][DATA_NUM];
float Obstacles_pos_y_data[10][DATA_NUM];
float Obstacles_speed_x_data[10][DATA_NUM];
float Obstacles_speed_y_data[10][DATA_NUM];
float Obstacles_acc_x_data[10][DATA_NUM];
float Obstacles_pos_yaw_data[10][DATA_NUM];

void AgsmDataParsing(float** values,
                     int numColumns,
                     char** columns,
                     int* valuesCount,
                     int* totalFrame) {
  int Ts = 0, EGO_V = 0;
  int IVS_Present[10] = {0}, IVS_Class[10] = {0}, IVS_LaDis[10] = {0},
      IVS_LgDis[10] = {0}, IVS_V[10] = {0}, IVS_LaV[10] = {0}, IVS_A[10] = {0},
      IVS_Yaw[10] = {0};
  int CFT_PATH[10] = {0};
  int LH_C_0[8] = {0}, LH_C_1[8] = {0}, LA_C_0[8] = {0}, LA_C_1[8] = {0};

  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "t[s]") == 0 ||
        strcmp(columns[i], "timestamps") == 0)
      Ts = i;
    else if (strcmp(columns[i], "VfAGSM_VehicleSpd_mps") == 0)
      EGO_V = i;

    else if (strcmp(columns[i], "VfAGSM_ConftPathTypC0_m") == 0)
      CFT_PATH[0] = i;
    else if (strcmp(columns[i], "VfAGSM_ConftPathTypC1_rad") == 0)
      CFT_PATH[1] = i;
    else if (strcmp(columns[i], "VfAGSM_ConftPathTypC2_co") == 0)
      CFT_PATH[2] = i;
    else if (strcmp(columns[i], "VfAGSM_1ConftPathTypC3_co") == 0)
      CFT_PATH[3] = i;
    else if (strcmp(columns[i], "VfAGSM_2ConftPathTypC3_co") == 0)
      CFT_PATH[4] = i;
    else if (strcmp(columns[i], "VfAGSM_3ConftPathTypC3_co") == 0)
      CFT_PATH[5] = i;
    else if (strcmp(columns[i], "VfAGSM_1ConftPathLength_m") == 0)
      CFT_PATH[6] = i;
    else if (strcmp(columns[i], "VfAGSM_2ConftPathLength_m") == 0)
      CFT_PATH[7] = i;
    else if (strcmp(columns[i], "VfAGSM_3ConftPathLength_m") == 0)
      CFT_PATH[8] = i;
    else if (strcmp(columns[i], "VeAGSM_ConftPathValid_enum") == 0)
      CFT_PATH[9] = i;

    // obstacle, 0 = IV
    else if (strcmp(columns[i], "VeINP_IVClass") == 0)
      IVS_Class[0] = i;
    else if (strcmp(columns[i], "VfINP_IVLaDis_m") == 0)
      IVS_LaDis[0] = i;
    else if (strcmp(columns[i], "VfINP_IVLgDis_m") == 0)
      IVS_LgDis[0] = i;
    else if (strcmp(columns[i], "VbINP_IVPresent_flg") == 0)
      IVS_Present[0] = i;
    else if (strcmp(columns[i], "VfINP_IVV_mps") == 0)
      IVS_V[0] = i;
    else if (strcmp(columns[i], "VfINP_IVLaSpd_mps") == 0)
      IVS_LaV[0] = i;
    else if (strcmp(columns[i], "VfINP_IVAcc_mpss") == 0)
      IVS_A[0] = i;
    else if (strcmp(columns[i], "VfINP_IVHeading_rad") == 0)
      IVS_Yaw[0] = i;

    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0C0_m") == 0)
      LH_C_0[0] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1C0_m") == 0)
      LH_C_1[0] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0C1_rad") == 0)
      LH_C_0[1] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1C1_rad") == 0)
      LH_C_1[1] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0C2_co") == 0)
      LH_C_0[2] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1C2_co") == 0)
      LH_C_1[2] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0C3_co") == 0)
      LH_C_0[3] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1C3_co") == 0)
      LH_C_1[3] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0StRange_m") == 0)
      LH_C_0[6] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1StRange_m") == 0)
      LH_C_1[6] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr0EdRange_m") == 0)
      LH_C_0[7] = i;
    else if (strcmp(columns[i], "VfAGSM_LHLaneMkr1EdRange_m") == 0)
      LH_C_1[7] = i;
  }

  *totalFrame = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;
  for (int t = 0; t < *totalFrame; t++) {
    time_data[t] = values[Ts][t];

    VehSpd_data[t] = EGO_V ? values[EGO_V][t] : 0;

    // ME original lines. c0~c3 opposite, c4 and c5 ignored, for no input
    for (int k = 0; k < 8; k++) {
      if (LH_C_0[k] == 0)
        continue;
      float correct_fac = (k <= 3 ? -1 : 1);
      if (k <= 3 || k >= 6) {
        LeftLine_data[k][t] = values[LH_C_0[k]][t] * correct_fac;
        RightLine_data[k][t] = values[LH_C_1[k]][t] * correct_fac;
        NextLeftLine_data[k][t] = values[LA_C_0[k]][t] * correct_fac;
        NextRightLine_data[k][t] = values[LA_C_1[k]][t] * correct_fac;
      }
    }

    if (CFT_PATH[0]) {
      ConftPathTypC0_data[t] = values[CFT_PATH[0]][t];
      ConftPathTypC1_data[t] = values[CFT_PATH[1]][t];
      ConftPathTypC2_data[t] = values[CFT_PATH[2]][t];
      ConftPathTypC31_data[t] = values[CFT_PATH[3]][t];
      ConftPathTypC32_data[t] = values[CFT_PATH[4]][t];
      ConftPathTypC33_data[t] = values[CFT_PATH[5]][t];
      ConftPathLength1_data[t] = values[CFT_PATH[6]][t];
      ConftPathLength2_data[t] = values[CFT_PATH[7]][t];
      ConftPathLength3_data[t] = values[CFT_PATH[8]][t];
      ConftPathValid_data[t] = values[CFT_PATH[9]][t];
    }

    float pos_x_compensation = 0;
    for (int k = 0; k < 10; k++) {
      if (IVS_Present[k] == 0)
        continue;

      // rear obs lat spd not stable, unacceptable
      if (0 == k || 2 == k || 3 == k || 6 == k || 7 == k) {
        pos_x_compensation =
            2.5f + (Obstacles_type_data[k][t] == 2 ? 10.0f : 5.0f) / 2.0f;
        Obstacles_speed_y_data[k][t] =
            IVS_LaV[k] ? values[IVS_LaV[k]][t] * -1 : 0;
      } else {
        pos_x_compensation = 2.5f;
        // Obstacles_speed_y_data[k][t] = IVS_LaV[k] ? values[IVS_LaV[k]][t] *
        // -1 : 0;
      }
      Obstacles_valid_flag_data[k][t] = values[IVS_Present[k]][t];
      Obstacles_type_data[k][t] = values[IVS_Class[k]][t];
      Obstacles_pos_x_data[k][t] = values[IVS_LgDis[k]][t] + pos_x_compensation;
      Obstacles_pos_y_data[k][t] = values[IVS_LaDis[k]][t] * -1;
      Obstacles_speed_x_data[k][t] = values[IVS_V[k]][t];
      Obstacles_acc_x_data[k][t] = values[IVS_A[k]][t];
      Obstacles_pos_yaw_data[k][t] = values[IVS_Yaw[k]][t] * -1;

      if (k <= 1)
        Obstacles_lane_index_data[k][t] = 3;
      else if (k <= 5)
        Obstacles_lane_index_data[k][t] = 2;
      else
        Obstacles_lane_index_data[k][t] = 4;
    }
  }

  return;
}

#endif

#ifdef RADAR_DEMO_TEST

extern float time_data[DATA_NUM];

int iFL_ObjectId_data[OBJ_NUM][DATA_NUM];
float fFL_ExistProb_data[OBJ_NUM][DATA_NUM];
float fFL_DistX_data[OBJ_NUM][DATA_NUM];
float fFL_DistY_data[OBJ_NUM][DATA_NUM];

int iFR_ObjectId_data[OBJ_NUM][DATA_NUM];
float fFR_ExistProb_data[OBJ_NUM][DATA_NUM];
float fFR_DistX_data[OBJ_NUM][DATA_NUM];
float fFR_DistY_data[OBJ_NUM][DATA_NUM];

int iRL_ObjectId_data[OBJ_NUM][DATA_NUM];
float fRL_ExistProb_data[OBJ_NUM][DATA_NUM];
float fRL_DistX_data[OBJ_NUM][DATA_NUM];
float fRL_DistY_data[OBJ_NUM][DATA_NUM];

int iRR_ObjectId_data[OBJ_NUM][DATA_NUM];
float fRR_ExistProb_data[OBJ_NUM][DATA_NUM];
float fRR_DistX_data[OBJ_NUM][DATA_NUM];
float fRR_DistY_data[OBJ_NUM][DATA_NUM];

void RadarDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame) {
  int Ts = 0;
  int FL_ID[OBJ_NUM] = {0}, FL_EXI_PB[OBJ_NUM] = {0}, FL_DIS_X[OBJ_NUM] = {0},
      FL_DIS_Y[OBJ_NUM] = {0};
  int FR_ID[OBJ_NUM] = {0}, FR_EXI_PB[OBJ_NUM] = {0}, FR_DIS_X[OBJ_NUM] = {0},
      FR_DIS_Y[OBJ_NUM] = {0};
  int RL_ID[OBJ_NUM] = {0}, RL_EXI_PB[OBJ_NUM] = {0}, RL_DIS_X[OBJ_NUM] = {0},
      RL_DIS_Y[OBJ_NUM] = {0};
  int RR_ID[OBJ_NUM] = {0}, RR_EXI_PB[OBJ_NUM] = {0}, RR_DIS_X[OBJ_NUM] = {0},
      RR_DIS_Y[OBJ_NUM] = {0};
  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "timestamps") == 0)
      Ts = i;

    for (int j = 0; j < OBJ_NUM; j++) {
      char obs_title[70] = "AswIf_SRRDataFrontLeft.SyncTrack_s.radar_track_s._";
      char exiP_title[85] =
          "AswIf_SRRDataFrontLeft.SyncTrack_s.radar_track_s._";
      char disX_title[70] =
          "AswIf_SRRDataFrontLeft.SyncTrack_s.radar_track_s._";
      char disY_title[70] =
          "AswIf_SRRDataFrontLeft.SyncTrack_s.radar_track_s._";

      const int obs_len = strlen(obs_title);
      const int ext_len = strlen(exiP_title);
      const int disx_len = strlen(disX_title);
      const int dixy_len = strlen(disY_title);

      snprintf(obs_title + obs_len, sizeof(obs_title) - obs_len, "%d_.ID_u8",
               j);
      snprintf(exiP_title + ext_len, sizeof(exiP_title) - ext_len,
               "%d_.existance_probability_f32", j);
      snprintf(disX_title + disx_len, sizeof(disX_title) - disx_len,
               "%d_.x_pos_f32", j);
      snprintf(disY_title + dixy_len, sizeof(disY_title) - dixy_len,
               "%d_.y_pos_f32", j);

      if (strcmp(columns[i], obs_title) == 0) {
        FL_ID[j] = i;
      } else if (strcmp(columns[i], exiP_title) == 0) {
        FL_EXI_PB[j] = i;
      } else if (strcmp(columns[i], disX_title) == 0) {
        FL_DIS_X[j] = i;
      } else if (strcmp(columns[i], disY_title) == 0) {
        FL_DIS_Y[j] = i;
      }
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      char obs_title[70] =
          "AswIf_SRRDataFrontRight.SyncTrack_s.radar_track_s._";
      char exiP_title[85] =
          "AswIf_SRRDataFrontRight.SyncTrack_s.radar_track_s._";
      char disX_title[70] =
          "AswIf_SRRDataFrontRight.SyncTrack_s.radar_track_s._";
      char disY_title[70] =
          "AswIf_SRRDataFrontRight.SyncTrack_s.radar_track_s._";
      const int obs_len = strlen(obs_title);
      const int ext_len = strlen(exiP_title);
      const int disx_len = strlen(disX_title);
      const int dixy_len = strlen(disY_title);
      snprintf(obs_title + obs_len, sizeof(obs_title) - obs_len, "%d_.ID_u8",
               j);
      snprintf(exiP_title + ext_len, sizeof(exiP_title) - ext_len,
               "%d_.existance_probability_f32", j);
      snprintf(disX_title + disx_len, sizeof(disX_title) - disx_len,
               "%d_.x_pos_f32", j);
      snprintf(disY_title + dixy_len, sizeof(disY_title) - dixy_len,
               "%d_.y_pos_f32", j);

      if (strcmp(columns[i], obs_title) == 0) {
        FR_ID[j] = i;
      } else if (strcmp(columns[i], exiP_title) == 0) {
        FR_EXI_PB[j] = i;
      } else if (strcmp(columns[i], disX_title) == 0) {
        FR_DIS_X[j] = i;
      } else if (strcmp(columns[i], disY_title) == 0) {
        FR_DIS_Y[j] = i;
      }
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      char obs_title[70] = "AswIf_SRRDataRearLeft.SyncTrack_s.radar_track_s._";
      char exiP_title[85] = "AswIf_SRRDataRearLeft.SyncTrack_s.radar_track_s._";
      char disX_title[70] = "AswIf_SRRDataRearLeft.SyncTrack_s.radar_track_s._";
      char disY_title[70] = "AswIf_SRRDataRearLeft.SyncTrack_s.radar_track_s._";
      const int obs_len = strlen(obs_title);
      const int ext_len = strlen(exiP_title);
      const int disx_len = strlen(disX_title);
      const int dixy_len = strlen(disY_title);
      snprintf(obs_title + obs_len, sizeof(obs_title) - obs_len, "%d_.ID_u8",
               j);
      snprintf(exiP_title + ext_len, sizeof(exiP_title) - ext_len,
               "%d_.existance_probability_f32", j);
      snprintf(disX_title + disx_len, sizeof(disX_title) - disx_len,
               "%d_.x_pos_f32", j);
      snprintf(disY_title + dixy_len, sizeof(disY_title) - dixy_len,
               "%d_.y_pos_f32", j);

      if (strcmp(columns[i], obs_title) == 0) {
        RL_ID[j] = i;
      } else if (strcmp(columns[i], exiP_title) == 0) {
        RL_EXI_PB[j] = i;
      } else if (strcmp(columns[i], disX_title) == 0) {
        RL_DIS_X[j] = i;
      } else if (strcmp(columns[i], disY_title) == 0) {
        RL_DIS_Y[j] = i;
      }
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      char obs_title[70] = "AswIf_SRRDataRearRight.SyncTrack_s.radar_track_s._";
      char exiP_title[85] =
          "AswIf_SRRDataRearRight.SyncTrack_s.radar_track_s._";
      char disX_title[70] =
          "AswIf_SRRDataRearRight.SyncTrack_s.radar_track_s._";
      char disY_title[70] =
          "AswIf_SRRDataRearRight.SyncTrack_s.radar_track_s._";
      const int obs_len = strlen(obs_title);
      const int ext_len = strlen(exiP_title);
      const int disx_len = strlen(disX_title);
      const int dixy_len = strlen(disY_title);
      snprintf(obs_title + obs_len, sizeof(obs_title) - obs_len, "%d_.ID_u8",
               j);
      snprintf(exiP_title + ext_len, sizeof(exiP_title) - ext_len,
               "%d_.existance_probability_f32", j);
      snprintf(disX_title + disx_len, sizeof(disX_title) - disx_len,
               "%d_.x_pos_f32", j);
      snprintf(disY_title + dixy_len, sizeof(disY_title) - dixy_len,
               "%d_.y_pos_f32", j);

      if (strcmp(columns[i], obs_title) == 0) {
        RR_ID[j] = i;
      } else if (strcmp(columns[i], exiP_title) == 0) {
        RR_EXI_PB[j] = i;
      } else if (strcmp(columns[i], disX_title) == 0) {
        RR_DIS_X[j] = i;
      } else if (strcmp(columns[i], disY_title) == 0) {
        RR_DIS_Y[j] = i;
      }
    }
  }

  *totalFrame = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;
  /*   for (int j = 0; j < OBJ_NUM; j++) {
      printf("FL_ID[%d] = %d\t %d, %d, %d\n", j, FL_ID[j], FR_ID[j], RL_ID[j],
             RR_ID[j]);
    } */
  for (int t = 0; t < *totalFrame; t++) {
    time_data[t] = values[Ts][t];

    for (int j = 0; j < OBJ_NUM; j++) {
      if (FL_ID[j] == 0)
        continue;
      iFL_ObjectId_data[j][t] = values[FL_ID[j]][t];
      fFL_ExistProb_data[j][t] = values[FL_EXI_PB[j]][t];
      fFL_DistX_data[j][t] = values[FL_DIS_X[j]][t];
      fFL_DistY_data[j][t] = values[FL_DIS_Y[j]][t];
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      if (FR_ID[j] == 0)
        continue;
      iFR_ObjectId_data[j][t] = values[FR_ID[j]][t];
      fFR_ExistProb_data[j][t] = values[FR_EXI_PB[j]][t];
      fFR_DistX_data[j][t] = values[FR_DIS_X[j]][t];
      fFR_DistY_data[j][t] = values[FR_DIS_Y[j]][t];
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      if (RL_ID[j] == 0)
        continue;
      iRL_ObjectId_data[j][t] = values[RL_ID[j]][t];
      fRL_ExistProb_data[j][t] = values[RL_EXI_PB[j]][t];
      fRL_DistX_data[j][t] = values[RL_DIS_X[j]][t];
      fRL_DistY_data[j][t] = values[RL_DIS_Y[j]][t];
    }

    for (int j = 0; j < OBJ_NUM; j++) {
      if (RR_ID[j] == 0)
        continue;
      iRR_ObjectId_data[j][t] = values[RR_ID[j]][t];
      fRR_ExistProb_data[j][t] = values[RR_EXI_PB[j]][t];
      fRR_DistX_data[j][t] = values[RR_DIS_X[j]][t];
      fRR_DistY_data[j][t] = values[RR_DIS_Y[j]][t];
    }
  }
  return;
}
#endif
