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

int iObjectId_data[32][DATA_NUM];
float fDistX_data[32][DATA_NUM];
float fDistY_data[32][DATA_NUM];

void RadarDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame) {
  int Ts = 0;
  int ID[32] = {0}, DIS_X[32] = {0}, DIS_Y[32] = {0};
  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "t[s]") == 0 ||
        strcmp(columns[i], "timestamps") == 0)
      Ts = i;

    for (int j = 0; j < 32; j++) {
      char obs_title[25] = "iObjectId_";
      char disX_title[25] = "fDistX_";
      char disY_title[25] = "fDistY_";
      snprintf(obs_title + strlen(obs_title),
               sizeof(obs_title) - strlen(obs_title), "%02d[]", j);
      snprintf(disX_title + strlen(disX_title),
               sizeof(disX_title) - strlen(disX_title), "%02d[m]", j);
      snprintf(disY_title + strlen(disY_title),
               sizeof(disY_title) - strlen(disY_title), "%02d[m]", j);

      if (strncmp(columns[i], obs_title, 12) == 0) {
        ID[j] = i;
      } else if (strncmp(columns[i], disX_title, 9) == 0) {
        DIS_X[j] = i;
      } else if (strncmp(columns[i], disY_title, 9) == 0) {
        DIS_Y[j] = i;
      }
    }
  }

  *totalFrame = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;
  /*   for (int j = 0; j < 32; j++) {
      printf("ID[%d] = %d\n", j, ID[j]);
    } */
  for (int t = 0; t < *totalFrame; t++) {
    time_data[t] = values[Ts][t];
    for (int j = 0; j < 32; j++) {
      if (ID[j] == 0)
        continue;
      iObjectId_data[j][t] = values[ID[j]][t];
      fDistX_data[j][t] = values[DIS_X[j]][t];
      fDistY_data[j][t] = values[DIS_Y[j]][t];
    }
  }
  return;
}
#endif
