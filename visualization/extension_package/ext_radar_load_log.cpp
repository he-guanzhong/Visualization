#include "visualization/extension_package/ext_radar_load_log.h"

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
  int IVS_ID[10] = {0}, IVS_Present[10] = {0}, IVS_Class[10] = {0},
      IVS_LaDis[10] = {0}, IVS_LgDis[10] = {0}, IVS_V[10] = {0},
      IVS_LaV[10] = {0}, IVS_A[10] = {0}, IVS_Yaw[10] = {0}, IVS_CUT[10] = {0},
      IVS_LaVF[10] = {0};

  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "t[s]") == 0 ||
        strcmp(columns[i], "timestamps") == 0)
      Ts = i;
    else if (strcmp(columns[i], "VfAGSM_VehicleSpd_mps") == 0)
      EGO_V = i;

    // obstacle, 0 = IV
    else if (strncmp(columns[i], "VeINP_ExeedIVClass[enum]", 18) == 0)
      IVS_Class[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVLaDis_m[m]", 20) == 0)
      IVS_LaDis[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVLgDis_m[m]", 20) == 0)
      IVS_LgDis[0] = i;
    else if (strncmp(columns[i], "VbINP_ExeedIVPresent_flg[flg]", 24) == 0)
      IVS_Present[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVV_mps[mps]", 18) == 0)
      IVS_V[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVLaSpd_mps[]", 22) == 0)
      IVS_LaV[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVHeading_rad[]", 24) == 0)
      IVS_Yaw[0] = i;
    else if (strncmp(columns[i], "VfINP_ExeedIVAcc_mpss[]", 21) == 0)
      IVS_A[0] = i;

    // obstacle, 1 = RIV
    else if (strncmp(columns[i], "VeINP_ExeedRIVClass_enum[enum]", 24) == 0)
      IVS_Class[1] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLaDis_m[m]", 21) == 0)
      IVS_LaDis[1] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLgDis_m[m]", 21) == 0)
      IVS_LgDis[1] = i;
    else if (strncmp(columns[i], "VbINP_ExeedRIVPresent_flg[flg]", 25) == 0)
      IVS_Present[1] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVV_mps[mps]", 19) == 0)
      IVS_V[1] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLaV_mps[mps]", 21) == 0)
      IVS_LaV[1] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVHeading_rad[]", 25) == 0)
      IVS_Yaw[1] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedRIVAcc_mpss[]", 23) == 0)
      IVS_A[1] = i;

    // obstacle, 2 = NIVL
    else if (strncmp(columns[i], "VeINP_ExeedNIVLClass_enum[enum]", 25) == 0)
      IVS_Class[2] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVLLaDis_m[m]", 22) == 0)
      IVS_LaDis[2] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVLLgDis_m[m]", 22) == 0)
      IVS_LgDis[2] = i;
    else if (strncmp(columns[i], "VbINP_ExeedNIVLPresent_flg[flg]", 26) == 0)
      IVS_Present[2] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVLV_mps[mps]", 20) == 0)
      IVS_V[2] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVLLaV_mps[mps]", 22) == 0)
      IVS_LaV[2] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVLHeading_rad[]", 26) == 0)
      IVS_Yaw[2] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedNIVLAcc_mpss[]", 24) == 0)
      IVS_A[2] = i;
    else if (strncmp(columns[i], "VePASP_ExeedNIVLCutIn[]", 21) == 0)
      IVS_CUT[2] = i;

    // obstacle, 6 = NIVR
    else if (strncmp(columns[i], "VeINP_ExeedNIVRClass_enum[enum]", 25) == 0)
      IVS_Class[6] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVRLaDis_m[m]", 22) == 0)
      IVS_LaDis[6] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVRLgDis_m[m]", 22) == 0)
      IVS_LgDis[6] = i;
    else if (strncmp(columns[i], "VbINP_ExeedNIVRPresent_flg[flg]", 26) == 0)
      IVS_Present[6] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVRV_mps[mps]", 20) == 0)
      IVS_V[6] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVRLaV_mps[mps]", 22) == 0)
      IVS_LaV[6] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIVRHeading_rad[]", 26) == 0)
      IVS_Yaw[6] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedNIVRAcc_mpss[]", 24) == 0)
      IVS_A[6] = i;
    else if (strncmp(columns[i], "VePASP_ExeedNIVRCutIn[]", 21) == 0)
      IVS_CUT[6] = i;

    // obstacle, 3 = NIIVL
    else if (strncmp(columns[i], "VeINP_ExeedNIIVLClass_enum[enum]", 26) == 0)
      IVS_Class[3] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVLLaDis_m[m]", 23) == 0)
      IVS_LaDis[3] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVLLgDis_m[m]", 23) == 0)
      IVS_LgDis[3] = i;
    else if (strncmp(columns[i], "VbINP_ExeedNIIVLPresent_flg[flg]", 27) == 0)
      IVS_Present[3] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVLV_mps[mps]", 21) == 0)
      IVS_V[3] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVLLaV_mps[mps]", 23) == 0)
      IVS_LaV[3] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVLHeading_rad[]", 27) == 0)
      IVS_Yaw[3] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedNIIVLAcc_mpss[]", 25) == 0)
      IVS_A[3] = i;
    else if (strncmp(columns[i], "VePASP_ExeedNIIVLCutIn[]", 22) == 0)
      IVS_CUT[3] = i;

    // obstacle, 7 = NIIVR
    else if (strncmp(columns[i], "VeINP_ExeedNIIVRClass_enum[enum]", 26) == 0)
      IVS_Class[7] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVRLaDis_m[m]", 23) == 0)
      IVS_LaDis[7] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVRLgDis_m[m]", 23) == 0)
      IVS_LgDis[7] = i;
    else if (strncmp(columns[i], "VbINP_ExeedNIIVRPresent_flg[flg]", 27) == 0)
      IVS_Present[7] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVRV_mps[mps]", 21) == 0)
      IVS_V[7] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVRLaV_mps[mps]", 23) == 0)
      IVS_LaV[7] = i;
    else if (strncmp(columns[i], "VfINP_ExeedNIIVRHeading_rad[]", 27) == 0)
      IVS_Yaw[7] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedNIIVRAcc_mpss[]", 25) == 0)
      IVS_A[7] = i;
    else if (strncmp(columns[i], "VePASP_ExeedNIIVRCutIn[]", 22) == 0)
      IVS_CUT[7] = i;

    // obstacle, 4 = RIVL
    else if (strncmp(columns[i], "VeINP_ExeedRIVLClass_enum[enum]", 25) == 0)
      IVS_Class[4] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLLaDis_m[m]", 22) == 0)
      IVS_LaDis[4] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLLgDis_m[m]", 22) == 0)
      IVS_LgDis[4] = i;
    else if (strncmp(columns[i], "VbINP_ExeedRIVLPresent_flg[flg]", 26) == 0)
      IVS_Present[4] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLV_mps[mps]", 20) == 0)
      IVS_V[4] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLLaV_mps[mps]", 22) == 0)
      IVS_LaV[4] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVLHeading_rad[]", 26) == 0)
      IVS_Yaw[4] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedRIVLAcc_mpss[]", 24) == 0)
      IVS_A[4] = i;

    // obstacle, 8 = RIVR
    else if (strncmp(columns[i], "VeINP_ExeedRIVRClass_enum[enum]", 25) == 0)
      IVS_Class[8] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVRLaDis_m[m]", 22) == 0)
      IVS_LaDis[8] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVRLgDis_m[m]", 22) == 0)
      IVS_LgDis[8] = i;
    else if (strncmp(columns[i], "VbINP_ExeedRIVRPresent_flg[flag]", 26) == 0)
      IVS_Present[8] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVRV_mps[mps]", 20) == 0)
      IVS_V[8] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVRLaV_mps[mps]", 22) == 0)
      IVS_LaV[8] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIVRHeading_rad[]", 26) == 0)
      IVS_Yaw[8] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedRIVRAcc_mpss[]", 24) == 0)
      IVS_A[8] = i;

    // obstacle, 5 = RIIVL
    else if (strncmp(columns[i], "VeINP_ExeedRIIVLClass_enum[enum]", 26) == 0)
      IVS_Class[5] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVLLaDis_m[m]", 23) == 0)
      IVS_LaDis[5] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVLLgDis_m[m]", 23) == 0)
      IVS_LgDis[5] = i;
    else if (strncmp(columns[i], "VbINP_ExeedRIIVLPresent_flg[flag]", 27) == 0)
      IVS_Present[5] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVLV_mps[mps]", 21) == 0)
      IVS_V[5] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVLLaV_mps[mps]", 23) == 0)
      IVS_LaV[5] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVLHeading_rad[]", 27) == 0)
      IVS_Yaw[5] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedRIIVLAcc_mpss[]", 25) == 0)
      IVS_A[5] = i;

    // obstacle, 9 = RIIVR
    else if (strncmp(columns[i], "VeINP_ExeedRIIVRClass_enum[enum]", 26) == 0)
      IVS_Class[9] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVRLaDis_m[m]", 23) == 0)
      IVS_LaDis[9] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVRLgDis_m[m]", 23) == 0)
      IVS_LgDis[9] = i;
    else if (strncmp(columns[i], "VbINP_ExeedRIIVRPresent_flg[flag]", 27) == 0)
      IVS_Present[9] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVRV_mps[mps]", 21) == 0)
      IVS_V[9] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVRLaV_mps[mps]", 23) == 0)
      IVS_LaV[9] = i;
    else if (strncmp(columns[i], "VfINP_ExeedRIIVRHeading_rad[]", 27) == 0)
      IVS_Yaw[9] = i;
    else if (strncmp(columns[i], "VfPASP_ExeedRIIVRAcc_mpss[]", 25) == 0)
      IVS_A[9] = i;

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

int iFL_ObjectId_data[RADAR_OBS][DATA_NUM];
float fFL_ExistProb_data[RADAR_OBS][DATA_NUM];
float fFL_DistX_data[RADAR_OBS][DATA_NUM];
float fFL_DistY_data[RADAR_OBS][DATA_NUM];

int iFR_ObjectId_data[RADAR_OBS][DATA_NUM];
float fFR_ExistProb_data[RADAR_OBS][DATA_NUM];
float fFR_DistX_data[RADAR_OBS][DATA_NUM];
float fFR_DistY_data[RADAR_OBS][DATA_NUM];

int iRL_ObjectId_data[RADAR_OBS][DATA_NUM];
float fRL_ExistProb_data[RADAR_OBS][DATA_NUM];
float fRL_DistX_data[RADAR_OBS][DATA_NUM];
float fRL_DistY_data[RADAR_OBS][DATA_NUM];

int iRR_ObjectId_data[RADAR_OBS][DATA_NUM];
float fRR_ExistProb_data[RADAR_OBS][DATA_NUM];
float fRR_DistX_data[RADAR_OBS][DATA_NUM];
float fRR_DistY_data[RADAR_OBS][DATA_NUM];

void RadarDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame) {
  int Ts = 0;
  int FL_ID[RADAR_OBS] = {0}, FL_EXI_PB[RADAR_OBS] = {0},
      FL_DIS_X[RADAR_OBS] = {0}, FL_DIS_Y[RADAR_OBS] = {0};
  int FR_ID[RADAR_OBS] = {0}, FR_EXI_PB[RADAR_OBS] = {0},
      FR_DIS_X[RADAR_OBS] = {0}, FR_DIS_Y[RADAR_OBS] = {0};
  int RL_ID[RADAR_OBS] = {0}, RL_EXI_PB[RADAR_OBS] = {0},
      RL_DIS_X[RADAR_OBS] = {0}, RL_DIS_Y[RADAR_OBS] = {0};
  int RR_ID[RADAR_OBS] = {0}, RR_EXI_PB[RADAR_OBS] = {0},
      RR_DIS_X[RADAR_OBS] = {0}, RR_DIS_Y[RADAR_OBS] = {0};
  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "timestamps") == 0)
      Ts = i;

    for (int j = 0; j < RADAR_OBS; j++) {
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

    for (int j = 0; j < RADAR_OBS; j++) {
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

    for (int j = 0; j < RADAR_OBS; j++) {
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

    for (int j = 0; j < RADAR_OBS; j++) {
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
  /*   for (int j = 0; j < RADAR_OBS; j++) {
      printf("FL_ID[%d] = %d\t %d, %d, %d\n", j, FL_ID[j], FR_ID[j], RL_ID[j],
             RR_ID[j]);
    } */
  for (int t = 0; t < *totalFrame; t++) {
    time_data[t] = values[Ts][t];

    for (int j = 0; j < RADAR_OBS; j++) {
      if (FL_ID[j] == 0)
        continue;
      iFL_ObjectId_data[j][t] = values[FL_ID[j]][t];
      fFL_ExistProb_data[j][t] = values[FL_EXI_PB[j]][t];
      fFL_DistX_data[j][t] = values[FL_DIS_X[j]][t];
      fFL_DistY_data[j][t] = values[FL_DIS_Y[j]][t];
    }

    for (int j = 0; j < RADAR_OBS; j++) {
      if (FR_ID[j] == 0)
        continue;
      iFR_ObjectId_data[j][t] = values[FR_ID[j]][t];
      fFR_ExistProb_data[j][t] = values[FR_EXI_PB[j]][t];
      fFR_DistX_data[j][t] = values[FR_DIS_X[j]][t];
      fFR_DistY_data[j][t] = values[FR_DIS_Y[j]][t];
    }

    for (int j = 0; j < RADAR_OBS; j++) {
      if (RL_ID[j] == 0)
        continue;
      iRL_ObjectId_data[j][t] = values[RL_ID[j]][t];
      fRL_ExistProb_data[j][t] = values[RL_EXI_PB[j]][t];
      fRL_DistX_data[j][t] = values[RL_DIS_X[j]][t];
      fRL_DistY_data[j][t] = values[RL_DIS_Y[j]][t];
    }

    for (int j = 0; j < RADAR_OBS; j++) {
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

#ifdef MEOBJ_DEMO_TEST
extern float time_data[DATA_NUM];

int iId_data[ME_OBS][DATA_NUM];
int iClass_data[ME_OBS][DATA_NUM];
float fLongDis_data[ME_OBS][DATA_NUM];
float fLatDis_data[ME_OBS][DATA_NUM];
float fLen_data[ME_OBS][DATA_NUM];
float fWid_data[ME_OBS][DATA_NUM];
float fLongSpd_data[ME_OBS][DATA_NUM];
float fHeading_data[ME_OBS][DATA_NUM];

void MeObjDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame) {
  int Ts = 0;
  int ID[ME_OBS] = {0}, CLASS[ME_OBS] = {0}, LONDIS[ME_OBS] = {0},
      LADIS[ME_OBS] = {0}, LON[ME_OBS] = {0}, WID[ME_OBS] = {0},
      SPD[ME_OBS] = {0}, YAW[ME_OBS] = {0};
  int LH_C_0[8] = {0}, LH_C_1[8] = {0}, LA_C_0[8] = {0}, LA_C_1[8] = {0};

  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "timestamps") == 0)
      Ts = i;

    else if (strncmp(columns[i], "VfINP_LH_Line_First_C0_0[m]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C0_0[]", 25) == 0)
      LH_C_0[0] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C0_1[m]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C0_1[]", 25) == 0)
      LH_C_1[0] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C1_0[rad]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C1_0[]", 25) == 0)
      LH_C_0[1] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C1_1[rad]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C1_1[]", 25) == 0)
      LH_C_1[1] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C2_0[]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C2_0[]", 25) == 0)
      LH_C_0[2] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C2_1[]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C2_1[]", 25) == 0)
      LH_C_1[2] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C3_0[]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C3_0[]", 25) == 0)
      LH_C_0[3] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C3_1[]", 24) == 0 ||
             strncmp(columns[i], "VfPASP_LH_Line_First_C3_1[]", 25) == 0)
      LH_C_1[3] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_Start_0[m]", 25) == 0 ||
             strncmp(columns[i], "VfPASP_LH_First_VR_Start_0[]", 26) == 0)
      LH_C_0[6] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_Start_1[m]", 25) == 0 ||
             strncmp(columns[i], "VfPASP_LH_First_VR_Start_1[]", 26) == 0)
      LH_C_1[6] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_End_0[m]", 23) == 0 ||
             strncmp(columns[i], "VfPASP_LH_First_VR_End_0[]", 24) == 0)
      LH_C_0[7] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_End_1[m]", 23) == 0 ||
             strncmp(columns[i], "VfPASP_LH_First_VR_End_1[]", 24) == 0)
      LH_C_1[7] = i;

    else if (strncmp(columns[i], "VfINP_LA_Line_C0_0[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C0_0[]", 19) == 0)
      LA_C_0[0] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C0_1[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C0_1[]", 19) == 0)
      LA_C_1[0] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C1_0[rad]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C1_0[]", 19) == 0)
      LA_C_0[1] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C1_1[rad]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C1_1[]", 19) == 0)
      LA_C_1[1] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C2_0[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C2_0[]", 19) == 0)
      LA_C_0[2] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C2_1[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C2_1[]", 19) == 0)
      LA_C_1[2] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C3_0[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C3_0[]", 19) == 0)
      LA_C_0[3] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C3_1[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_LA_Line_C3_1[]", 19) == 0)
      LA_C_1[3] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_Start_0[m]", 27) == 0 ||
             strncmp(columns[i], "VfPASP_LA_View_Range_Start_0[]", 28) == 0)
      LA_C_0[6] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_Start_1[m]", 27) == 0 ||
             strncmp(columns[i], "VfPASP_LA_View_Range_Start_1[]", 28) == 0)
      LA_C_1[6] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_End_0[m]", 25) == 0 ||
             strncmp(columns[i], "VfPASP_LA_View_Range_End_0[]", 26) == 0)
      LA_C_0[7] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_End_1[m]", 25) == 0 ||
             strncmp(columns[i], "VfPASP_LA_View_Range_End_1[]", 26) == 0)
      LA_C_1[7] = i;

    for (int j = 0; j < ME_OBS; j++) {
      char id_title[70] = "OBJ_ID_";
      char class_title[85] = "OBJ_Object_Class_";
      char long_dis_title[70] = "OBJ_Long_Distance_";
      char lat_dis_title[70] = "OBJ_Lat_Distance_";
      char len_title[70] = "OBJ_Length_";
      char wid_title[70] = "OBJ_Width_";
      char spd_title[70] = "OBJ_Abs_Long_Velocity_";
      char yaw_title[70] = "OBJ_Heading_";

      const int id_len = strlen(id_title);
      const int class_len = strlen(class_title);
      const int long_dis_len = strlen(long_dis_title);
      const int lat_dis_len = strlen(lat_dis_title);
      const int len_len = strlen(len_title);
      const int wid_len = strlen(wid_title);
      const int spd_len = strlen(spd_title);
      const int yaw_len = strlen(yaw_title);

      snprintf(id_title + id_len, sizeof(id_title) - id_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(class_title + class_len, sizeof(class_title) - class_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(long_dis_title + long_dis_len,
               sizeof(long_dis_title) - long_dis_len, "%d_proc_Sensor_disp",
               j + 1);
      snprintf(lat_dis_title + lat_dis_len, sizeof(lat_dis_title) - lat_dis_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(len_title + len_len, sizeof(len_title) - len_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(wid_title + wid_len, sizeof(wid_title) - wid_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(spd_title + spd_len, sizeof(spd_title) - spd_len,
               "%d_proc_Sensor_disp", j + 1);
      snprintf(yaw_title + yaw_len, sizeof(yaw_title) - yaw_len,
               "%d_proc_Sensor_disp", j + 1);

      if (strcmp(columns[i], id_title) == 0) {
        ID[j] = i;
      } else if (strcmp(columns[i], class_title) == 0) {
        CLASS[j] = i;
      } else if (strcmp(columns[i], long_dis_title) == 0) {
        LONDIS[j] = i;
      } else if (strcmp(columns[i], lat_dis_title) == 0) {
        LADIS[j] = i;
      } else if (strcmp(columns[i], len_title) == 0) {
        LON[j] = i;
      } else if (strcmp(columns[i], wid_title) == 0) {
        WID[j] = i;
      } else if (strcmp(columns[i], spd_title) == 0) {
        SPD[j] = i;
      } else if (strcmp(columns[i], yaw_title) == 0) {
        YAW[j] = i;
      }
    }
  }

  *totalFrame = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;
  /*   for (int j = 0; j < ME_OBS; j++) {
      printf("FL_ID[%d] = %d\t %d, %d, %d\n", j, FL_ID[j], FR_ID[j], RL_ID[j],
             RR_ID[j]);
    } */
  for (int t = 0; t < *totalFrame; t++) {
    time_data[t] = values[Ts][t];

    for (int j = 0; j < ME_OBS; j++) {
      if (ID[j] == 0)
        continue;
      iId_data[j][t] = values[ID[j]][t];
      iClass_data[j][t] = values[CLASS[j]][t];
      fLongDis_data[j][t] = values[LONDIS[j]][t];
      fLatDis_data[j][t] = values[LADIS[j]][t];
      fLen_data[j][t] = values[LON[j]][t];
      fWid_data[j][t] = values[WID[j]][t];
      fLongSpd_data[j][t] = values[SPD[j]][t];
      fHeading_data[j][t] = values[YAW[j]][t];
    }
  }
  return;
}
#endif
