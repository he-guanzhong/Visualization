#include "visualization/show_load_log.h"

extern int totalTime;
extern char csvFileName[150];

extern float time_data[DATA_NUM];
extern float egoSpd_data[DATA_NUM];
extern float egoAcc_data[DATA_NUM];
extern float spdLmt_data[DATA_NUM];
extern float alc_path_data[6][DATA_NUM];
extern int accMode_data[DATA_NUM];
extern int alcSts_data[2][DATA_NUM];

extern bool AlcLgtCtrlEnbl_data[DATA_NUM];
extern int truncated_col_data[DATA_NUM];
extern float ctrl_point_data[2][DATA_NUM];
extern float s_points_data[6][DATA_NUM];
extern float v_points_data[6][DATA_NUM];
extern float a_points_data[6][DATA_NUM];
extern float t_points_data[6][DATA_NUM];

extern bool objs_valid_flag_data[10][DATA_NUM];
extern int objs_lane_index_data[10][DATA_NUM];
extern int objs_type_data[10][DATA_NUM];
extern float objs_pos_x_data[10][DATA_NUM];
extern float objs_pos_y_data[10][DATA_NUM];
extern float objs_speed_x_data[10][DATA_NUM];
extern float objs_speed_y_data[10][DATA_NUM];
extern float objs_acc_x_data[10][DATA_NUM];
extern float objs_pos_yaw_data[10][DATA_NUM];

extern int tsr_spd_data[DATA_NUM];
extern bool tsr_spd_warn_data[DATA_NUM];
extern int tsr_tsi_data[2][DATA_NUM];

extern bool tsr_valid_flag_data[3][DATA_NUM];
extern int tsr_type_data[3][DATA_NUM];
extern float tsr_pos_x_data[3][DATA_NUM];
extern float tsr_pos_y_data[3][DATA_NUM];

extern float ll_path_data[8][DATA_NUM];
extern float l_path_data[8][DATA_NUM];
extern float r_path_data[8][DATA_NUM];
extern float rr_path_data[8][DATA_NUM];
extern float ll_path_me_data[8][DATA_NUM];
extern float l_path_me_data[8][DATA_NUM];
extern float r_path_me_data[8][DATA_NUM];
extern float rr_path_me_data[8][DATA_NUM];

inline float readValue(float** values, int col_name, int t) {
  if (values[col_name][t] == 0 && t > 0)
    values[col_name][t] = values[col_name][t - 1];
  return values[col_name][t];
}

void LoadLog() {
  FILE* file = fopen(csvFileName, "r");
  if (!file) {
    totalTime = 0;  // ERROR CODE
    perror("Error opening file");
    return;
  }

  // char line[MAX_LINE_SIZE];
  // char** columns = NULL;
  // float values[MAX_COLUMNS][MAX_VALUES_PER_COLUMN] = {0};
  // int valuesCount[MAX_COLUMNS] = {0};
  int numColumns = 0;

  char* line = (char*)malloc(MAX_LINE_SIZE * sizeof(char));
  if (line == NULL) {
    perror("Failed to allocate memory for line");
    exit(EXIT_FAILURE);
  }

  char** columns = (char**)malloc(MAX_COLUMNS * sizeof(char*));
  if (columns == NULL) {
    // 处理内存分配失败的情况
    perror("Failed to allocate memory for columns");
    free(line);  // 释放已分配的内存
    exit(EXIT_FAILURE);
  }
  // 初始化 columns 数组中的每个指针为 NULL
  for (int i = 0; i < MAX_COLUMNS; ++i) {
    columns[i] = NULL;
  }

  // 动态分配 values 二维数组
  float** values = (float**)malloc(MAX_COLUMNS * sizeof(float*));
  if (values == NULL) {
    // 处理内存分配失败的情况
    perror("Failed to allocate memory for values pointer array");
    for (int i = 0; i < MAX_COLUMNS; ++i) {
      free(columns[i]);  // 释放已分配的 columns 字符串
    }
    free(columns);  // 释放 columns 指针数组
    free(line);     // 释放 line 数组
    exit(EXIT_FAILURE);
  }
  for (int i = 0; i < MAX_COLUMNS; ++i) {
    values[i] = (float*)malloc(MAX_VALUES_PER_COLUMN * sizeof(float));
    if (values[i] == NULL) {
      perror("Failed to allocate memory for a column of values");
      // 释放之前已经分配的内存
      for (int j = 0; j < i; ++j) {
        free(values[j]);
      }
      free(values);  // 释放 values 指针数组
      for (int j = 0; j < MAX_COLUMNS; ++j) {
        free(columns[j]);  // 释放已分配的 columns 字符串
      }
      free(columns);  // 释放 columns 指针数组
      free(line);     // 释放 line 数组
      exit(EXIT_FAILURE);
    }
  }
  // 初始化 values 数组为 0
  for (int i = 0; i < MAX_COLUMNS; ++i) {
    for (int j = 0; j < MAX_VALUES_PER_COLUMN; ++j) {
      values[i][j] = 0;
    }
  }

  // 动态分配 valuesCount 数组
  int* valuesCount = (int*)malloc(MAX_COLUMNS * sizeof(int));
  if (valuesCount == NULL) {
    perror("Failed to allocate memory for valuesCount");
    // 释放之前已经分配的内存
    for (int i = 0; i < MAX_COLUMNS; ++i) {
      free(values[i]);
    }
    free(values);  // 释放 values 指针数组
    for (int i = 0; i < MAX_COLUMNS; ++i) {
      free(columns[i]);  // 释放已分配的 columns 字符串
    }
    free(columns);  // 释放 columns 指针数组
    free(line);     // 释放 line 数组
    exit(EXIT_FAILURE);
  }
  // 初始化 valuesCount 数组为 0
  for (int i = 0; i < MAX_COLUMNS; ++i) {
    valuesCount[i] = 0;
  }

  // 读取第一行作为列
  if (fgets(line, MAX_LINE_SIZE * sizeof(char), file)) {
    char* token = strtok(line, ",");
    while (token) {
      numColumns++;
      columns =
          static_cast<char**>(realloc(columns, numColumns * sizeof(char*)));
      columns[numColumns - 1] = strdup(token);
      token = strtok(NULL, ",");
    }
  }

  // 读取后续行并转换为浮点数
  while (fgets(line, MAX_LINE_SIZE * sizeof(char), file)) {
    char* token = strtok(line, ",");
    int columnIndex = 0;
    while (token) {
      if (strlen(token) > 0) {
        if (columnIndex < numColumns &&
            valuesCount[columnIndex] < MAX_VALUES_PER_COLUMN) {
          values[columnIndex][valuesCount[columnIndex]++] = atof(token);
        }
      }
      token = strtok(NULL, ",");
      columnIndex++;
    }
  }

  // 输出结果
  /*   for (int i = 0; i < numColumns; i++) {
      printf("%s: ", columns[i]);
      for (int j = 0; j < valuesCount[i]; j++) {
        printf("%.2f ", values[i][j]);
      }
      printf("\n");
    } */

  // time, alc path and speed plan input and output results
  int Ts = 0, EGO_V = 0, EGO_A = 0, SPD_LMT = 0, LGT_ENBL = 0, TRUC_CL = 0,
      ACC_MODE = 0;
  int ALC_SIDE = 0, ALC_STS = 0, ALC_C[6] = {0};
  int P_T[7] = {0}, P_S[7] = {0}, P_V[7] = {0}, P_A[7] = {0};

  // Obstacles, In-path VehicleS (IVS)
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  int IVS_Present[10] = {0}, IVS_Class[10] = {0}, IVS_LaDis[10] = {0},
      IVS_LgDis[10] = {0}, IVS_V[10] = {0}, IVS_LaV[10] = {0}, IVS_A[10] = {0},
      IVS_Yaw[10] = {0};

  // TSR signs
  int TSR_Spd = 0, TSR_Warn = 0, TSR_TSI[2] = {0};
  int TSR_ID[3] = {0}, TSR_Type[3] = {0}, TSR_LaDis[3] = {0},
      TSR_LgDis[3] = {0};

  // Lateral control lines.
  // 8 coeffs, [0,5] for c0~c5(to be corrected), 6 = start, 7 = end.
  int L_C[8] = {0}, R_C[8] = {0}, LL_C[8] = {0}, RR_C[8] = {0};

  // ME road lines. Lane Host(LH), Lane Adjacent(LA) Coefficient
  // 8 coeffs, [0,5] for c0~c5(real), 6 = start, 7 = end.
  int LH_C_0[8] = {0}, LH_C_1[8] = {0}, LA_C_0[8] = {0}, LA_C_1[8] = {0};

  // assign col name from measurements
  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "t[s]") == 0)
      Ts = i;
    else if (strcmp(columns[i], "VfPASP_egoSpd_mps[]") == 0 ||
             strcmp(columns[i], "VfACCSTM_VehicleSpeed_mps[]") == 0)
      EGO_V = i;
    else if (strcmp(columns[i], "VfPASP_egoAcc_mpss[]") == 0 ||
             strcmp(columns[i], "VfACCSTM_Algt_mpss[]") == 0)
      EGO_A = i;
    else if (strcmp(columns[i], "VfPASP_spdLmt_kph[]") == 0 ||
             strcmp(columns[i], "VfLGIN_SetSpeedRaw_kph[]") == 0)
      SPD_LMT = i;

    else if (strcmp(columns[i], "VeACCSTM_AccMode_enum[]") == 0)
      ACC_MODE = i;
    else if (strcmp(columns[i], "VePASP_AutoLaneChgSide[]") == 0)
      ALC_SIDE = i;
    else if (strcmp(columns[i], "VePASP_AutoLaneChgSts[]") == 0)
      ALC_STS = i;
    else if (strcmp(columns[i], "VbPASP_AlcLgtCtrlEnbl[]") == 0)
      LGT_ENBL = i;
    else if (strcmp(columns[i], "g_truncated_col[]") == 0)
      TRUC_CL = i;

    // alc path c0-c5
    else if (strcmp(columns[i], "VfPASP_ALC_path_C5[]") == 0)
      ALC_C[5] = i;
    else if (strcmp(columns[i], "VfPASP_ALC_path_C4[]") == 0)
      ALC_C[4] = i;
    else if (strcmp(columns[i], "VfPASP_ALC_path_C3[]") == 0)
      ALC_C[3] = i;
    else if (strcmp(columns[i], "VfPASP_ALC_path_C2[]") == 0)
      ALC_C[2] = i;
    else if (strcmp(columns[i], "VfPASP_ALC_path_C1[]") == 0)
      ALC_C[1] = i;
    else if (strcmp(columns[i], "VfPASP_ALC_path_C0[]") == 0)
      ALC_C[0] = i;

    // speed plan output points
    else if (strcmp(columns[i], "VfPASP_StPoint0_t_s[]") == 0)
      P_T[0] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint0_s_m[]") == 0)
      P_S[0] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint0_v_mps[]") == 0)
      P_V[0] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint0_a_mpss[]") == 0)
      P_A[0] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint1_t_s[]") == 0)
      P_T[1] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint1_s_m[]") == 0)
      P_S[1] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint1_v_mps[]") == 0)
      P_V[1] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint1_a_mpss[]") == 0)
      P_A[1] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint2_t_s[]") == 0)
      P_T[2] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint2_s_m[]") == 0)
      P_S[2] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint2_v_mps[]") == 0)
      P_V[2] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint2_a_mpss[]") == 0)
      P_A[2] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint3_t_s[]") == 0)
      P_T[3] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint3_s_m[]") == 0)
      P_S[3] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint3_v_mps[]") == 0)
      P_V[3] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint3_a_mpss[]") == 0)
      P_A[3] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint4_t_s[]") == 0)
      P_T[4] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint4_s_m[]") == 0)
      P_S[4] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint4_v_mps[]") == 0)
      P_V[4] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint4_a_mpss[]") == 0)
      P_A[4] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint5_t_s[]") == 0)
      P_T[5] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint5_s_m[]") == 0)
      P_S[5] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint5_v_mps[]") == 0)
      P_V[5] = i;
    else if (strcmp(columns[i], "VfPASP_StPoint5_a_mpss[]") == 0)
      P_A[5] = i;
    else if (strcmp(columns[i], "VfPASP_StPointCtrl_t_s[]") == 0)
      P_T[6] = i;
    else if (strcmp(columns[i], "VfPASP_StPointCtrl_s_m[]") == 0)
      P_S[6] = i;
    else if (strcmp(columns[i], "VfPASP_StPointCtrl_v_mps[]") == 0)
      P_V[6] = i;
    else if (strcmp(columns[i], "VfPASP_StPointCtrl_a_mpss[]") == 0)
      P_A[6] = i;

    // obstacle, 0 = IV
    else if (strncmp(columns[i], "VeINP_IVClass[enum]", 14) == 0)
      IVS_Class[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLaDis_m[m]", 16) == 0)
      IVS_LaDis[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLgDis_m[m]", 16) == 0)
      IVS_LgDis[0] = i;
    else if (strncmp(columns[i], "VbINP_IVPresent_flg[flg]", 20) == 0)
      IVS_Present[0] = i;
    else if (strncmp(columns[i], "VfINP_IVV_mps[mps]", 14) == 0)
      IVS_V[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLaSpd_mps[]", 18) == 0)
      IVS_LaV[0] = i;
    else if (strncmp(columns[i], "VfINP_IVAcc_mpss[]", 17) == 0)
      IVS_A[0] = i;
    else if (strncmp(columns[i], "VfINP_IVHeading_rad[]", 19) == 0)
      IVS_Yaw[0] = i;

    // obstacle, 1 = RIV
    else if (strncmp(columns[i], "VeINP_RIVClass[enum]", 15) == 0)
      IVS_Class[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLaDis_m[m]", 17) == 0)
      IVS_LaDis[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLgDis_m[m]", 17) == 0)
      IVS_LgDis[1] = i;
    else if (strncmp(columns[i], "VbINP_RIVPresent_flg[flg]", 21) == 0)
      IVS_Present[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVV_mps[mps]", 15) == 0)
      IVS_V[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLaV_mps[mps]", 17) == 0)
      IVS_LaV[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVHeading_rad[]", 20) == 0)
      IVS_Yaw[1] = i;

    // obstacle, 2 = NIVL
    else if (strncmp(columns[i], "VeINP_NIVLClass_enum[enum]", 21) == 0)
      IVS_Class[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLaDis_m[m]", 18) == 0)
      IVS_LaDis[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLgDis_m[m]", 18) == 0)
      IVS_LgDis[2] = i;
    else if (strncmp(columns[i], "VbINP_NIVLPresent_flg[flg]", 22) == 0)
      IVS_Present[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLV_mps[mps]", 16) == 0)
      IVS_V[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLaV_mps[mps]", 18) == 0)
      IVS_LaV[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLHeading_rad[]", 21) == 0)
      IVS_Yaw[2] = i;

    // obstacle, 6 = NIVR
    else if (strncmp(columns[i], "VeINP_NIVRClass_enum[enum]", 21) == 0)
      IVS_Class[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLaDis_m[m]", 18) == 0)
      IVS_LaDis[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLgDis_m[m]", 18) == 0)
      IVS_LgDis[6] = i;
    else if (strncmp(columns[i], "VbINP_NIVRPresent_flg[flg]", 22) == 0)
      IVS_Present[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRV_mps[mps]", 16) == 0)
      IVS_V[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLaV_mps[mps]", 18) == 0)
      IVS_LaV[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRHeading_rad[]", 21) == 0)
      IVS_Yaw[6] = i;

    // obstacle, 3 = NIIVL
    else if (strncmp(columns[i], "VeINP_NIIVLClass_enum[enum]", 22) == 0)
      IVS_Class[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLaDis_m[m]", 19) == 0)
      IVS_LaDis[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLgDis_m[m]", 19) == 0)
      IVS_LgDis[3] = i;
    else if (strncmp(columns[i], "VbINP_NIIVLPresent_flg[flg]", 23) == 0)
      IVS_Present[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLV_mps[mps]", 17) == 0)
      IVS_V[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLaV_mps[mps]", 19) == 0)
      IVS_LaV[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLHeading_rad[]", 22) == 0)
      IVS_Yaw[3] = i;

    // obstacle, 7 = NIIVR
    else if (strncmp(columns[i], "VeINP_NIIVRClass_enum[enum]", 22) == 0)
      IVS_Class[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLaDis_m[m]", 19) == 0)
      IVS_LaDis[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLgDis_m[m]", 19) == 0)
      IVS_LgDis[7] = i;
    else if (strncmp(columns[i], "VbINP_NIIVRPresent_flg[flg]", 23) == 0)
      IVS_Present[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRV_mps[mps]", 17) == 0)
      IVS_V[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLaV_mps[mps]", 19) == 0)
      IVS_LaV[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRHeading_rad[]", 22) == 0)
      IVS_Yaw[7] = i;

    // obstacle, 4 = RIVL
    else if (strncmp(columns[i], "VeINP_RIVLClass_enum[enum]", 21) == 0)
      IVS_Class[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLaDis_m[m]", 18) == 0)
      IVS_LaDis[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLgDis_m[m]", 18) == 0)
      IVS_LgDis[4] = i;
    else if (strncmp(columns[i], "VbINP_RIVLPresent_flg[flg]", 22) == 0)
      IVS_Present[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLV_mps[mps]", 16) == 0)
      IVS_V[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLaV_mps[mps]", 18) == 0)
      IVS_LaV[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLHeading_rad[]", 21) == 0)
      IVS_Yaw[4] = i;

    // obstacle, 8 = RIVR
    else if (strncmp(columns[i], "VeINP_RIVRClass_enum[enum]", 21) == 0)
      IVS_Class[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLaDis_m[m]", 18) == 0)
      IVS_LaDis[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLgDis_m[m]", 18) == 0)
      IVS_LgDis[8] = i;
    else if (strncmp(columns[i], "VbINP_RIVRPresent_flg[flag]", 22) == 0)
      IVS_Present[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRV_mps[mps]", 16) == 0)
      IVS_V[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLaV_mps[mps]", 18) == 0)
      IVS_LaV[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRHeading_rad[]", 21) == 0)
      IVS_Yaw[8] = i;

    // obstacle, 5 = RIIVL
    else if (strncmp(columns[i], "VeINP_RIIVLClass_enum[enum]", 22) == 0)
      IVS_Class[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLaDis_m[m]", 19) == 0)
      IVS_LaDis[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLgDis_m[m]", 19) == 0)
      IVS_LgDis[5] = i;
    else if (strncmp(columns[i], "VbINP_RIIVLPresent_flg[flag]", 23) == 0)
      IVS_Present[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLV_mps[mps]", 17) == 0)
      IVS_V[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLaV_mps[mps]", 19) == 0)
      IVS_LaV[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLHeading_rad[]", 22) == 0)
      IVS_Yaw[5] = i;

    // obstacle, 9 = RIIVR
    else if (strncmp(columns[i], "VeINP_RIIVRClass_enum[enum]", 22) == 0)
      IVS_Class[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLaDis_m[m]", 19) == 0)
      IVS_LaDis[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLgDis_m[m]", 19) == 0)
      IVS_LgDis[9] = i;
    else if (strncmp(columns[i], "VbINP_RIIVRPresent_flg[flag]", 23) == 0)
      IVS_Present[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRV_mps[mps]", 17) == 0)
      IVS_V[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLaV_mps[mps]", 19) == 0)
      IVS_LaV[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRHeading_rad[]", 22) == 0)
      IVS_Yaw[9] = i;

    // lateral control line coefficent
    else if (strcmp(columns[i], "VfLSSIN_l_LaneMkrOffsLe[]") == 0)
      L_C[0] = i;
    else if (strcmp(columns[i], "VfLSSIN_Ag_LaneMkrHdgAgLe[]") == 0)
      L_C[1] = i;
    else if (strcmp(columns[i], "VfLSSIN_Crvt_LaneMkrCrvtLe[]") == 0)
      L_C[2] = i;
    else if (strcmp(columns[i], "VfLSSIN_dCrvt_LaneMkrCrvtRateLe[]") == 0)
      L_C[3] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      L_C[4] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      L_C[5] = i;
    else if (strcmp(columns[i], "VfLSSIN_l_StrtRngLe[]") == 0)
      L_C[6] = i;
    else if (strcmp(columns[i], "VfLSSIN_l_ViewRngLe[]") == 0)
      L_C[7] = i;
    else if (strcmp(columns[i], "VfLSSIN_l_LaneMkrOffsRi[]") == 0)
      R_C[0] = i;
    else if (strcmp(columns[i], "VfLSSIN_Ag_LaneMkrHdgAgRi[]") == 0)
      R_C[1] = i;
    else if (strcmp(columns[i], "VfLSSIN_Crvt_LaneMkrCrvtRi[]") == 0)
      R_C[2] = i;
    else if (strcmp(columns[i], "VfLSSIN_dCrvt_LaneMkrCrvtRateRi[]") == 0)
      R_C[3] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      R_C[4] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      R_C[5] = i;
    else if (strcmp(columns[i], "VfLSSIN_l_StrtRngRi[]") == 0)
      R_C[6] = i;
    else if (strcmp(columns[i], "VfLSSIN_l_ViewRngRi[]") == 0)
      R_C[7] = i;

    else if (strcmp(columns[i], "VFLSSIN7_l_LaneMkrOffsNxtLe[]") == 0)
      LL_C[0] = i;
    else if (strcmp(columns[i], "VFLSSIN7_Ag_LaneMkrHdgAgNxtLe[]") == 0)
      LL_C[1] = i;
    else if (strcmp(columns[i], "VFLSSIN7_Crvt_LaneMkrCrvtNxtLe[]") == 0)
      LL_C[2] = i;
    else if (strcmp(columns[i], "VFLSSIN7_dCrvt_LaneMkrCrvtRateNxtLe[]") == 0)
      LL_C[3] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      LL_C[4] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      LL_C[5] = i;
    else if (strcmp(columns[i], "VFLSSIN7_l_StrtRngNxtLe[]") == 0)
      LL_C[6] = i;
    else if (strcmp(columns[i], "VfLSSIN7_l_ViewRngNxtLe[]") == 0)
      LL_C[7] = i;

    else if (strcmp(columns[i], "VFLSSIN7_l_LaneMkrOffsNxtRi[]") == 0)
      RR_C[0] = i;
    else if (strcmp(columns[i], "VFLSSIN7_Ag_LaneMkrHdgAgNxtRi[]") == 0)
      RR_C[1] = i;
    else if (strcmp(columns[i], "VFLSSIN7_Crvt_LaneMkrCrvtNxtRi[]") == 0)
      RR_C[2] = i;
    else if (strcmp(columns[i], "VFLSSIN7_dCrvt_LaneMkrCrvtRateNxtRi[]") == 0)
      RR_C[3] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      RR_C[4] = i;
    else if (strcmp(columns[i], "NaN") == 0)
      RR_C[5] = i;
    else if (strcmp(columns[i], "VFLSSIN7_l_StrtRngNxtRi[]") == 0)
      RR_C[6] = i;
    else if (strcmp(columns[i], "VfLSSIN7_l_ViewRngNxtRi[]") == 0)
      RR_C[7] = i;

    //  Mobile Eye original lines
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C0_0[m]", 24) == 0)
      LH_C_0[0] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C0_1[m]", 24) == 0)
      LH_C_1[0] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C1_0[rad]", 24) == 0)
      LH_C_0[1] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C1_1[rad]", 24) == 0)
      LH_C_1[1] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C2_0[]", 24) == 0)
      LH_C_0[2] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C2_1[]", 24) == 0)
      LH_C_1[2] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C3_0[]", 24) == 0)
      LH_C_0[3] = i;
    else if (strncmp(columns[i], "VfINP_LH_Line_First_C3_1[]", 24) == 0)
      LH_C_1[3] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_Start_0[m]", 25) == 0)
      LH_C_0[6] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_Start_1[m]", 25) == 0)
      LH_C_1[6] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_End_0[m]", 23) == 0)
      LH_C_0[7] = i;
    else if (strncmp(columns[i], "VfINP_LH_First_VR_End_1[m]", 23) == 0)
      LH_C_1[7] = i;

    else if (strncmp(columns[i], "VfINP_LA_Line_C0_0[m]", 18) == 0)
      LA_C_0[0] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C0_1[m]", 18) == 0)
      LA_C_1[0] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C1_0[rad]", 18) == 0)
      LA_C_0[1] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C1_1[rad]", 18) == 0)
      LA_C_1[1] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C2_0[]", 18) == 0)
      LA_C_0[2] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C2_1[]", 18) == 0)
      LA_C_1[2] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C3_0[]", 18) == 0)
      LA_C_0[3] = i;
    else if (strncmp(columns[i], "VfINP_LA_Line_C3_1[]", 18) == 0)
      LA_C_1[3] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_Start_0[m]", 27) == 0)
      LA_C_0[6] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_Start_1[m]", 27) == 0)
      LA_C_1[6] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_End_0[m]", 25) == 0)
      LA_C_0[7] = i;
    else if (strncmp(columns[i], "VfINP_LA_View_Range_End_1[m]", 25) == 0)
      LA_C_1[7] = i;

    // TSR status
    else if (strcmp(columns[i], "VfLGIN_TsrTrgtSpdReq_kph[]") == 0)
      TSR_Spd = i;
    else if (strcmp(columns[i], "VeLGIN_ISLCOverSpeedAlarmSts[]") == 0)
      TSR_Warn = i;
    else if (strcmp(columns[i], "VeLGIN_TSI_SignShortStayMisc_1[]") == 0)
      TSR_TSI[0] = i;
    else if (strcmp(columns[i], "VeLGIN_TSI_SignShortStayMisc_2[]") == 0)
      TSR_TSI[1] = i;

    // TSR, original signals 3 key signs are enough
    else if (strcmp(columns[i], "VeINP_TSRID0Proc[]") == 0)
      TSR_ID[0] = i;
    else if (strcmp(columns[i], "VuINP_TSRSignName0Proc[]") == 0)
      TSR_Type[0] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLatDist0Proc_m[]") == 0)
      TSR_LaDis[0] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLgDist0Proc_m[]") == 0)
      TSR_LgDis[0] = i;
    else if (strcmp(columns[i], "VeINP_TSRID1Proc[]") == 0)
      TSR_ID[1] = i;
    else if (strcmp(columns[i], "VuINP_TSRSignName1Proc[]") == 0)
      TSR_Type[1] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLatDist1Proc_m[]") == 0)
      TSR_LaDis[1] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLgDist1Proc_m[]") == 0)
      TSR_LgDis[1] = i;
    else if (strcmp(columns[i], "VeINP_TSRID2Proc[]") == 0)
      TSR_ID[2] = i;
    else if (strcmp(columns[i], "VuINP_TSRSignName2Proc[]") == 0)
      TSR_Type[2] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLatDist2Proc_m[]") == 0)
      TSR_LaDis[2] = i;
    else if (strcmp(columns[i], "VfINP_TSRSignLgDist2Proc_m[]") == 0)
      TSR_LgDis[2] = i;
  }

  // log total time. ATTENTION: NaN strings occupy last 8 rows of csv
  totalTime = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;

  // load data to local variables
  for (int t = 0; t < totalTime; t++) {
    time_data[t] = values[Ts][t];
    //  inner spd lmt unit: m/s, record spd lmt unit:kph
    if (EGO_V != 0) {
      egoSpd_data[t] = values[EGO_V][t];
      egoAcc_data[t] = values[EGO_A][t];
      spdLmt_data[t] = values[SPD_LMT][t] / 3.6f;
      accMode_data[t] = values[ACC_MODE][t];
    }
    AlcLgtCtrlEnbl_data[t] = LGT_ENBL ? values[LGT_ENBL][t] : 0;
    truncated_col_data[t] = TRUC_CL ? values[TRUC_CL][t] : 0;
    // alc_sts[0] for alc side: 0-0ff, 1-left, 2-right
    // alc_sts[1] for alc sts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving,
    // 4-in target line, 5-finished,6-Back to Ego, 8-takeover, 9-popMsgReq
    alcSts_data[0][t] = ALC_SIDE ? values[ALC_SIDE][t] : 0;
    alcSts_data[1][t] = ALC_STS ? values[ALC_STS][t] : 0;

    // alc path and speed plan points
    for (int k = 0; k <= 5; k++) {
      if (ALC_C[k] == 0)
        continue;
      alc_path_data[k][t] = values[ALC_C[k]][t];
      s_points_data[k][t] = values[P_S[k]][t];
      v_points_data[k][t] = values[P_V[k]][t];
      a_points_data[k][t] = values[P_A[k]][t];
      t_points_data[k][t] = values[P_T[k]][t];
    }
    ctrl_point_data[0][t] = P_A[6] ? values[P_T[6]][t] : 0;
    ctrl_point_data[1][t] = P_A[6] ? values[P_A[6]][t] : 0;

    // obstacles, BTL original: lateral distance/speed direction opposite.
    // longi distance needs compensation to transfer pos centre to centre
    // for front obs, ego front bumper center to obs rear bumper.
    // for rear obs, ego front bumper center to rear car center.
    float pos_x_compensation = 0;
    for (int k = 0; k < 10; k++) {
      if (IVS_Present[k] == 0)
        continue;

      if (0 == k || 2 == k || 3 == k || 6 == k || 7 == k) {
        pos_x_compensation =
            2.5f + (objs_type_data[k][t] == 2 ? 7.6f : 5.0f) / 2.0f;
      } else {
        pos_x_compensation = 2.5f;
      }
      objs_valid_flag_data[k][t] = values[IVS_Present[k]][t];
      objs_type_data[k][t] = values[IVS_Class[k]][t];
      objs_pos_x_data[k][t] = values[IVS_LgDis[k]][t] + pos_x_compensation;
      objs_pos_y_data[k][t] = values[IVS_LaDis[k]][t] * -1;
      objs_speed_x_data[k][t] = values[IVS_V[k]][t];
      objs_speed_y_data[k][t] = IVS_LaV[k] ? values[IVS_LaV[k]][t] * -1 : 0;
      objs_acc_x_data[k][t] = IVS_A[k] ? values[IVS_A[k]][t] : 0;
      objs_pos_yaw_data[k][t] = IVS_Yaw[k] ? values[IVS_Yaw[k]][t] * -1 : 0;

      if (k <= 1)
        objs_lane_index_data[k][t] = 3;
      else if (k <= 5)
        objs_lane_index_data[k][t] = 2;
      else
        objs_lane_index_data[k][t] = 4;
    }

    // lateral processed lines. c4 and c5 ignored, for no input
    // c2 = crvt * 0.5, c3 = d_crvt * 1/6
    for (int k = 0; k < 8; k++) {
      if (L_C[k] == 0)
        continue;
      float correct_fac = (k == 2 ? 0.5f : (k == 3 ? 0.166667f : 1.0f));
      if (k <= 3 || k >= 6) {
        l_path_data[k][t] = values[L_C[k]][t] * correct_fac;
        r_path_data[k][t] = values[R_C[k]][t] * correct_fac;
        ll_path_data[k][t] = values[LL_C[k]][t] * correct_fac;
        rr_path_data[k][t] = values[RR_C[k]][t] * correct_fac;
      }
    }

    // ME original lines. c0~c3 opposite, c4 and c5 ignored, for no input
    for (int k = 0; k < 8; k++) {
      if (LH_C_0[k] == 0)
        continue;
      if (k <= 3) {
        l_path_me_data[k][t] = values[LH_C_0[k]][t] * -1;
        r_path_me_data[k][t] = values[LH_C_1[k]][t] * -1;
        ll_path_me_data[k][t] = values[LA_C_0[k]][t] * -1;
        rr_path_me_data[k][t] = values[LA_C_1[k]][t] * -1;
      } else if (k >= 6) {
        l_path_me_data[k][t] = values[LH_C_0[k]][t];
        r_path_me_data[k][t] = values[LH_C_1[k]][t];
        ll_path_me_data[k][t] = values[LA_C_0[k]][t];
        rr_path_me_data[k][t] = values[LA_C_1[k]][t];
      }
    }

    // TSR status
    if (TSR_Spd != 0) {
      tsr_spd_data[t] = values[TSR_Spd][t];
      tsr_spd_warn_data[t] = TSR_Warn == 0 ? 0 : values[TSR_Warn][t];
      tsr_tsi_data[0][t] = values[TSR_TSI[0]][t];
      tsr_tsi_data[1][t] = values[TSR_TSI[1]][t];
    }

    // TSR, lateral distance opposite
    for (int k = 0; k < 3; k++) {
      if (TSR_ID[k] == 0)
        continue;
      tsr_valid_flag_data[k][t] = values[TSR_ID[k]][t];
      tsr_type_data[k][t] = values[TSR_Type[k]][t];
      tsr_pos_x_data[k][t] = values[TSR_LgDis[k]][t];
      tsr_pos_y_data[k][t] = values[TSR_LaDis[k]][t] * -1;
    }
  }

  // 释放已分配的内存
  for (int i = 0; i < numColumns; ++i) {
    free(values[i]);
  }
  free(values);
  for (int i = 0; i < numColumns; ++i) {
    free(columns[i]);
  }
  free(columns);
  free(line);
  free(valuesCount);

  fclose(file);

  return;
}
