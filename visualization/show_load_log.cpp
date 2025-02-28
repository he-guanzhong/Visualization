#include "visualization/show_load_log.h"

#define RAW_LINE 0
#define READ_EXEED_OBS 0

static inline float readValue(float** values, int col_name, int t) {
  if (values[col_name][t] == 0 && t > 0)
    values[col_name][t] = values[col_name][t - 1];
  return values[col_name][t];
}

void LoadLog(const char* const csvFileName, int* totalFrame) {
  // 检查文件是否在黑名单
  const char* device_blacklist[2] = {"/dev/tty", "/dev/null"};
  for (int i = 0; i < 2 && device_blacklist[i] != NULL; ++i) {
    if (strcmp(csvFileName, device_blacklist[i]) == 0) {
      return;
    }
  }

  FILE* file = fopen(csvFileName, "r");
  if (!file) {
    *totalFrame = 0;  // ERROR CODE
    perror("Error opening file");
    return;
  }

  int numColumns = 0;

  char* line = (char*)malloc(MAX_LINE_SIZE * sizeof(char));
  if (line == NULL) {
    perror("Failed to allocate memory for line");
    (void)fclose(file);
    exit(EXIT_FAILURE);
  }

  char** columns = (char**)malloc(MAX_COLUMNS * sizeof(char*));
  if (columns == NULL) {
    // 处理内存分配失败的情况
    perror("Failed to allocate memory for columns");
    free(line);  // 释放已分配的内存
    (void)fclose(file);
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
    (void)fclose(file);
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
      (void)fclose(file);
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
    (void)fclose(file);
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
      if (++numColumns > MAX_COLUMNS)
        break;
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
    // char* tokenEndPtr;
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

#ifdef RADAR_DEMO_TEST
  RadarDataParsing(values, numColumns, columns, valuesCount, totalFrame);
#endif
#ifdef AGSM_DEMO_TEST
  AgsmDataParsing(values, numColumns, columns, valuesCount, totalFrame);
#endif
#ifdef REM_DEMO_TEST
  RemDataParsing(totalFrame);
#else
  SpdPlanDataParsing(values, numColumns, columns, valuesCount, totalFrame);
#endif

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

  (void)fclose(file);

  return;
}

void SpdPlanDataParsing(float** values,
                        const int numColumns,
                        char** columns,
                        const int* valuesCount,
                        int* totalFrame) {
  // time, alc path and speed plan input and output results
  int Ts = 0, EGO_V = 0, EGO_A = 0, SPD_LMT = 0, ENBL_STS = 0, TRUC_CL = 0,
      ACC_MODE = 0, TAU_GAP = 0, ACC_DREF = 0, ACC_CURV = 0;
  int IN_SPDLMT = 0, SPC_FLG = 0, SCE_FLG = 0, MAX_DCL = 0;
  int EGO_PATH[9] = {0};
  int ALC_SIDE = 0, ALC_STS = 0, ALC_LBT = 0, ALC_RBT = 0, NOA_STS = 0,
      ALC_RMP = 0, ALC_RMP_D = 0, ALC_EXT_D = 0;
  int ALC_GAP = 0, ALC_TARS = 0, ALC_TARV = 0, ALC_ST[6] = {0};
  int ALC_C[8] = {0};
  int P_T[7] = {0}, P_S[7] = {0}, P_V[7] = {0}, P_A[7] = {0};

  // Obstacles, In-path VehicleS (IVS)
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  int IVS_ID[10] = {0}, IVS_Present[10] = {0}, IVS_Class[10] = {0},
      IVS_LaDis[10] = {0}, IVS_LgDis[10] = {0}, IVS_V[10] = {0},
      IVS_LaV[10] = {0}, IVS_A[10] = {0}, IVS_Yaw[10] = {0}, IVS_CUT[10] = {0},
      IVS_LaVF[10] = {0};

  // TSR signs
  int TSR_Spd = 0, TSR_Warn = 0, TSR_TSI[2] = {0};
  int TSR_ID[3] = {0}, TSR_Type[3] = {0}, TSR_LaDis[3] = {0},
      TSR_LgDis[3] = {0};

  // ME road lines. Lane Host(LH), Lane Adjacent(LA) Coefficient
  // 8 coeffs, [0,5] for c0~c5(real), 6 = start, 7 = end.
  int LH_C_0[8] = {0}, LH_C_1[8] = {0}, LA_C_0[8] = {0}, LA_C_1[8] = {0};

  // REM info, DP lines
  int EGO_DP[4] = {0}, TAR_DP[4] = {0}, MRG_DIS = 0, MRG_DIR = 0, MRG_ID = 0;

  // assign col name from measurements
  for (int i = 0; i < numColumns; i++) {
    if (strcmp(columns[i], "t[s]") == 0 ||
        strcmp(columns[i], "timestamps") == 0)
      Ts = i;
    else if (strncmp(columns[i], "VfACCSTM_VehicleSpeed_mps[]", 25) == 0 ||
             strncmp(columns[i], "VfPASP_egoSpd_mps[]", 17) == 0)
      EGO_V = i;
    else if (strncmp(columns[i], "VfACCSTM_Algt_mpss[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_egoAcc_mpss[]", 18) == 0)
      EGO_A = i;
    else if (strncmp(columns[i], "VfLGIN_SetSpeedRaw_kph[]", 22) == 0 ||
             strncmp(columns[i], "VfPASP_spdLmt_kph[]", 17) == 0)
      SPD_LMT = i;
    else if (strncmp(columns[i], "VeACCSTM_AccMode_enum[]", 21) == 0 ||
             strncmp(columns[i], "VePASP_ACCMode[]", 14) == 0)
      ACC_MODE = i;
    else if (strncmp(columns[i], "VfACCTRL_TimeGapIndex[]", 21) == 0 ||
             strncmp(columns[i], "VePASP_TauGapSet[]", 16) == 0)
      TAU_GAP = i;
    else if (strncmp(columns[i], "VfPASP_ACC_Dref_m[]", 17) == 0)
      ACC_DREF = i;
    else if (strncmp(columns[i], "VfPASP_ACC_SetSpeedByCurve_kph[]", 30) == 0)
      ACC_CURV = i;

    else if (strncmp(columns[i], "VePASP_AutoLaneChgSide[]", 22) == 0)
      ALC_SIDE = i;
    else if (strncmp(columns[i], "VePASP_AutoLaneChgSts[]", 21) == 0)
      ALC_STS = i;
    else if (strncmp(columns[i], "VePASP_LeftBoundaryType[]", 23) == 0)
      ALC_LBT = i;
    else if (strncmp(columns[i], "VePASP_RightBoundaryType[]", 24) == 0)
      ALC_RBT = i;
    else if (strncmp(columns[i], "VePASP_NOAStatus[]", 16) == 0)
      NOA_STS = i;
    else if (strncmp(columns[i], "VePASP_NaviPilotIsRamp[]", 22) == 0)
      ALC_RMP = i;
    else if (strncmp(columns[i], "VuPASP_NaviPilot1stRampOnDis_m[]", 30) == 0)
      ALC_RMP_D = i;
    else if (strncmp(columns[i], "VuPASP_NaviPilot1stExitDis_m[]", 28) == 0)
      ALC_EXT_D = i;

    else if (strncmp(columns[i], "VePASP_AlcGapIndex[]", 18) == 0)
      ALC_GAP = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapTarS_m[]", 19) == 0)
      ALC_TARS = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapTarV_mps[]", 21) == 0)
      ALC_TARV = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C0[]", 18) == 0)
      ALC_ST[0] = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C1[]", 18) == 0)
      ALC_ST[1] = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C2[]", 18) == 0)
      ALC_ST[2] = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C3[]", 18) == 0)
      ALC_ST[3] = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C4[]", 18) == 0)
      ALC_ST[4] = i;
    else if (strncmp(columns[i], "VfPASP_AlcGapSt_C5[]", 18) == 0)
      ALC_ST[5] = i;

    else if (strncmp(columns[i], "VePASP_SpdPlanEnblSts[]", 21) == 0)
      ENBL_STS = i;
    else if (strncmp(columns[i], "VfPASP_InnerSetSpd_kph[]", 22) == 0)
      IN_SPDLMT = i;
    else if (strncmp(columns[i], "VePASP_SpecialCaseFlg[]", 21) == 0)
      SPC_FLG = i;
    else if (strncmp(columns[i], "VePASP_ScenarioFlg[]", 18) == 0)
      SCE_FLG = i;
    else if (strncmp(columns[i], "VfPASP_maxDecel_mpss[]", 20) == 0)
      MAX_DCL = i;
    else if (strncmp(columns[i], "g_truncated_col[]", 15) == 0)
      TRUC_CL = i;

    // alc path c0-c5
    else if (strncmp(columns[i], "VfPASP_ALC_path_C5[]", 18) == 0)
      ALC_C[5] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_C4[]", 18) == 0)
      ALC_C[4] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_C3[]", 18) == 0)
      ALC_C[3] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_C2[]", 18) == 0)
      ALC_C[2] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_C1[]", 18) == 0)
      ALC_C[1] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_C0[]", 18) == 0)
      ALC_C[0] = i;
    else if (strncmp(columns[i], "VfPASP_ALC_path_ViewRng[]", 23) == 0)
      ALC_C[7] = i;

    // ego path [9]: c0, c1, c2, c31, c32, c33, len1, len2, len3
    else if (strncmp(columns[i], "VfPASP_ego_path_C0[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_EnvModelPath_C0[]", 22) == 0)
      EGO_PATH[0] = i;
    else if (strncmp(columns[i], "VfPASP_ego_path_C1[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_EnvModelPath_C1[]", 22) == 0)
      EGO_PATH[1] = i;
    else if (strncmp(columns[i], "VfPASP_ego_path_C2[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_EnvModelPath_C2[]", 22) == 0)
      EGO_PATH[2] = i;
    else if (strncmp(columns[i], "VfPASP_ego_path_C3[]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_EnvModelPath_C3_1[]", 24) == 0)
      EGO_PATH[3] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModelPath_C3_2[]", 24) == 0)
      EGO_PATH[4] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModelPath_C3_3[]", 24) == 0)
      EGO_PATH[5] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModelPath_Length1[]", 27) == 0)
      EGO_PATH[6] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModelPath_Length2[]", 27) == 0)
      EGO_PATH[7] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModelPath_Length3[]", 27) == 0)
      EGO_PATH[8] = i;

    // speed plan output points
    else if (strncmp(columns[i], "VfPASP_StPoint0_t_s[]", 19) == 0)
      P_T[0] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint0_s_m[]", 19) == 0)
      P_S[0] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint0_v_mps[]", 21) == 0)
      P_V[0] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint0_a_mpss[]", 22) == 0)
      P_A[0] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint1_t_s[]", 19) == 0)
      P_T[1] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint1_s_m[]", 19) == 0)
      P_S[1] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint1_v_mps[]", 21) == 0)
      P_V[1] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint1_a_mpss[]", 22) == 0)
      P_A[1] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint2_t_s[]", 19) == 0)
      P_T[2] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint2_s_m[]", 19) == 0)
      P_S[2] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint2_v_mps[]", 21) == 0)
      P_V[2] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint2_a_mpss[]", 22) == 0)
      P_A[2] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint3_t_s[]", 19) == 0)
      P_T[3] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint3_s_m[]", 19) == 0)
      P_S[3] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint3_v_mps[]", 21) == 0)
      P_V[3] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint3_a_mpss[]", 22) == 0)
      P_A[3] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint4_t_s[]", 19) == 0)
      P_T[4] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint4_s_m[]", 19) == 0)
      P_S[4] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint4_v_mps[]", 21) == 0)
      P_V[4] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint4_a_mpss[]", 22) == 0)
      P_A[4] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint5_t_s[]", 19) == 0)
      P_T[5] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint5_s_m[]", 19) == 0)
      P_S[5] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint5_v_mps[]", 21) == 0)
      P_V[5] = i;
    else if (strncmp(columns[i], "VfPASP_StPoint5_a_mpss[]", 22) == 0)
      P_A[5] = i;
    else if (strncmp(columns[i], "VfPASP_StPointCtrl0_t_s[]", 23) == 0)
      P_T[6] = i;
    else if (strncmp(columns[i], "VfPASP_StPointCtrl0_s_m[]", 23) == 0)
      P_S[6] = i;
    else if (strncmp(columns[i], "VfPASP_StPointCtrl0_v_mps[]", 25) == 0)
      P_V[6] = i;
    else if (strncmp(columns[i], "VfPASP_StPointCtrl0_a_mpss[]", 26) == 0)
      P_A[6] = i;

#if READ_EXEED_OBS == 0
    // obstacle, 0 = IV
    else if (strncmp(columns[i], "VeINP_IVID[]", 10) == 0 ||
             strncmp(columns[i], "VePASP_IVID[]", 11) == 0)
      IVS_ID[0] = i;
    else if (strncmp(columns[i], "VeINP_IVClass[enum]", 13) == 0 ||
             strncmp(columns[i], "VePASP_IVClass[]", 14) == 0)
      IVS_Class[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLaDis_m[m]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_IVLaDis_m[m]", 16) == 0)
      IVS_LaDis[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLgDis_m[m]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_IVLgDis_m[]", 16) == 0)
      IVS_LgDis[0] = i;
    else if (strncmp(columns[i], "VbINP_IVPresent_flg[flg]", 19) == 0 ||
             strncmp(columns[i], "VbPASP_IVPresent_flg[]", 20) == 0)
      IVS_Present[0] = i;
    else if (strncmp(columns[i], "VfINP_IVV_mps[mps]", 13) == 0 ||
             strncmp(columns[i], "VfPASP_IVV_mps[]", 14) == 0)
      IVS_V[0] = i;
    else if (strncmp(columns[i], "VfINP_IVLaSpd_mps[]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_IVLaSpd_mps[]", 18) == 0)
      IVS_LaV[0] = i;
    else if (strncmp(columns[i], "VfINP_IVHeading_rad[]", 19) == 0 ||
             strncmp(columns[i], "VfPASP_IVHeading_rad[]", 20) == 0)
      IVS_Yaw[0] = i;
    else if (strncmp(columns[i], "VfINP_IVAcc_mpss[]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_IVAcc_mpss[]", 17) == 0)
      IVS_A[0] = i;

    // obstacle, 1 = RIV
    else if (strncmp(columns[i], "VeINP_RIVID[]", 11) == 0 ||
             strncmp(columns[i], "VePASP_RIVID[]", 12) == 0)
      IVS_ID[1] = i;
    else if (strncmp(columns[i], "VeINP_RIVClass_enum[enum]", 19) == 0 ||
             strncmp(columns[i], "VePASP_RIVClass_enum[]", 20) == 0)
      IVS_Class[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLaDis_m[m]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLaDis_m[]", 17) == 0)
      IVS_LaDis[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLgDis_m[m]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLgDis_m[]", 17) == 0)
      IVS_LgDis[1] = i;
    else if (strncmp(columns[i], "VbINP_RIVPresent_flg[flg]", 20) == 0 ||
             strncmp(columns[i], "VbPASP_RIVPresent_flg[]", 21) == 0)
      IVS_Present[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVV_mps[mps]", 14) == 0 ||
             strncmp(columns[i], "VfPASP_RIVV_mps[]", 15) == 0)
      IVS_V[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVLaV_mps[mps]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLaV_mps[]", 17) == 0)
      IVS_LaV[1] = i;
    else if (strncmp(columns[i], "VfINP_RIVHeading_rad[]", 20) == 0 ||
             strncmp(columns[i], "VfPASP_RIVHeading_rad[]", 21) == 0)
      IVS_Yaw[1] = i;
    else if (strncmp(columns[i], "VfPASP_RIVAcc_mpss[]", 18) == 0)
      IVS_A[1] = i;

    // obstacle, 2 = NIVL
    else if (strncmp(columns[i], "VeINP_NIVLID[]", 12) == 0 ||
             strncmp(columns[i], "VePASP_NIVLID[]", 13) == 0)
      IVS_ID[2] = i;
    else if (strncmp(columns[i], "VeINP_NIVLClass_enum[enum]", 20) == 0 ||
             strncmp(columns[i], "VePASP_NIVLClass_enum[]", 21) == 0)
      IVS_Class[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLaDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVLLaDis_m[]", 18) == 0)
      IVS_LaDis[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLgDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVLLgDis_m[]", 18) == 0)
      IVS_LgDis[2] = i;
    else if (strncmp(columns[i], "VbINP_NIVLPresent_flg[flg]", 21) == 0 ||
             strncmp(columns[i], "VbPASP_NIVLPresent_flg[]", 22) == 0)
      IVS_Present[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLV_mps[mps]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_NIVLV_mps[]", 16) == 0)
      IVS_V[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLLaV_mps[mps]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVLLaV_mps[]", 18) == 0)
      IVS_LaV[2] = i;
    else if (strncmp(columns[i], "VfINP_NIVLHeading_rad[]", 21) == 0 ||
             strncmp(columns[i], "VfPASP_NIVLHeading_rad[]", 22) == 0)
      IVS_Yaw[2] = i;
    else if (strncmp(columns[i], "VfPASP_NIVLAcc_mpss[]", 19) == 0)
      IVS_A[2] = i;
    else if (strncmp(columns[i], "VePASP_NIVLCutIn[]", 16) == 0)
      IVS_CUT[2] = i;

    // obstacle, 6 = NIVR
    else if (strncmp(columns[i], "VeINP_NIVRID[]", 12) == 0 ||
             strncmp(columns[i], "VePASP_NIVRID[]", 13) == 0)
      IVS_ID[6] = i;
    else if (strncmp(columns[i], "VeINP_NIVRClass_enum[enum]", 20) == 0 ||
             strncmp(columns[i], "VePASP_NIVRClass_enum[]", 21) == 0)
      IVS_Class[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLaDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVRLaDis_m[]", 18) == 0)
      IVS_LaDis[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLgDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVRLgDis_m[]", 18) == 0)
      IVS_LgDis[6] = i;
    else if (strncmp(columns[i], "VbINP_NIVRPresent_flg[flg]", 21) == 0 ||
             strncmp(columns[i], "VbPASP_NIVRPresent_flg[]", 22) == 0)
      IVS_Present[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRV_mps[mps]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_NIVRV_mps[]", 16) == 0)
      IVS_V[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRLaV_mps[mps]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_NIVRLaV_mps[]", 18) == 0)
      IVS_LaV[6] = i;
    else if (strncmp(columns[i], "VfINP_NIVRHeading_rad[]", 21) == 0 ||
             strncmp(columns[i], "VfPASP_NIVRHeading_rad[]", 22) == 0)
      IVS_Yaw[6] = i;
    else if (strncmp(columns[i], "VfPASP_NIVRAcc_mpss[]", 19) == 0)
      IVS_A[6] = i;
    else if (strncmp(columns[i], "VePASP_NIVRCutIn[]", 16) == 0)
      IVS_CUT[6] = i;

    // obstacle, 3 = NIIVL
    else if (strncmp(columns[i], "VeINP_NIIVLID[]", 13) == 0 ||
             strncmp(columns[i], "VePASP_NIIVLID[]", 14) == 0)
      IVS_ID[3] = i;
    else if (strncmp(columns[i], "VeINP_NIIVLClass_enum[enum]", 21) == 0 ||
             strncmp(columns[i], "VePASP_NIIVLClass_enum[]", 22) == 0)
      IVS_Class[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLaDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVLLaDis_m[]", 19) == 0)
      IVS_LaDis[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLgDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVLLgDis_m[]", 19) == 0)
      IVS_LgDis[3] = i;
    else if (strncmp(columns[i], "VbINP_NIIVLPresent_flg[flg]", 22) == 0 ||
             strncmp(columns[i], "VbPASP_NIIVLPresent_flg[]", 23) == 0)
      IVS_Present[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLV_mps[mps]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVLV_mps[]", 17) == 0)
      IVS_V[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLLaV_mps[mps]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVLLaV_mps[]", 19) == 0)
      IVS_LaV[3] = i;
    else if (strncmp(columns[i], "VfINP_NIIVLHeading_rad[]", 22) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVLHeading_rad[]", 23) == 0)
      IVS_Yaw[3] = i;
    else if (strncmp(columns[i], "VfPASP_NIIVLAcc_mpss[]", 20) == 0)
      IVS_A[3] = i;
    else if (strncmp(columns[i], "VePASP_NIIVLCutIn[]", 17) == 0)
      IVS_CUT[3] = i;

    // obstacle, 7 = NIIVR
    else if (strncmp(columns[i], "VeINP_NIIVRID[]", 13) == 0 ||
             strncmp(columns[i], "VePASP_NIIVRID[]", 14) == 0)
      IVS_ID[7] = i;
    else if (strncmp(columns[i], "VeINP_NIIVRClass_enum[enum]", 21) == 0 ||
             strncmp(columns[i], "VePASP_NIIVRClass_enum[]", 22) == 0)
      IVS_Class[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLaDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVRLaDis_m[]", 19) == 0)
      IVS_LaDis[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLgDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVRLgDis_m[]", 19) == 0)
      IVS_LgDis[7] = i;
    else if (strncmp(columns[i], "VbINP_NIIVRPresent_flg[flg]", 22) == 0 ||
             strncmp(columns[i], "VbPASP_NIIVRPresent_flg[]", 23) == 0)
      IVS_Present[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRV_mps[mps]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVRV_mps[]", 17) == 0)
      IVS_V[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRLaV_mps[mps]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVRLaV_mps[]", 19) == 0)
      IVS_LaV[7] = i;
    else if (strncmp(columns[i], "VfINP_NIIVRHeading_rad[]", 22) == 0 ||
             strncmp(columns[i], "VfPASP_NIIVRHeading_rad[]", 23) == 0)
      IVS_Yaw[7] = i;
    else if (strncmp(columns[i], "VfPASP_NIIVRAcc_mpss[]", 20) == 0)
      IVS_A[7] = i;
    else if (strncmp(columns[i], "VePASP_NIIVRCutIn[]", 17) == 0)
      IVS_CUT[7] = i;

    // obstacle, 4 = RIVL
    else if (strncmp(columns[i], "VeINP_RIVLID[]", 12) == 0 ||
             strncmp(columns[i], "VePASP_RIVLID[]", 13) == 0)
      IVS_ID[4] = i;
    else if (strncmp(columns[i], "VeINP_RIVLClass_enum[enum]", 20) == 0 ||
             strncmp(columns[i], "VePASP_RIVLClass_enum[]", 21) == 0)
      IVS_Class[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLaDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLLaDis_m[]", 18) == 0)
      IVS_LaDis[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLgDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLLgDis_m[]", 18) == 0)
      IVS_LgDis[4] = i;
    else if (strncmp(columns[i], "VbINP_RIVLPresent_flg[flg]", 21) == 0 ||
             strncmp(columns[i], "VbPASP_RIVLPresent_flg[]", 22) == 0)
      IVS_Present[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLV_mps[mps]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLV_mps[]", 16) == 0)
      IVS_V[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLLaV_mps[mps]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLLaV_mps[]", 18) == 0)
      IVS_LaV[4] = i;
    else if (strncmp(columns[i], "VfINP_RIVLHeading_rad[]", 21) == 0 ||
             strncmp(columns[i], "VfPASP_RIVLHeading_rad[]", 22) == 0)
      IVS_Yaw[4] = i;
    else if (strncmp(columns[i], "VfPASP_RIVLAcc_mpss[]", 19) == 0)
      IVS_A[4] = i;

    // obstacle, 8 = RIVR
    else if (strncmp(columns[i], "VeINP_RIVRID[]", 12) == 0 ||
             strncmp(columns[i], "VePASP_RIVRID[]", 13) == 0)
      IVS_ID[8] = i;
    else if (strncmp(columns[i], "VeINP_RIVRClass_enum[enum]", 20) == 0 ||
             strncmp(columns[i], "VePASP_RIVRClass_enum[]", 21) == 0)
      IVS_Class[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLaDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVRLaDis_m[]", 18) == 0)
      IVS_LaDis[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLgDis_m[m]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVRLgDis_m[]", 18) == 0)
      IVS_LgDis[8] = i;
    else if (strncmp(columns[i], "VbINP_RIVRPresent_flg[flag]", 21) == 0 ||
             strncmp(columns[i], "VbPASP_RIVRPresent_flg[]", 22) == 0)
      IVS_Present[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRV_mps[mps]", 15) == 0 ||
             strncmp(columns[i], "VfPASP_RIVRV_mps[]", 16) == 0)
      IVS_V[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRLaV_mps[mps]", 17) == 0 ||
             strncmp(columns[i], "VfPASP_RIVRLaV_mps[]", 18) == 0)
      IVS_LaV[8] = i;
    else if (strncmp(columns[i], "VfINP_RIVRHeading_rad[]", 21) == 0 ||
             strncmp(columns[i], "VfPASP_RIVRHeading_rad[]", 22) == 0)
      IVS_Yaw[8] = i;
    else if (strncmp(columns[i], "VfPASP_RIVRAcc_mpss[]", 19) == 0)
      IVS_A[8] = i;

    // obstacle, 5 = RIIVL
    else if (strncmp(columns[i], "VeINP_RIIVLID[]", 13) == 0 ||
             strncmp(columns[i], "VePASP_RIIVLID[]", 14) == 0)
      IVS_ID[5] = i;
    else if (strncmp(columns[i], "VeINP_RIIVLClass_enum[enum]", 21) == 0 ||
             strncmp(columns[i], "VePASP_RIIVLClass_enum[]", 22) == 0)
      IVS_Class[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLaDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVLLaDis_m[]", 19) == 0)
      IVS_LaDis[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLgDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVLLgDis_m[]", 19) == 0)
      IVS_LgDis[5] = i;
    else if (strncmp(columns[i], "VbINP_RIIVLPresent_flg[flag]", 22) == 0 ||
             strncmp(columns[i], "VbPASP_RIIVLPresent_flg[]", 23) == 0)
      IVS_Present[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLV_mps[mps]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVLV_mps[]", 17) == 0)
      IVS_V[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLLaV_mps[mps]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVLLaV_mps[]", 19) == 0)
      IVS_LaV[5] = i;
    else if (strncmp(columns[i], "VfINP_RIIVLHeading_rad[]", 22) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVLHeading_rad[]", 23) == 0)
      IVS_Yaw[5] = i;
    else if (strncmp(columns[i], "VfPASP_RIIVLAcc_mpss[]", 20) == 0)
      IVS_A[5] = i;

    // obstacle, 9 = RIIVR
    else if (strncmp(columns[i], "VeINP_RIIVRID[]", 13) == 0 ||
             strncmp(columns[i], "VePASP_RIIVRID[]", 14) == 0)
      IVS_ID[9] = i;
    else if (strncmp(columns[i], "VeINP_RIIVRClass_enum[enum]", 21) == 0 ||
             strncmp(columns[i], "VePASP_RIIVRClass_enum[]", 22) == 0)
      IVS_Class[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLaDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVRLaDis_m[]", 19) == 0)
      IVS_LaDis[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLgDis_m[m]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVRLgDis_m[]", 19) == 0)
      IVS_LgDis[9] = i;
    else if (strncmp(columns[i], "VbINP_RIIVRPresent_flg[flag]", 22) == 0 ||
             strncmp(columns[i], "VbPASP_RIIVRPresent_flg[]", 23) == 0)
      IVS_Present[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRV_mps[mps]", 16) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVRV_mps[]", 17) == 0)
      IVS_V[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRLaV_mps[mps]", 18) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVRLaV_mps[]", 19) == 0)
      IVS_LaV[9] = i;
    else if (strncmp(columns[i], "VfINP_RIIVRHeading_rad[]", 22) == 0 ||
             strncmp(columns[i], "VfPASP_RIIVRHeading_rad[]", 23) == 0)
      IVS_Yaw[9] = i;
    else if (strncmp(columns[i], "VfPASP_RIIVRAcc_mpss[]", 20) == 0)
      IVS_A[9] = i;
#else
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
#endif

//  Mobile Eye original lines
#if RAW_LINE == 1
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

#else
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0LineFirstC0[]", 30) == 0)
      LH_C_0[0] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1LineFirstC0[]", 30) == 0)
      LH_C_1[0] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0LineFirstC1[]", 30) == 0)
      LH_C_0[1] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1LineFirstC1[]", 30) == 0)
      LH_C_1[1] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0LineFirstC2[]", 30) == 0)
      LH_C_0[2] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1LineFirstC2[]", 30) == 0)
      LH_C_1[2] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0LineFirstC3[]", 30) == 0)
      LH_C_0[3] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1LineFirstC3[]", 30) == 0)
      LH_C_1[3] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0FirstVRStart[]", 31) == 0)
      LH_C_0[6] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1FirstVRStart[]", 31) == 0)
      LH_C_1[6] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH0FirstVREnd[]", 29) == 0)
      LH_C_0[7] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LH1FirstVREnd[]", 29) == 0)
      LH_C_1[7] = i;

    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0LineFirstC0[]", 30) == 0)
      LA_C_0[0] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1LineFirstC0[]", 30) == 0)
      LA_C_1[0] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0LineFirstC1[]", 30) == 0)
      LA_C_0[1] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1LineFirstC1[]", 30) == 0)
      LA_C_1[1] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0LineFirstC2[]", 30) == 0)
      LA_C_0[2] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1LineFirstC2[]", 30) == 0)
      LA_C_1[2] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0LineFirstC3[]", 30) == 0)
      LA_C_0[3] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1LineFirstC3[]", 30) == 0)
      LA_C_1[3] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0FirstVRStart[]", 31) == 0)
      LA_C_0[6] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1FirstVRStart[]", 31) == 0)
      LA_C_1[6] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA0FirstVREnd[]", 29) == 0)
      LA_C_0[7] = i;
    else if (strncmp(columns[i], "VfPASP_EnvModel_LA1FirstVREnd[]", 29) == 0)
      LA_C_1[7] = i;
#endif

    // TSR status
    else if (strncmp(columns[i], "VfLGIN_TsrTrgtSpdReq_kph[]", 24) == 0)
      TSR_Spd = i;
    else if (strncmp(columns[i], "VeLGIN_ISLCOverSpeedAlarmSts[]", 28) == 0)
      TSR_Warn = i;
    else if (strncmp(columns[i], "VeLGIN_TSI_SignShortStayMisc_1[]", 30) == 0)
      TSR_TSI[0] = i;
    else if (strncmp(columns[i], "VeLGIN_TSI_SignShortStayMisc_2[]", 30) == 0)
      TSR_TSI[1] = i;

    // TSR, original signals 3 key signs are enough
    else if (strncmp(columns[i], "VeINP_TSRID0Proc[]", 16) == 0)
      TSR_ID[0] = i;
    else if (strncmp(columns[i], "VuINP_TSRSignName0Proc[]", 22) == 0)
      TSR_Type[0] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLatDist0Proc_m[]", 27) == 0)
      TSR_LaDis[0] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLgDist0Proc_m[]", 26) == 0)
      TSR_LgDis[0] = i;
    else if (strncmp(columns[i], "VeINP_TSRID1Proc[]", 16) == 0)
      TSR_ID[1] = i;
    else if (strncmp(columns[i], "VuINP_TSRSignName1Proc[]", 22) == 0)
      TSR_Type[1] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLatDist1Proc_m[]", 27) == 0)
      TSR_LaDis[1] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLgDist1Proc_m[]", 26) == 0)
      TSR_LgDis[1] = i;
    else if (strncmp(columns[i], "VeINP_TSRID2Proc[]", 16) == 0)
      TSR_ID[2] = i;
    else if (strncmp(columns[i], "VuINP_TSRSignName2Proc[]", 22) == 0)
      TSR_Type[2] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLatDist2Proc_m[]", 27) == 0)
      TSR_LaDis[2] = i;
    else if (strncmp(columns[i], "VfINP_TSRSignLgDist2Proc_m[]", 26) == 0)
      TSR_LgDis[2] = i;

    else if (strncmp(columns[i], "VfPASP_EHNearestMergedist_m[]", 27) == 0)
      MRG_DIS = i;
    else if (strncmp(columns[i], "VePASP_EHNearestMergeptDirForEgo[]", 32) == 0)
      MRG_DIR = i;
    else if (strncmp(columns[i], "ViPASP_EHMerge_DP_ID[]", 20) == 0)
      MRG_ID = i;

    else if (strncmp(columns[i], "VfREM_EHEgoDP_C0_L4C[]", 20) == 0)
      EGO_DP[0] = i;
    else if (strncmp(columns[i], "VfREM_EHEgoDP_C1_L4C[]", 20) == 0)
      EGO_DP[1] = i;
    else if (strncmp(columns[i], "VfREM_EHEgoDP_C2_L4C[]", 20) == 0)
      EGO_DP[2] = i;
    else if (strncmp(columns[i], "VfREM_EHEgoDP_C3_L4C[]", 20) == 0)
      EGO_DP[3] = i;
    else if (strncmp(columns[i], "VfREM_EHTarDP_C0_L4C[]", 20) == 0)
      TAR_DP[0] = i;
    else if (strncmp(columns[i], "VfREM_EHTarDP_C1_L4C[]", 20) == 0)
      TAR_DP[1] = i;
    else if (strncmp(columns[i], "VfREM_EHTarDP_C2_L4C[]", 20) == 0)
      TAR_DP[2] = i;
    else if (strncmp(columns[i], "VfREM_EHTarDP_C3_L4C[]", 20) == 0)
      TAR_DP[3] = i;
  }
  // log total time. ATTENTION: NaN strings occupy last 8 rows of csv
  *totalFrame = valuesCount[Ts] - 8 > 0 ? valuesCount[Ts] - 8 : 0;

  // load data to local variables
  for (int t = 0; t < *totalFrame; ++t) {
    time_data[t] = values[Ts][t];

    //  inner spd lmt unit: m/s, record spd lmt unit:kph
    if (EGO_V) {
      egoSpd_data[t] = values[EGO_V][t];
      egoAcc_data[t] = values[EGO_A][t];
      spdLmt_data[t] = values[SPD_LMT][t];
      accMode_data[t] = values[ACC_MODE][t];
      tauGap_data[t] = values[TAU_GAP][t];
      spdPlanEnblSts_data[t] = values[ENBL_STS][t];
      innerSpdLmt_data[t] = values[IN_SPDLMT][t];
    }
    accDisRef_data[t] = ACC_DREF ? values[ACC_DREF][t] : 0;
    accCurveSpdLmt_data[t] = ACC_CURV ? values[ACC_CURV][t] : 0;

    truncated_col_data[t] = TRUC_CL ? values[TRUC_CL][t] : 0;
    specialCaseFlg_data[t] = values[SPC_FLG][t];
    scenarioFlg_data[t] = values[SCE_FLG][t];
    maxDecel_data[t] = MAX_DCL ? values[MAX_DCL][t] : 0;

    // alc find gap
    for (int k = 0; k < 6; k++) {
      alcStCoeff_data[k][t] = values[ALC_ST[k]][t];
    }
    alcGapIndex_data[t] = values[ALC_GAP][t];
    alcGapTarS_data[t] = values[ALC_TARS][t];
    alcGapTarV_data[t] = values[ALC_TARV][t];

    // alc_sts[0] for alc side: 0-0ff, 1-left, 2-right
    // alc_sts[1] for alc sts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving,
    // 4-in target line, 5-finished,6-Back to Ego, 8-takeover, 9-popMsgReq
    if (ALC_SIDE) {
      alcBehav_data[0][t] = values[ALC_SIDE][t];
      alcBehav_data[1][t] = values[ALC_STS][t];
      alcBehav_data[2][t] = values[ALC_LBT][t];
      alcBehav_data[3][t] = values[ALC_RBT][t];
      alcBehav_data[4][t] = NOA_STS ? values[NOA_STS][t] : 0;
      alcBehav_data[5][t] = ALC_RMP ? values[ALC_RMP][t] : 0;
      alcBehav_data[6][t] = ALC_RMP_D ? values[ALC_RMP_D][t] : 0;
      alcBehav_data[7][t] = ALC_EXT_D ? values[ALC_EXT_D][t] : 0;
    }

    // alc path and speed plan points
    for (int k = 0; k < 8; k++) {
      alc_path_data[k][t] = ALC_C[k] ? values[ALC_C[k]][t] : 0;
    }
    alc_path_data[7][t] = ALC_C[7] ? values[ALC_C[7]][t] : 50;

    for (int k = 0; k <= 5; k++) {
      if (0 == P_S[k])
        continue;
      s_points_data[k][t] = values[P_S[k]][t];
      v_points_data[k][t] = values[P_V[k]][t];
      a_points_data[k][t] = values[P_A[k]][t];
      t_points_data[k][t] = values[P_T[k]][t];
    }
    if (P_T[6]) {
      ctrl_point_data[0][t] = values[P_T[6]][t];
      ctrl_point_data[1][t] = values[P_S[6]][t];
      ctrl_point_data[2][t] = values[P_V[6]][t];
      ctrl_point_data[3][t] = values[P_A[6]][t];
    }

    // ego_path[9]: c0,c1,c2,c31,c32,c33,len1,len2,len3
    for (int k = 0; k < 9; k++) {
      ego_path_data[k][t] = EGO_PATH[k] ? values[EGO_PATH[k]][t] : 0;
      if (6 == k && 0 == EGO_PATH[k])
        ego_path_data[k][t] = 80;
      /*       if (6 == k)
              ego_path_data[k][t] = 80;  // default disp ego length: 80
            else if (7 == k || 8 == k || 4 == k || 5 == k)
              ego_path_data[k][t] = 0; */
    }
    // obstacles, BTL original: lateral distance/speed direction opposite.
    // longi distance needs compensation to transfer pos centre to centre
    // for front obs, ego front bumper center to obs rear bumper.
    // for rear obs, ego front bumper center to rear car center.
    float pos_x_compensation = 0;
    for (int k = 0; k < 10; k++) {
      if (IVS_Present[k] == 0) {
        continue;
      }
      // rear obs lat spd not stable, unacceptable
      if (0 == k || 2 == k || 3 == k || 6 == k || 7 == k) {
        pos_x_compensation =
            2.5f + (objs_type_data[k][t] == 2 ? 10.0f : 5.0f) / 2.0f;
        objs_speed_y_data[k][t] = IVS_LaV[k] ? values[IVS_LaV[k]][t] * -1 : 0;
      } else {
        pos_x_compensation = 2.5f;
        objs_speed_y_data[k][t] = IVS_LaV[k] ? values[IVS_LaV[k]][t] * -1 : 0;
      }
      objs_valid_flag_data[k][t] = values[IVS_Present[k]][t];
      objs_type_data[k][t] = values[IVS_Class[k]][t];
      objs_pos_x_data[k][t] = values[IVS_LgDis[k]][t] + pos_x_compensation;
      objs_pos_y_data[k][t] = values[IVS_LaDis[k]][t] * -1;
      objs_speed_x_data[k][t] = values[IVS_V[k]][t];
      objs_acc_x_data[k][t] = IVS_A[k] ? values[IVS_A[k]][t] : 0;
      objs_pos_yaw_data[k][t] = values[IVS_Yaw[k]][t] * -1;
      objs_cut_in_data[k][t] = IVS_CUT[k] ? values[IVS_CUT[k]][t] : 0;
      objs_id_data[k][t] = IVS_ID[k] ? values[IVS_ID[k]][t] : 0;

      if (k <= 1) {
        objs_lane_index_data[k][t] = 3;
      } else if (k <= 5) {
        objs_lane_index_data[k][t] = 2;
      } else {
        objs_lane_index_data[k][t] = 4;
      }
    }

    // ME original lines. c0~c3 opposite, c4 and c5 ignored, for no input
    for (int k = 0; k < 8; k++) {
      if (LH_C_0[k] == 0) {
        continue;
      }
#if RAW_LINE == 1
      const float correct_fac = (k <= 3 ? -1 : 1);
#else
      const float correct_fac = 1;
#endif
      if (k == 4 || k == 5) {
        continue;
      }
      l_path_data[k][t] = values[LH_C_0[k]][t] * correct_fac;
      r_path_data[k][t] = values[LH_C_1[k]][t] * correct_fac;
      ll_path_data[k][t] = values[LA_C_0[k]][t] * correct_fac;
      rr_path_data[k][t] = values[LA_C_1[k]][t] * correct_fac;
    }

    // TSR status
    if (TSR_Spd != 0) {
      tsr_spd_data[t] = values[TSR_Spd][t];
      tsr_spd_warn_data[t] = values[TSR_Warn][t];
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
    // REM, map merge
    if (MRG_DIS != 0) {
      merge_dis_data[t] = values[MRG_DIS][t];
      merge_dir_data[t] = values[MRG_DIR][t];
      merge_id_data[t] = values[MRG_ID][t];
    }
    /*     for (int k = 0; k < 4; k++) {
          if (EGO_DP[k] == 0)
            continue;
          ego_dp_data[k][t] = values[EGO_DP[k]][t];
          tar_dp_data[k][t] = values[TAR_DP[k]][t];
        } */
  }

  return;
}
