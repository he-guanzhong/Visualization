#include "visualization/extension_package/ext_rem_read_data.h"

#ifdef REM_DEMO_TEST

extern float time_data[DATA_NUM];
extern float alc_path_data[8][DATA_NUM];  // c0~c5,start,end
extern float ego_path_data[9][DATA_NUM];  // c0~c2,c31~c32,len1~len3
extern float ll_path_data[8][DATA_NUM];   // c0~c5,start,end
extern float l_path_data[8][DATA_NUM];    // c0~c5,start,end
extern float r_path_data[8][DATA_NUM];
extern float rr_path_data[8][DATA_NUM];
extern float ego_dp_data[4][DATA_NUM];  // c0~c3
extern float tar_dp_data[4][DATA_NUM];  // c0~c3

void ReadRemInputData(const int t,
                      MotionInfo* motionInfo,
                      LinesInfo* linesInfo,
                      RemPointsInfo* remPointsInfo) {
  // ego path, cubic polynominal
  linesInfo->ego_coeffs.C0[0] = ego_path_data[0][t];
  linesInfo->ego_coeffs.C1[0] = ego_path_data[1][t];
  linesInfo->ego_coeffs.C2[0] = ego_path_data[2][t];
  linesInfo->ego_coeffs.C3[0] = ego_path_data[3][t];
  linesInfo->ego_coeffs.C3[1] = ego_path_data[4][t];
  linesInfo->ego_coeffs.C3[2] = ego_path_data[5][t];
  linesInfo->ego_coeffs.Len[0] = ego_path_data[6][t];
  linesInfo->ego_coeffs.Len[1] = ego_path_data[7][t];
  linesInfo->ego_coeffs.Len[2] = ego_path_data[8][t];
  for (int i = 0; i <= 1; i++) {
    linesInfo->ego_coeffs.C0[i + 1] =
        linesInfo->ego_coeffs.C0[i] +
        linesInfo->ego_coeffs.C1[i] * linesInfo->ego_coeffs.Len[i] +
        linesInfo->ego_coeffs.C2[i] * linesInfo->ego_coeffs.Len[i] *
            linesInfo->ego_coeffs.Len[i] +
        linesInfo->ego_coeffs.C3[i] * linesInfo->ego_coeffs.Len[i] *
            linesInfo->ego_coeffs.Len[i] * linesInfo->ego_coeffs.Len[i];
    linesInfo->ego_coeffs.C1[i + 1] =
        linesInfo->ego_coeffs.C1[i] +
        2.0f * linesInfo->ego_coeffs.C2[i] * linesInfo->ego_coeffs.Len[i] +
        3.0f * linesInfo->ego_coeffs.C3[i] * linesInfo->ego_coeffs.Len[i] *
            linesInfo->ego_coeffs.Len[i];
    linesInfo->ego_coeffs.C2[i + 1] =
        (2.0f * linesInfo->ego_coeffs.C2[i] +
         6.0f * linesInfo->ego_coeffs.C3[i] * linesInfo->ego_coeffs.Len[i]) /
        2.0f;
  }

  // alc_path and Mobileye lines, quintic polynominal
  for (int i = 0; i < 8; i++) {
    linesInfo->alc_coeffs[i] = alc_path_data[i][t];
    linesInfo->left_coeffs[i] = l_path_data[i][t];
    linesInfo->right_coeffs[i] = r_path_data[i][t];
    linesInfo->leftleft_coeffs[i] = ll_path_data[i][t];
    linesInfo->rightright_coeffs[i] = rr_path_data[i][t];
  }
  // dp lines
  for (int i = 0; i < 4; i++) {
    linesInfo->ego_dp[i] = ego_dp_data[i][t];
    linesInfo->tar_dp[i] = tar_dp_data[i][t];
  }
  linesInfo->ego_dp[6] = linesInfo->tar_dp[6] = -5;
  linesInfo->ego_dp[7] = linesInfo->tar_dp[7] = 40;

  // REM dp points
  remPointsInfo->nums = dp_point_nums_data[t];
  for (int i = 0; i < remPointsInfo->nums; i++) {
    remPointsInfo->egoDpPoint[i].x = ego_dp_point_x_data[i][t];
    remPointsInfo->egoDpPoint[i].y = ego_dp_point_y_data[i][t];
    remPointsInfo->tarDpPoint[i].x = tar_dp_point_x_data[i][t];
    remPointsInfo->tarDpPoint[i].y = tar_dp_point_y_data[i][t];
  }
  return;
}

#endif