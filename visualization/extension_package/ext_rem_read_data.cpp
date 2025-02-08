#include "visualization/extension_package/ext_rem_read_data.h"

#ifdef REM_DEMO_TEST

extern float time_data[DATA_NUM];
extern float alc_path_data[8][DATA_NUM];  // c0~c5,start,end
extern float ego_path_data[9][DATA_NUM];  // c0~c2,c31~c32,len1~len3
extern float ll_path_data[8][DATA_NUM];   // c0~c5,start,end
extern float l_path_data[8][DATA_NUM];    // c0~c5,start,end
extern float r_path_data[8][DATA_NUM];
extern float rr_path_data[8][DATA_NUM];

void ReadRemInputData(const int t,
                      MotionInfo* motionInfo,
                      LinesInfo* linesInfo,
                      RemInfo* remInfo) {
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

  // REM dp real lines
  for (int i = 0; i < 4; i++) {
    remInfo->ego_dp_org[i] = ego_dp_org_data[i][t];
    remInfo->tar_dp_org[i] = tar_dp_org_data[i][t];
  }
  remInfo->ego_dp_org[6] = remInfo->tar_dp_org[6] = -5;
  remInfo->ego_dp_org[7] = remInfo->tar_dp_org[7] = 40;

  // REM dp off lines
  for (int i = 0; i < 4; i++) {
    remInfo->ego_dp_off[i] = ego_dp_off_data[i][t];
    remInfo->tar_dp_off[i] = tar_dp_off_data[i][t];
  }
  remInfo->ego_dp_off[6] = remInfo->tar_dp_off[6] = -5;
  remInfo->ego_dp_off[7] = remInfo->tar_dp_off[7] = 40;

  // REM dp points
  remInfo->point_nums = dp_point_nums_data[t];
  for (int i = 0; i < remInfo->point_nums; i++) {
    remInfo->ego_dp_point[i].x = ego_dp_point_x_data[i][t];
    remInfo->ego_dp_point[i].y = ego_dp_point_y_data[i][t];
    remInfo->tar_dp_point[i].x = tar_dp_point_x_data[i][t];
    remInfo->tar_dp_point[i].y = tar_dp_point_y_data[i][t];
  }

  return;
}

#endif