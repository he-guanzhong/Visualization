#include "visualization/extension_package/ext_rem_load_log.h"

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

float ego_dp_point_x_data[POINT_NUM][DATA_NUM];
float ego_dp_point_y_data[POINT_NUM][DATA_NUM];
float tar_dp_point_x_data[POINT_NUM][DATA_NUM];
float tar_dp_point_y_data[POINT_NUM][DATA_NUM];
int dp_point_nums_data[DATA_NUM];

void RemDataParsing(int* totalFrame) {
  *totalFrame = 0;
  for (int i = 0; i < DATA_NUM; i++) {
    time_data[i] = 0;

    return;
  }
}
#endif
