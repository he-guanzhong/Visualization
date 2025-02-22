#include "visualization/extension_package/ext_rem_load_log.h"

#ifdef REM_DEMO_TEST

extern float time_data[DATA_NUM];
extern float alc_path_data[8][DATA_NUM];  // c0~c5,start,end
extern float ego_path_data[9][DATA_NUM];  // c0~c2,c31~c32,len1~len3
extern float ll_path_data[8][DATA_NUM];   // c0~c5,start,end
extern float l_path_data[8][DATA_NUM];    // c0~c5,start,end
extern float r_path_data[8][DATA_NUM];
extern float rr_path_data[8][DATA_NUM];

extern int current_frame;

float ego_dp_org_data[4][DATA_NUM];  // c0~c3
float tar_dp_org_data[4][DATA_NUM];  // c0~c3
float ego_dp_off_data[6][DATA_NUM];  // c0~c3
float tar_dp_off_data[6][DATA_NUM];  // c0~c3

int ego_dp_point_nums_data[DATA_NUM];
int tar_dp_point_nums_data[DATA_NUM];
float ego_dp_point_x_data[POINT_NUM][DATA_NUM];
float ego_dp_point_y_data[POINT_NUM][DATA_NUM];
float tar_dp_point_x_data[POINT_NUM][DATA_NUM];
float tar_dp_point_y_data[POINT_NUM][DATA_NUM];

int dp_usage[2][DATA_NUM];
float special_point_distance_data[2][DATA_NUM];  // 0:merge 1:split
int split_attribute_data[2][DATA_NUM];           // 0:direction 1:id
int forward_lanes_data[4][DATA_NUM];

float left_lanemark_data[3][DATA_NUM];
float right_lanemark_data[3][DATA_NUM];

void RemDataParsing(int* totalFrame) {
  /* *totalFrame = current_frame;
    printf("total frame is %d\n", *totalFrame);
    for (int i = 0; i < 10; i++) {
      printf("t1: %.3f\t, t2: %.3f\t, dp: %.3f\n", time_data[i],
             test_time_data[i], ego_dp_data[0][i]);
    }
    for (int i = 0; i < 20000; i++) {
      time_data[i] = test_time_data[i];
    } */
}
#endif
