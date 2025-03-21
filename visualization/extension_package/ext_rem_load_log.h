#ifndef EXT_REM_LOAD_LOG_
#define EXT_REM_LOAD_LOG_

#ifdef REM_DEMO_TEST

/* Note: max frame numbers */
#ifndef DATA_NUM
#define DATA_NUM 24480
#endif

#define POINT_NUM 24

extern float ego_dp_point_x_data[POINT_NUM][DATA_NUM];
extern float ego_dp_point_y_data[POINT_NUM][DATA_NUM];
extern float tar_dp_point_x_data[POINT_NUM][DATA_NUM];
extern float tar_dp_point_y_data[POINT_NUM][DATA_NUM];

extern int ego_dp_point_nums_data[DATA_NUM];
extern int tar_dp_point_nums_data[DATA_NUM];
extern float ego_dp_org_data[4][DATA_NUM];  // c0~c3
extern float tar_dp_org_data[4][DATA_NUM];  // c0~c3
extern float ego_dp_off_data[6][DATA_NUM];  // c0~c5
extern float tar_dp_off_data[6][DATA_NUM];  // c0~c5

extern int dp_usage[2][DATA_NUM];
extern float special_point_distance_data[2][DATA_NUM];  // 0:merge 1:split
extern int split_attribute_data[2][DATA_NUM];
extern int forward_lanes_data[4][DATA_NUM];

extern float left_lanemark_data[3][DATA_NUM];
extern float right_lanemark_data[3][DATA_NUM];

void RemDataParsing(int* totalFrame);

#endif

#endif  // EXT_REM_LOAD_LOG_
