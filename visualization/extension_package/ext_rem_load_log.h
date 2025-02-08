#ifndef EXT_REM_LOAD_LOG_
#define EXT_REM_LOAD_LOG_

#ifdef REM_DEMO_TEST

/* Note: max frame numbers */
#ifndef DATA_NUM
#define DATA_NUM 20480
#endif

#define POINT_NUM 15

extern float ego_dp_point_x_data[POINT_NUM][DATA_NUM];
extern float ego_dp_point_y_data[POINT_NUM][DATA_NUM];
extern float tar_dp_point_x_data[POINT_NUM][DATA_NUM];
extern float tar_dp_point_y_data[POINT_NUM][DATA_NUM];

extern int dp_point_nums_data[DATA_NUM];
extern float ego_dp_org_data[4][DATA_NUM];  // c0~c3
extern float tar_dp_org_data[4][DATA_NUM];  // c0~c3
extern float ego_dp_off_data[4][DATA_NUM];  // c0~c3
extern float tar_dp_off_data[4][DATA_NUM];  // c0~c3

void RemDataParsing(int* totalFrame);

#endif

#endif  // EXT_REM_LOAD_LOG_
