#include "visualization/show_data_storage.h"

float time_data[DATA_NUM];
float egoSpd_data[DATA_NUM];
float egoAcc_data[DATA_NUM];
float spdLmt_data[DATA_NUM];
float alc_path_data[8][DATA_NUM];
float ego_path_data[9][DATA_NUM];
int alcBehav_data[5][DATA_NUM];
int accMode_data[DATA_NUM];

int spdPlanEnblSts_data[DATA_NUM];
int truncated_col_data[DATA_NUM];
float innerSpdLmt_data[DATA_NUM];
int specialCaseFlg_data[DATA_NUM];
int scenarioFlg_data[DATA_NUM];

float ctrl_point_data[2][DATA_NUM];
float s_points_data[6][DATA_NUM];
float v_points_data[6][DATA_NUM];
float a_points_data[6][DATA_NUM];
float t_points_data[6][DATA_NUM];

bool objs_valid_flag_data[10][DATA_NUM];
int objs_lane_index_data[10][DATA_NUM];
int objs_type_data[10][DATA_NUM];
float objs_pos_x_data[10][DATA_NUM];
float objs_pos_y_data[10][DATA_NUM];
float objs_speed_x_data[10][DATA_NUM];
float objs_speed_y_data[10][DATA_NUM];
float objs_acc_x_data[10][DATA_NUM];
float objs_pos_yaw_data[10][DATA_NUM];

int tsr_spd_data[DATA_NUM];
bool tsr_spd_warn_data[DATA_NUM];
int tsr_tsi_data[2][DATA_NUM];

bool tsr_valid_flag_data[3][DATA_NUM];
int tsr_type_data[3][DATA_NUM];
float tsr_pos_x_data[3][DATA_NUM];
float tsr_pos_y_data[3][DATA_NUM];

float ll_path_data[8][DATA_NUM];
float l_path_data[8][DATA_NUM];
float r_path_data[8][DATA_NUM];
float rr_path_data[8][DATA_NUM];

float original_data[DATA_NUM];
float loopback_data[DATA_NUM];
float tempMeasureVal_data[DATA_NUM];
float alcStCoeff_data[6][DATA_NUM];

#ifdef RADAR_DEMO_DISP
int iObjectId_data[32][DATA_NUM];
float fDistX_data[32][DATA_NUM];
float fDistY_data[32][DATA_NUM];
#endif
