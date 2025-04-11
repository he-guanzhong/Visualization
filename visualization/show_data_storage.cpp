#include "visualization/show_data_storage.h"

float time_data[DATA_NUM];
float egoSpd_data[DATA_NUM];
float egoAcc_data[DATA_NUM];
float spdLmt_data[DATA_NUM];
float alc_path_data[8][DATA_NUM];
float alc_path2_data[8][DATA_NUM];
float ego_path_data[9][DATA_NUM];
int alcBehav_data[8][DATA_NUM];
int accMode_data[DATA_NUM];
int tauGap_data[DATA_NUM];
float accDisRef_data[DATA_NUM];
float accCurveSpdLmt_data[DATA_NUM];

int spdPlanEnblSts_data[DATA_NUM];
int truncated_col_data[DATA_NUM];
float innerSpdLmt_data[DATA_NUM];
int specialCaseFlg_data[DATA_NUM];
int scenarioFlg_data[DATA_NUM];
float maxDecel_data[DATA_NUM];
int alcGapIndex_data[DATA_NUM];
float alcGapTarS_data[DATA_NUM];
float alcGapTarV_data[DATA_NUM];
float alcStCoeff_data[6][DATA_NUM];

float ctrl_point_data[4][DATA_NUM];
float s_points_data[6][DATA_NUM];
float v_points_data[6][DATA_NUM];
float a_points_data[6][DATA_NUM];
float t_points_data[6][DATA_NUM];

bool objs_valid_flag_data[OBJ_NUM][DATA_NUM];
int objs_lane_index_data[OBJ_NUM][DATA_NUM];
int objs_type_data[OBJ_NUM][DATA_NUM];
float objs_pos_x_data[OBJ_NUM][DATA_NUM];
float objs_pos_y_data[OBJ_NUM][DATA_NUM];
float objs_speed_x_data[OBJ_NUM][DATA_NUM];
float objs_speed_y_data[OBJ_NUM][DATA_NUM];
float objs_acc_x_data[OBJ_NUM][DATA_NUM];
float objs_pos_yaw_data[OBJ_NUM][DATA_NUM];
int objs_cut_in_data[OBJ_NUM][DATA_NUM];
int objs_id_data[OBJ_NUM][DATA_NUM];
float objs_speed_y_f_data[OBJ_NUM][DATA_NUM];
float objs_width_data[OBJ_NUM][DATA_NUM];

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

float merge_dis_data[DATA_NUM];
int merge_dir_data[DATA_NUM];
int merge_id_data[DATA_NUM];

float original_data[DATA_NUM];
float loopback_data[DATA_NUM];

float tempMeasureVal_data[4][DATA_NUM];

float reserved_data[4][DATA_NUM];
