#ifndef SHOW_DATA_STORAGE_H_
#define SHOW_DATA_STORAGE_H_

/* csv row nums (total steps) exceeding DATA_NUM will result in errors */
#ifndef DATA_NUM
#define DATA_NUM 24480
#endif

#ifndef OBJ_NUM
#define OBJ_NUM 14
#endif

extern float time_data[DATA_NUM];
extern float egoSpd_data[DATA_NUM];
extern float egoAcc_data[DATA_NUM];
extern float spdLmt_data[DATA_NUM];
extern float alc_path_data[8][DATA_NUM];
extern float alc_path2_data[8][DATA_NUM];
extern float ego_path_data[9][DATA_NUM];
extern int alcBehav_data[8][DATA_NUM];
extern int accMode_data[DATA_NUM];
extern int tauGap_data[DATA_NUM];
extern float accDisRef_data[DATA_NUM];
extern float accCurveSpdLmt_data[DATA_NUM];

extern int spdPlanEnblSts_data[DATA_NUM];
extern int truncated_col_data[DATA_NUM];
extern float innerSpdLmt_data[DATA_NUM];
extern int specialCaseFlg_data[DATA_NUM];
extern int scenarioFlg_data[DATA_NUM];
extern float maxDecel_data[DATA_NUM];
extern int alcGapIndex_data[DATA_NUM];
extern float alcGapTarS_data[DATA_NUM];
extern float alcGapTarV_data[DATA_NUM];
extern float alcStCoeff_data[6][DATA_NUM];

extern float ctrl_point_data[4][DATA_NUM];
extern float s_points_data[6][DATA_NUM];
extern float v_points_data[6][DATA_NUM];
extern float a_points_data[6][DATA_NUM];
extern float t_points_data[6][DATA_NUM];

extern bool objs_valid_flag_data[OBJ_NUM][DATA_NUM];
extern int objs_lane_index_data[OBJ_NUM][DATA_NUM];
extern int objs_type_data[OBJ_NUM][DATA_NUM];
extern float objs_pos_x_data[OBJ_NUM][DATA_NUM];
extern float objs_pos_y_data[OBJ_NUM][DATA_NUM];
extern float objs_speed_x_data[OBJ_NUM][DATA_NUM];
extern float objs_speed_y_data[OBJ_NUM][DATA_NUM];
extern float objs_acc_x_data[OBJ_NUM][DATA_NUM];
extern float objs_pos_yaw_data[OBJ_NUM][DATA_NUM];
extern int objs_cut_in_data[OBJ_NUM][DATA_NUM];
extern int objs_id_data[OBJ_NUM][DATA_NUM];
extern float objs_speed_y_f_data[OBJ_NUM][DATA_NUM];
extern float objs_width_data[OBJ_NUM][DATA_NUM];

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

extern float merge_dis_data[DATA_NUM];
extern int merge_dir_data[DATA_NUM];
extern int merge_id_data[DATA_NUM];

extern float original_data[DATA_NUM];
extern float loopback_data[DATA_NUM];
extern float tempMeasureVal_data[4][DATA_NUM];

#endif  // SHOW_DATA_STORAGE_H_
