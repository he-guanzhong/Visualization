#ifndef EXT_RADAR_LOAD_LOG_
#define EXT_RADAR_LOAD_LOG_

#include <stdio.h>
#include <string.h>

#ifdef AGSM_DEMO_TEST

#ifndef DATA_NUM
#define DATA_NUM 20480
#endif

extern float VehSpd_data[DATA_NUM];
extern float LeftLine_data[8][DATA_NUM];
extern float RightLine_data[8][DATA_NUM];
extern float NextLeftLine_data[8][DATA_NUM];
extern float NextRightLine_data[8][DATA_NUM];

extern float ConftPathTypC0_data[DATA_NUM];
extern float ConftPathTypC1_data[DATA_NUM];
extern float ConftPathTypC2_data[DATA_NUM];
extern float ConftPathTypC31_data[DATA_NUM];
extern float ConftPathTypC32_data[DATA_NUM];
extern float ConftPathTypC33_data[DATA_NUM];
extern float ConftPathLength1_data[DATA_NUM];
extern float ConftPathLength2_data[DATA_NUM];
extern float ConftPathLength3_data[DATA_NUM];
extern int ConftPathValid_data[DATA_NUM];

extern bool Obstacles_valid_flag_data[10][DATA_NUM];
extern int Obstacles_lane_index_data[10][DATA_NUM];
extern int Obstacles_type_data[10][DATA_NUM];
extern float Obstacles_pos_x_data[10][DATA_NUM];
extern float Obstacles_pos_y_data[10][DATA_NUM];
extern float Obstacles_speed_x_data[10][DATA_NUM];
extern float Obstacles_speed_y_data[10][DATA_NUM];
extern float Obstacles_acc_x_data[10][DATA_NUM];
extern float Obstacles_pos_yaw_data[10][DATA_NUM];

void AgsmDataParsing(float** values,
                     int numColumns,
                     char** columns,
                     int* valuesCount,
                     int* totalFrame);

#endif

#ifdef RADAR_DEMO_TEST

/* Note:Please change the marco define as follows
  #define MAX_LINE_SIZE 22000
  #define MAX_COLUMNS 350
*/

#ifndef DATA_NUM
#define DATA_NUM 20480
#endif

#define OBJ_NUM 20

extern int iFL_ObjectId_data[OBJ_NUM][DATA_NUM];
extern float fFL_ExistProb_data[OBJ_NUM][DATA_NUM];
extern float fFL_DistX_data[OBJ_NUM][DATA_NUM];
extern float fFL_DistY_data[OBJ_NUM][DATA_NUM];

extern int iFR_ObjectId_data[OBJ_NUM][DATA_NUM];
extern float fFR_ExistProb_data[OBJ_NUM][DATA_NUM];
extern float fFR_DistX_data[OBJ_NUM][DATA_NUM];
extern float fFR_DistY_data[OBJ_NUM][DATA_NUM];

extern int iRL_ObjectId_data[OBJ_NUM][DATA_NUM];
extern float fRL_ExistProb_data[OBJ_NUM][DATA_NUM];
extern float fRL_DistX_data[OBJ_NUM][DATA_NUM];
extern float fRL_DistY_data[OBJ_NUM][DATA_NUM];

extern int iRR_ObjectId_data[OBJ_NUM][DATA_NUM];
extern float fRR_ExistProb_data[OBJ_NUM][DATA_NUM];
extern float fRR_DistX_data[OBJ_NUM][DATA_NUM];
extern float fRR_DistY_data[OBJ_NUM][DATA_NUM];

void RadarDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame);
#endif

#ifdef MEOBJ_DEMO_TEST

#ifndef DATA_NUM
#define DATA_NUM 20480
#endif

#define OBJ_NUM 12

extern int iId_data[OBJ_NUM][DATA_NUM];
extern int iClass_data[OBJ_NUM][DATA_NUM];
extern float fLongDis_data[OBJ_NUM][DATA_NUM];
extern float fLatDis_data[OBJ_NUM][DATA_NUM];
extern float fLen_data[OBJ_NUM][DATA_NUM];
extern float fWid_data[OBJ_NUM][DATA_NUM];
extern float fLongSpd_data[OBJ_NUM][DATA_NUM];

void MeObjDataParsing(float** values,
                      const int numColumns,
                      char** columns,
                      const int* valuesCount,
                      int* totalFrame);
#endif

#endif  // EXT_RADAR_LOAD_LOG_
