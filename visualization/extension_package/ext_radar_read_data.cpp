#include "visualization/extension_package/ext_radar_read_data.h"

#ifdef AGSM_DEMO_TEST

void ReadAgsmInputData(const int t,
                       MotionInfo* motionInfo,
                       AgsmLinesInfo* agsmLinesInfo,
                       SsmObjType* ssmObjs) {
  // ego motion status
  motionInfo->egoSpd = VehSpd_data[t];

  // obstacles
  ssmObjs->obj_num = 10;
  for (int i = 0; i < ssmObjs->obj_num; i++) {
    if (i == 0) {  // only cipv is read
      ssmObjs->obj_lists[i].valid_flag = Obstacles_valid_flag_data[i][t];
      ssmObjs->obj_lists[i].type = Obstacles_type_data[i][t];
      ssmObjs->obj_lists[i].pos_x = Obstacles_pos_x_data[i][t];
      ssmObjs->obj_lists[i].pos_y = Obstacles_pos_y_data[i][t];
      ssmObjs->obj_lists[i].lane_index = Obstacles_lane_index_data[i][t];
      ssmObjs->obj_lists[i].speed_x = Obstacles_speed_x_data[i][t];
      ssmObjs->obj_lists[i].speed_y = Obstacles_speed_y_data[i][t];
      ssmObjs->obj_lists[i].acc_x = Obstacles_acc_x_data[i][t];
      ssmObjs->obj_lists[i].pos_yaw = Obstacles_pos_yaw_data[i][t];
    }
  }

  // lines
  agsmLinesInfo->conft_path_record.c0 = ConftPathTypC0_data[t];
  agsmLinesInfo->conft_path_record.c1 = ConftPathTypC1_data[t];
  agsmLinesInfo->conft_path_record.c2 = ConftPathTypC2_data[t];
  agsmLinesInfo->conft_path_record.c3_array[0] = ConftPathTypC31_data[t];
  agsmLinesInfo->conft_path_record.c3_array[1] = ConftPathTypC32_data[t];
  agsmLinesInfo->conft_path_record.c3_array[2] = ConftPathTypC33_data[t];
  agsmLinesInfo->conft_path_record.segment_length_array[0] =
      ConftPathLength1_data[t];
  agsmLinesInfo->conft_path_record.segment_length_array[1] =
      ConftPathLength2_data[t];
  agsmLinesInfo->conft_path_record.segment_length_array[2] =
      ConftPathLength3_data[t];
  agsmLinesInfo->conft_path_record.valid = ConftPathValid_data[t];

  agsmLinesInfo->LH0.c0 = LeftLine_data[0][t];
  agsmLinesInfo->LH0.c1 = LeftLine_data[1][t];
  agsmLinesInfo->LH0.c2 = LeftLine_data[2][t];
  agsmLinesInfo->LH0.c3 = LeftLine_data[3][t];
  agsmLinesInfo->LH0.start = LeftLine_data[6][t];
  agsmLinesInfo->LH0.end = LeftLine_data[7][t];
  agsmLinesInfo->LH1.c0 = RightLine_data[0][t];
  agsmLinesInfo->LH1.c1 = RightLine_data[1][t];
  agsmLinesInfo->LH1.c2 = RightLine_data[2][t];
  agsmLinesInfo->LH1.c3 = RightLine_data[3][t];
  agsmLinesInfo->LH1.start = RightLine_data[6][t];
  agsmLinesInfo->LH1.end = RightLine_data[7][t];
  return;
}

#endif

#ifdef RADAR_DEMO_TEST

void ReadRadarInputData(const int t, RadarObjInfo* radarObjsInfo) {
  for (int j = 0; j < OBJ_NUM; j++) {
    radarObjsInfo[0].iObjectId[j] = iFL_ObjectId_data[j][t];
    radarObjsInfo[0].fExistProb[j] = fFL_ExistProb_data[j][t];
    radarObjsInfo[0].fDistX[j] = fFL_DistX_data[j][t];
    radarObjsInfo[0].fDistY[j] = fFL_DistY_data[j][t];
  }

  for (int j = 0; j < OBJ_NUM; j++) {
    radarObjsInfo[1].iObjectId[j] = iFR_ObjectId_data[j][t];
    radarObjsInfo[1].fExistProb[j] = fFR_ExistProb_data[j][t];
    radarObjsInfo[1].fDistX[j] = fFR_DistX_data[j][t];
    radarObjsInfo[1].fDistY[j] = fFR_DistY_data[j][t];
  }

  for (int j = 0; j < OBJ_NUM; j++) {
    radarObjsInfo[2].iObjectId[j] = iRL_ObjectId_data[j][t];
    radarObjsInfo[2].fExistProb[j] = fRL_ExistProb_data[j][t];
    radarObjsInfo[2].fDistX[j] = fRL_DistX_data[j][t];
    radarObjsInfo[2].fDistY[j] = fRL_DistY_data[j][t];
  }

  for (int j = 0; j < OBJ_NUM; j++) {
    radarObjsInfo[3].iObjectId[j] = iRR_ObjectId_data[j][t];
    radarObjsInfo[3].fExistProb[j] = fRR_ExistProb_data[j][t];
    radarObjsInfo[3].fDistX[j] = fRR_DistX_data[j][t];
    radarObjsInfo[3].fDistY[j] = fRR_DistY_data[j][t];
  }
}

#endif

#ifdef MEOBJ_DEMO_TEST
void ReadMeObjInputData(const int t, MeObjInfo* meObjsInfo) {
  for (int j = 0; j < OBJ_NUM; j++) {
    meObjsInfo->iId[j] = iId_data[j][t];
    meObjsInfo->iClass[j] = iClass_data[j][t];
    meObjsInfo->fLongDis[j] = fLongDis_data[j][t];
    meObjsInfo->fLatDis[j] = fLatDis_data[j][t];
    meObjsInfo->fLen[j] = fLen_data[j][t];
    meObjsInfo->fWid[j] = fWid_data[j][t];
    meObjsInfo->fLongSpd[j] = fLongSpd_data[j][t];
  }
}

#endif
