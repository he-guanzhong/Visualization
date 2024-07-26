#include "visualization/agsm/show_agsm_read_data.h"
#ifdef AGSM_LOCAL_TEST

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