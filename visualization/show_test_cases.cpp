#include "visualization/show_test_cases.h"

void CaseLeftChange(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 3;
  ssmObjs->obj_lists[0].pos_x = 40;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0.0f;
  ssmObjs->obj_lists[0].speed_x = 20.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[0].type = 1;        // hgz
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = FALSE;

  ssmObjs->obj_lists[2].pos_x = 1;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 15.0f;
  ssmObjs->obj_lists[2].speed_y = -6.8f / 5.0f;  // hgz
  ssmObjs->obj_lists[2].type = 1;                // hgz
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[4].pos_x = -20;
  ssmObjs->obj_lists[4].pos_y = 3.15f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 50.0f / 3.6f;
  ssmObjs->obj_lists[4].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[4].type = 2;        // hgz
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;
}

void CaseSideCarMoveSlowly(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 6;
  ssmObjs->obj_lists[6].pos_x = 40;
  ssmObjs->obj_lists[6].pos_y = -3.4f;
  ssmObjs->obj_lists[6].acc_x = 0.0f;
  ssmObjs->obj_lists[6].speed_x = 5.0f;
  ssmObjs->obj_lists[6].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[6].type = 2;        // hgz
  ssmObjs->obj_lists[6].lane_index = 4;
  ssmObjs->obj_lists[6].valid_flag = TRUE;

  ssmObjs->obj_lists[7].pos_x = 10;
  ssmObjs->obj_lists[7].pos_y = -3.4f;
  ssmObjs->obj_lists[7].acc_x = 0.0f;
  ssmObjs->obj_lists[7].speed_x = 6.0f;
  ssmObjs->obj_lists[7].speed_y = -6.8f / 5.0f;  // hgz
  ssmObjs->obj_lists[7].type = 1;                // hgz
  ssmObjs->obj_lists[7].lane_index = 4;
  ssmObjs->obj_lists[7].valid_flag = TRUE;

  ssmObjs->obj_lists[8].pos_x = -15;
  ssmObjs->obj_lists[8].pos_y = -3.4f;
  ssmObjs->obj_lists[8].acc_x = 0.0f;
  ssmObjs->obj_lists[8].speed_x = 1;
  ssmObjs->obj_lists[8].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[8].type = 0;        // hgz
  ssmObjs->obj_lists[8].lane_index = 4;
  ssmObjs->obj_lists[8].valid_flag = TRUE;

  ssmObjs->obj_lists[2].pos_x = 20;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 4;
  ssmObjs->obj_lists[2].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[2].type = 1;        // hgz
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[4].pos_x = -10;
  ssmObjs->obj_lists[4].pos_y = 3.4f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 3;
  ssmObjs->obj_lists[4].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[4].type = 0;        // hgz
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;
}

void Follow(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 1;
  ssmObjs->obj_lists[0].pos_x = 20;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = -3;
  ssmObjs->obj_lists[0].speed_x = 15.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[0].type = 1;        // hgz
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;
}

void LoadDummySSmData(SsmObjType* ssmObjs) {
  Follow(ssmObjs);
}
