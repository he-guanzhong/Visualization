#include "visualization/show_test_cases.h"

void CaseLeftChange(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 10;
  ssmObjs->obj_lists[0].pos_x = 80;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0.0f;
  ssmObjs->obj_lists[0].speed_x = 20.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[0].type = 1;        // hgz
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;

  ssmObjs->obj_lists[2].pos_x = 1;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 50.0f / 3.6f;
  ssmObjs->obj_lists[2].speed_y = -3.4f / 5.0f;
  ssmObjs->obj_lists[2].type = 1;
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[3].pos_x = 80;
  ssmObjs->obj_lists[3].pos_y = 3.4f;
  ssmObjs->obj_lists[3].acc_x = 0.0f;
  ssmObjs->obj_lists[3].speed_x = 15.0f;
  ssmObjs->obj_lists[3].speed_y = 0;  // hgz
  ssmObjs->obj_lists[3].type = 1;     // hgz
  ssmObjs->obj_lists[3].lane_index = 2;
  ssmObjs->obj_lists[3].valid_flag = FALSE;

  ssmObjs->obj_lists[4].pos_x = -20;
  ssmObjs->obj_lists[4].pos_y = 3.15f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 50.0f / 3.6f;
  ssmObjs->obj_lists[4].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[4].type = 0;        // hgz
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;
}

void CaseSideCarMoveSlowly(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 10;

  ssmObjs->obj_lists[0].pos_x = 60;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0.0f;
  ssmObjs->obj_lists[0].speed_x = 20.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[0].type = 1;        // hgz
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;

  ssmObjs->obj_lists[1].pos_x = -23;
  ssmObjs->obj_lists[1].pos_y = 0;
  ssmObjs->obj_lists[1].acc_x = 0.0f;
  ssmObjs->obj_lists[1].speed_x = 15.0f;
  ssmObjs->obj_lists[1].speed_y = 0.0f;
  ssmObjs->obj_lists[1].type = 0;
  ssmObjs->obj_lists[1].lane_index = 3;
  ssmObjs->obj_lists[1].valid_flag = TRUE;

  ssmObjs->obj_lists[2].pos_x = 40;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 15;
  ssmObjs->obj_lists[2].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[2].type = 1;        // hgz
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[3].pos_x = 80;
  ssmObjs->obj_lists[3].pos_y = 3.4f;
  ssmObjs->obj_lists[3].acc_x = 0.0f;
  ssmObjs->obj_lists[3].speed_x = 15.0f;
  ssmObjs->obj_lists[3].speed_y = 0;  // hgz
  ssmObjs->obj_lists[3].type = 1;     // hgz
  ssmObjs->obj_lists[3].lane_index = 2;
  ssmObjs->obj_lists[3].valid_flag = TRUE;

  ssmObjs->obj_lists[4].pos_x = -10;
  ssmObjs->obj_lists[4].pos_y = 3.4f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 3;
  ssmObjs->obj_lists[4].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[4].type = 0;        // hgz
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;

  ssmObjs->obj_lists[5].pos_x = -23;
  ssmObjs->obj_lists[5].pos_y = 3.4f;
  ssmObjs->obj_lists[5].acc_x = 0.0f;
  ssmObjs->obj_lists[5].speed_x = 3;
  ssmObjs->obj_lists[5].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[5].type = 0;        // hgz
  ssmObjs->obj_lists[5].lane_index = 2;
  ssmObjs->obj_lists[5].valid_flag = TRUE;

  ssmObjs->obj_lists[6].pos_x = 80;
  ssmObjs->obj_lists[6].pos_y = -3.4f;
  ssmObjs->obj_lists[6].acc_x = 0.0f;
  ssmObjs->obj_lists[6].speed_x = 5.0f;
  ssmObjs->obj_lists[6].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[6].type = 1;        // hgz
  ssmObjs->obj_lists[6].lane_index = 4;
  ssmObjs->obj_lists[6].valid_flag = TRUE;

  ssmObjs->obj_lists[7].pos_x = 40;
  ssmObjs->obj_lists[7].pos_y = -3.4f;
  ssmObjs->obj_lists[7].acc_x = 0.0f;
  ssmObjs->obj_lists[7].speed_x = 6.0f;
  ssmObjs->obj_lists[7].speed_y = -6.8f / 5.0f;  // hgz
  ssmObjs->obj_lists[7].type = 1;                // hgz
  ssmObjs->obj_lists[7].lane_index = 4;
  ssmObjs->obj_lists[7].valid_flag = TRUE;

  ssmObjs->obj_lists[8].pos_x = -10;
  ssmObjs->obj_lists[8].pos_y = -3.4f;
  ssmObjs->obj_lists[8].acc_x = 0.0f;
  ssmObjs->obj_lists[8].speed_x = 1;
  ssmObjs->obj_lists[8].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[8].type = 0;        // hgz
  ssmObjs->obj_lists[8].lane_index = 4;
  ssmObjs->obj_lists[8].valid_flag = TRUE;

  ssmObjs->obj_lists[9].pos_x = -23;
  ssmObjs->obj_lists[9].pos_y = -3.4f;
  ssmObjs->obj_lists[9].acc_x = 0.0f;
  ssmObjs->obj_lists[9].speed_x = 1;
  ssmObjs->obj_lists[9].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[9].type = 0;        // hgz
  ssmObjs->obj_lists[9].lane_index = 4;
  ssmObjs->obj_lists[9].valid_flag = TRUE;
}

void CaseFollow(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 1;
  ssmObjs->obj_lists[0].pos_x = 100;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0;
  ssmObjs->obj_lists[0].speed_x = 0.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;  // hgz
  ssmObjs->obj_lists[0].type = 2;        // hgz
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = FALSE;
}

void LoadDummyMotionData(float* egoSpd,
                         float* egoAcc,
                         float* spdLmt,
                         int* accMode,
                         AlcBehavior* alcBehav) {
  *egoSpd = 80.0f / 3.6f, *egoAcc = 0.0f, *spdLmt = 120.0f;
  *accMode = 5;
  alcBehav->AutoLaneChgSide = 0;
  alcBehav->AutoLaneChgSts = 1;
  alcBehav->LeftBoundaryType = 2;
  alcBehav->RightBoundaryType = 2;

  return;
}
void LoadDummyPathData(const float* alc_coeffs,
                       const float* ego_coeffs,
                       AlcPathVcc* alcPathVcc,
                       EgoPathVcc* egoPathVcc) {
  *alcPathVcc = {alc_coeffs[0], alc_coeffs[1], alc_coeffs[2], alc_coeffs[3],
                 alc_coeffs[4], alc_coeffs[5], alc_coeffs[7]};
  *egoPathVcc = {ego_coeffs[0],
                 ego_coeffs[1],
                 ego_coeffs[2],
                 ego_coeffs[3],
                 ego_coeffs[4],
                 ego_coeffs[5],
                 ego_coeffs[6],
                 ego_coeffs[7],
                 ego_coeffs[8],
                 3.4f,
                 1};
  return;
}

void LoadDummySSmData(SsmObjType* ssmObjs) {
  // CaseLeftChange(ssmObjs);
  //  CaseSideCarMoveSlowly(ssmObjs);
  CaseFollow(ssmObjs);
  return;
}
