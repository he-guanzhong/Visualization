#include "visualization/show_test_cases.h"

void CaseLeftChange(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 10;
  ssmObjs->obj_lists[0].pos_x = 60;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0.0f;
  ssmObjs->obj_lists[0].speed_x = 20.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;
  ssmObjs->obj_lists[0].type = 1;
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;

  ssmObjs->obj_lists[2].pos_x = 10;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 20.0f;
  ssmObjs->obj_lists[2].speed_y = 0;
  ssmObjs->obj_lists[2].type = 1;
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[3].pos_x = 45;
  ssmObjs->obj_lists[3].pos_y = 3.4f;
  ssmObjs->obj_lists[3].acc_x = 0.0f;
  ssmObjs->obj_lists[3].speed_x = 20.0f;
  ssmObjs->obj_lists[3].speed_y = 0;
  ssmObjs->obj_lists[3].type = 1;
  ssmObjs->obj_lists[3].lane_index = 2;
  ssmObjs->obj_lists[3].valid_flag = TRUE;

  ssmObjs->obj_lists[4].pos_x = -0;
  ssmObjs->obj_lists[4].pos_y = 3.4f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 20.0f;
  ssmObjs->obj_lists[4].speed_y = 0.0f;
  ssmObjs->obj_lists[4].type = 1;
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;

  ssmObjs->obj_lists[5].pos_x = -20;
  ssmObjs->obj_lists[5].pos_y = 3.4f;
  ssmObjs->obj_lists[5].acc_x = 0.0f;
  ssmObjs->obj_lists[5].speed_x = 20.0f;
  ssmObjs->obj_lists[5].speed_y = 0.0f;
  ssmObjs->obj_lists[5].type = 1;
  ssmObjs->obj_lists[5].lane_index = 2;
  ssmObjs->obj_lists[5].valid_flag = TRUE;
}

void CaseSideCarMoveSlowly(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 10;

  ssmObjs->obj_lists[0].pos_x = 60;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0.0f;
  ssmObjs->obj_lists[0].speed_x = 8.0f;
  ssmObjs->obj_lists[0].speed_y = 0.0f;
  ssmObjs->obj_lists[0].type = 1;
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;

  ssmObjs->obj_lists[1].pos_x = -23;
  ssmObjs->obj_lists[1].pos_y = 0;
  ssmObjs->obj_lists[1].acc_x = 0.0f;
  ssmObjs->obj_lists[1].speed_x = 8.0f;
  ssmObjs->obj_lists[1].speed_y = 0.0f;
  ssmObjs->obj_lists[1].type = 0;
  ssmObjs->obj_lists[1].lane_index = 3;
  ssmObjs->obj_lists[1].valid_flag = TRUE;

  ssmObjs->obj_lists[2].pos_x = 40;
  ssmObjs->obj_lists[2].pos_y = 3.4f;
  ssmObjs->obj_lists[2].acc_x = 0.0f;
  ssmObjs->obj_lists[2].speed_x = 8.0f;
  ssmObjs->obj_lists[2].speed_y = 0.0f;
  ssmObjs->obj_lists[2].type = 1;
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;

  ssmObjs->obj_lists[3].pos_x = 80;
  ssmObjs->obj_lists[3].pos_y = 3.4f;
  ssmObjs->obj_lists[3].acc_x = 0.0f;
  ssmObjs->obj_lists[3].speed_x = 8.0f;
  ssmObjs->obj_lists[3].speed_y = 0;
  ssmObjs->obj_lists[3].type = 1;
  ssmObjs->obj_lists[3].lane_index = 2;
  ssmObjs->obj_lists[3].valid_flag = TRUE;

  ssmObjs->obj_lists[4].pos_x = -10;
  ssmObjs->obj_lists[4].pos_y = 3.4f;
  ssmObjs->obj_lists[4].acc_x = 0.0f;
  ssmObjs->obj_lists[4].speed_x = 8.0f;
  ssmObjs->obj_lists[4].speed_y = 0.0f;
  ssmObjs->obj_lists[4].type = 0;
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;

  ssmObjs->obj_lists[5].pos_x = -23;
  ssmObjs->obj_lists[5].pos_y = 3.4f;
  ssmObjs->obj_lists[5].acc_x = 0.0f;
  ssmObjs->obj_lists[5].speed_x = 8.0f;
  ssmObjs->obj_lists[5].speed_y = 0.0f;
  ssmObjs->obj_lists[5].type = 0;
  ssmObjs->obj_lists[5].lane_index = 2;
  ssmObjs->obj_lists[5].valid_flag = TRUE;

  ssmObjs->obj_lists[6].pos_x = 80;
  ssmObjs->obj_lists[6].pos_y = -3.4f;
  ssmObjs->obj_lists[6].acc_x = 0.0f;
  ssmObjs->obj_lists[6].speed_x = 5.0f;
  ssmObjs->obj_lists[6].speed_y = 0.0f;
  ssmObjs->obj_lists[6].type = 1;
  ssmObjs->obj_lists[6].lane_index = 4;
  ssmObjs->obj_lists[6].valid_flag = TRUE;

  ssmObjs->obj_lists[7].pos_x = 40;
  ssmObjs->obj_lists[7].pos_y = -3.4f;
  ssmObjs->obj_lists[7].acc_x = 0.0f;
  ssmObjs->obj_lists[7].speed_x = 6.0f;
  ssmObjs->obj_lists[7].speed_y = -6.8f / 5.0f;
  ssmObjs->obj_lists[7].type = 1;
  ssmObjs->obj_lists[7].lane_index = 4;
  ssmObjs->obj_lists[7].valid_flag = TRUE;

  ssmObjs->obj_lists[8].pos_x = -10;
  ssmObjs->obj_lists[8].pos_y = -3.4f;
  ssmObjs->obj_lists[8].acc_x = 0.0f;
  ssmObjs->obj_lists[8].speed_x = 1;
  ssmObjs->obj_lists[8].speed_y = 0.0f;
  ssmObjs->obj_lists[8].type = 0;
  ssmObjs->obj_lists[8].lane_index = 4;
  ssmObjs->obj_lists[8].valid_flag = TRUE;

  ssmObjs->obj_lists[9].pos_x = -23;
  ssmObjs->obj_lists[9].pos_y = -3.4f;
  ssmObjs->obj_lists[9].acc_x = 0.0f;
  ssmObjs->obj_lists[9].speed_x = 1;
  ssmObjs->obj_lists[9].speed_y = 0.0f;
  ssmObjs->obj_lists[9].type = 0;
  ssmObjs->obj_lists[9].lane_index = 4;
  ssmObjs->obj_lists[9].valid_flag = TRUE;
}

void CaseFollow(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 1;
  ssmObjs->obj_lists[0].pos_x = 30;
  ssmObjs->obj_lists[0].pos_y = 0;
  ssmObjs->obj_lists[0].acc_x = 0;
  ssmObjs->obj_lists[0].speed_x = 15;
  ssmObjs->obj_lists[0].speed_y = 0.0f;
  ssmObjs->obj_lists[0].type = 1;
  ssmObjs->obj_lists[0].lane_index = 3;
  ssmObjs->obj_lists[0].valid_flag = TRUE;
}

void CaseCutIn(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 1;
  ssmObjs->obj_lists[2].pos_x = 15;
  ssmObjs->obj_lists[2].pos_y = 3.4;
  ssmObjs->obj_lists[2].acc_x = 0;
  ssmObjs->obj_lists[2].speed_x = 22;
  ssmObjs->obj_lists[2].speed_y = -3.4f / 10.0f;
  ssmObjs->obj_lists[2].type = 1;
  ssmObjs->obj_lists[2].lane_index = 2;
  ssmObjs->obj_lists[2].valid_flag = TRUE;
}

void CaseRearObs(SsmObjType* ssmObjs) {
  // 0 = IV, 1 = RIV, 2 = NIVL, 3 = NIIVL, 4 = RIVL, 5 = RIIVL
  // 6 = NIVR, 7 = NIIVR, 8 = RIVR, 9 = RIIVR
  ssmObjs->obj_num = 1;
  ssmObjs->obj_lists[4].pos_x = 1;
  ssmObjs->obj_lists[4].pos_y = 1.3;
  ssmObjs->obj_lists[4].acc_x = 0;
  ssmObjs->obj_lists[4].speed_x = 8;
  ssmObjs->obj_lists[4].speed_y = 0;
  ssmObjs->obj_lists[4].type = 0;
  ssmObjs->obj_lists[4].lane_index = 2;
  ssmObjs->obj_lists[4].valid_flag = TRUE;
}

void LoadDummySSmData(SsmObjType* ssmObjs) {
  // CaseLeftChange(ssmObjs);
  // CaseSideCarMoveSlowly(ssmObjs);
  // CaseFollow(ssmObjs);
  // CaseCutIn(ssmObjs);
  CaseRearObs(ssmObjs);
  return;
}

void LoadDummyMotionData(float* egoSpd,
                         float* egoAcc,
                         float* spdLmt,
                         int* accMode,
                         AlcBehavior* alcBehav) {
  *egoSpd = 8.0f, *egoAcc = 0, *spdLmt = 40.0f;
  *accMode = 5;
  alcBehav->AutoLaneChgSide = 1;
  alcBehav->AutoLaneChgSts = 1;
  alcBehav->LeftBoundaryType = 2;
  alcBehav->RightBoundaryType = 2;
  return;
}

void LoadDummyPathData(float* alc_coeffs,
                       EgoPathVcc* ego_coeffs,
                       float* left,
                       float* leftleft,
                       float* right,
                       float* rightright,
                       AlcPathVcc* alcPathVcc,
                       AgsmEnvModelPath* agsmEnvModelPath) {
  // left corner
  /*   alc_coeffs[0] = 0, alc_coeffs[1] = 0.01f, alc_coeffs[2] = 1.81E-05f,
    alc_coeffs[3] = 1.1E-05f;

    ego_coeffs->C1[0] = left[1] = right[1] = leftleft[1] = rightright[1] =
    0.01f; ego_coeffs->C2[0] = left[2] = right[2] = leftleft[2] = rightright[2]
    = 1.81E-05f; ego_coeffs->C3[0] = left[3] = right[3] = leftleft[3] =
    rightright[3] = 1.1E-05f; */
  /* 1.036868453	-0.03775819	0.000235738	-2.47E-05 */

  *alcPathVcc = {alc_coeffs[0], alc_coeffs[1], alc_coeffs[2], alc_coeffs[3],
                 alc_coeffs[4], alc_coeffs[5], alc_coeffs[7]};
  agsmEnvModelPath->C0 = ego_coeffs->C0[0];
  agsmEnvModelPath->C1 = ego_coeffs->C1[0];
  agsmEnvModelPath->C2 = ego_coeffs->C2[0];
  agsmEnvModelPath->C3_1 = ego_coeffs->C3[0];
  agsmEnvModelPath->C3_2 = ego_coeffs->C3[1];
  agsmEnvModelPath->C3_3 = ego_coeffs->C3[2];
  agsmEnvModelPath->Length1 = ego_coeffs->Len[0];
  agsmEnvModelPath->Length2 = ego_coeffs->Len[1];
  agsmEnvModelPath->Length3 = ego_coeffs->Len[2];
  agsmEnvModelPath->Width = ego_coeffs->Width;
  agsmEnvModelPath->Valid = ego_coeffs->Valid;
  return;
}
