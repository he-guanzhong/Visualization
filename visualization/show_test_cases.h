#ifndef SHOW_TEST_CASES_H_
#define SHOW_TEST_CASES_H_
#include "header/Rte_Type_Spd.h"

void CaseLeftChange(SsmObjType* ssmObjs);

void CaseSideCarMoveSlowly(SsmObjType* ssmObjs);

void CaseFollow(SsmObjType* ssmObjs);

void LoadDummySSmData(SsmObjType* ssmObjs);

void LoadDummyMotionData(float* egoSpd,
                         float* egoAcc,
                         float* spdLmt,
                         int* accMode,
                         int* tauGap,
                         AlcBehavior* alcBehav);

void LoadDummyPathData(float* alc_coeffs,
                       float* alc2_coeffs,
                       EgoPathVcc* ego_coeffs,
                       float* left,
                       float* leftleft,
                       float* right,
                       float* rightright,
                       AlcPathVcc* alcPathVcc,
                       AgsmEnvModel* agsmEnvModel);

#endif  // SHOW_TEST_CASES_H_
