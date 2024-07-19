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
                         AlcBehavior* alcBehav);

void LoadDummyPathData(const float* alc_coeffs,
                       const float* ego_coeffs,
                       AlcPathVcc* alcPathVcc,
                       AgsmEnvModelPath* agsmEnvModelPath);

#endif  // SHOW_TEST_CASES_H_