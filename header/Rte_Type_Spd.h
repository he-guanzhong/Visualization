/**********************************************************************************************************************
 *  COPYRIGHT
 *  -------------------------------------------------------------------------------------------------------------------
 *  \verbatim
 *
 *                This software is copyright protected and proprietary to Vector
 *Informatik GmbH. Vector Informatik GmbH grants to you only those rights as set
 *out in the license conditions. All other rights remain with Vector Informatik
 *GmbH. \endverbatim
 *  -------------------------------------------------------------------------------------------------------------------
 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *             File:  Rte_Type.h
 *           Config:  CBD2000993.dpa
 *      ECU-Project:  CBD2001029
 *
 *        Generator:  MICROSAR RTE Generator Version 4.24.0
 *                    RTE Core Version 1.24.0
 *          License:  CBD2000993
 *
 *      Description:  Header file containing user defined AUTOSAR types and RTE
 *structures
 *********************************************************************************************************************/
#ifndef RTE_TYPE_SPD_H
#define RTE_TYPE_SPD_H
#include "Platform_Types.h"
// #include "Rte.h"

/* PRQA S 1039 EOF */ /* MD_Rte_1039 */

/**********************************************************************************************************************
 * Data type definitions
 *********************************************************************************************************************/

#define Rte_TypeDef_Boolean
typedef boolean Boolean;

#define Rte_TypeDef_Float
typedef float32 Float;

#define Rte_TypeDef_SInt32
typedef sint32 SInt32;

#define Rte_TypeDef_SInt8
typedef sint8 SInt8;

#define Rte_TypeDef_SSWC_SENSOR_MODE_STATUS
typedef uint8 SSWC_SENSOR_MODE_STATUS;

#define Rte_TypeDef_SSWC_WORKING_STATUS
typedef uint8 SSWC_WORKING_STATUS;

#define Rte_TypeDef_UInt16
typedef uint16 UInt16;

#define Rte_TypeDef_UInt32
typedef uint32 UInt32;

#define Rte_TypeDef_UInt8
typedef uint8 UInt8;

#define Rte_TypeDef_rt_Array_Float_4
typedef Float rt_Array_Float_4[4];

#define Rte_TypeDef_rt_Array_Float_3
typedef Float rt_Array_Float_3[3];

/* AutoLaneChgSts: 0-OFF, 1-Selected, 2-hold ego lane, 3-leaving, 4-in target
                   line, 5-finished, 6-Back to Ego, 8-takeover, 9-popMsgReq
  AutoLaneChgSide: 0-OFF, 1-left, 2-right
  BoundaryType: 0-unknown, 1-solid, 2-dash,
  3 - Double Lane(Near Dashed,Far Solid) 9- DECELERATION_Dashed
  NOAStatus: 4- Activated */
#define Rte_TypeDef_AlcBehavior
typedef struct {
  UInt8 AutoLaneChgSide;
  UInt8 AutoLaneChgSts;
  UInt8 LeftBoundaryType;
  UInt8 RightBoundaryType;
  UInt8 NOAStatus;
  UInt8 NaviPilotIsRamp;
  UInt16 NaviPilot1stRampOnDis;
  UInt16 NaviPilot1stExitDis;
} AlcBehavior;

#define Rte_TypeDef_AlcPathVcc
typedef struct {
  Float ConCoeff;
  Float FirstCoeff;
  Float SecondCoeff;
  Float ThirdCoeff;
  Float FourthCoeff;
  Float FifthCoeff;
  Float ViewRng;
} AlcPathVcc;

#define Rte_TypeDef_AgsmEnvModelPath
typedef struct {
  Float C0;
  Float C1;
  Float C2;
  Float C3_1;
  Float C3_2;
  Float C3_3;
  Float Length1;
  Float Length2;
  Float Length3;
  Float Width;
  UInt8 Valid;
} AgsmEnvModelPath;

#define Rte_TypeDef_AgsmEnvModelLine
typedef struct {
  Float C0;
  Float C1;
  Float C2;
  Float C3;
  Float Start;
  Float End;
} AgsmEnvModelLine;

#define Rte_TypeDef_AgsmEnvModel
typedef struct {
  AgsmEnvModelPath EgoPath;
  AgsmEnvModelPath TarPath;
  AgsmEnvModelLine LH0;
  AgsmEnvModelLine LH1;
  AgsmEnvModelLine LA0;
  AgsmEnvModelLine LA1;
} AgsmEnvModel;

#define Rte_TypeDef_EgoMotionSts
typedef struct {
  Float EgoSpd;
  Float EgoAcc;
  Float SpdLmt;
  UInt8 AccMode;
  UInt8 TauGapSet;
  Float AccCurveSpdLmt;
  Float AccDisRef;
  Float AccFollowAccel;
  Float AccCruiseAccel;
} EgoMotionSts;

#define Rte_TypeDef_RemEhMerge
typedef struct {
  Float NearestMergedist;
  UInt8 NearestMergeptDirForEgo;
  SInt8 Merge_DP_ID;
  UInt8 TotalLaneNumL4C;
  UInt8 EgoPosAtWhichLane;
} RemEhMerge;

#define Rte_TypeDef_EgoPathVcc
typedef struct {
  rt_Array_Float_3 C0;
  rt_Array_Float_3 C1;
  rt_Array_Float_3 C2;
  rt_Array_Float_3 C3;
  rt_Array_Float_3 Len;
  Float Width;
  UInt8 Valid;
} EgoPathVcc;

#define Rte_TypeDef_SsmCurvatureType
typedef struct {
  UInt8 valid_flag;
  Float s;
  Float kappa;
} SsmCurvatureType;

#define Rte_TypeDef_SsmHeaderType
typedef struct {
  UInt32 ts_high;
  UInt32 ts_low;
} SsmHeaderType;

/* ObjType of ME:
 * 0=UNFILLED, 1=CAR, 2=TRUCK, 3=MOTORBIKE, 4=BICYCLE,
 * 5=PEDESTRIAN, 6=GENERAL_OBJECT, 7=ANIMAL,8=UNCERTAIN_VCL, 9=TWO_WHEELER */
#define Rte_TypeDef_SsmObsType
typedef struct {
  SInt32 id;
  UInt8 type;
  UInt8 sub_type;
  Float conf;
  Float pos_x;
  Float pos_y;
  Float pos_z;
  Float pos_yaw;
  Float pos_pitch;
  Float pos_roll;
  Float speed_x;
  Float speed_y;
  Float speed_z;
  Float acc_x;
  Float acc_y;
  Float acc_z;
  Float length;
  Float width;
  Float height;
  UInt8 source;
  UInt8 static_flag;
  UInt8 cut_in_flag;
  UInt8 pre_cut_in_flag;
  UInt8 cipv_flag;
  UInt8 valid_flag;
  SInt8 cut_in_line_index;
  Float cut_in_dist;
  UInt8 lane_index;
  UInt8 motion_state;
  UInt8 age;
  UInt8 meas_state;
  UInt8 lane_behavior;
} SsmObsType;

#define Rte_TypeDef_SsmRefPtType
typedef struct {
  Float s;
  UInt8 valid_flag;
  Float x;
  Float y;
} SsmRefPtType;

#define Rte_TypeDef_SsmSpeedRangeType
typedef struct {
  UInt8 valid_flag;
  Float s;
  Float max_v;
} SsmSpeedRangeType;

#define Rte_TypeDef_StPoint
typedef struct {
  Float t;
  Float s;
  Float v;
  Float a;
} StPoint;

#define Rte_TypeDef_TsObjTyp
typedef struct {
  Float PosnLgt;
  Float PosnLat;
  Float Spd;
  Float A;
  UInt8 ObjType;
  UInt8 MotionType;
  UInt16 Idn;
  Float headingAg;
  UInt8 trackStatus;
  UInt32 timeStamp;
  Float LatSpd;
  Float LatAcc;
  Float ExistPrblty;
  UInt8 SyncCtr;
  UInt8 Risk;
  Float Ttc;
  SInt32 Reserve1;
  SInt32 Reserve2;
} TsObjTyp;

// hgz 9->10
#define Rte_TypeDef_StPoint_Array_10
typedef StPoint StPoint_Array_10[10];

/* SpdPlanEnblSts: 1-takeover, 2-minAccel */
#define Rte_TypeDef_DpSpdPoints
typedef struct {
  Boolean AlcLatCtrlEnbl;
  Boolean AlcLgtCtrlEnbl;
  UInt8 AlcFindGapSts;
  UInt8 SpdPlanEnblSts;
  UInt8 SpdPlanTextInfo;
  UInt16 SpdPlanRollCnt;
  Boolean Reserved_b;
  UInt8 Reserved_u8;
  UInt16 Reserved_u16;
  UInt32 ALCDangerObjFrntId;
  UInt32 ALCDangerObjRearId;
  Float Reserved_f32;
  StPoint_Array_10 speedPoints;
} DpSpdPoints;

#define Rte_TypeDef_DpSpeedPointsTest  // hgz test
typedef struct {
  Boolean AlcLatCtrlEnbl;
  Boolean AlcLgtCtrlEnbl;
  StPoint_Array_10 Points;
} DpSpeedPointsTest;

/* SpdPlanTextInfo: 0x9 迫近前车 0xB即将超车 */
#define Rte_TypeDef_DpSpeedPoints  // hgz
typedef struct {
  Boolean AlcLatCtrlEnbl;
  Boolean AlcLgtCtrlEnbl;
  UInt8 AlcFindGapSts;
  UInt8 SpdPlanEnblSts;
  UInt8 SpdPlanTextInfo;
  UInt16 SpdPlanRollCnt;
  StPoint Point0;
  StPoint Point1;
  StPoint Point2;
  StPoint Point3;
  StPoint Point4;
  StPoint Point5;
  StPoint PointCtrl0;
  StPoint PointCtrl1;
  StPoint PointCtrl2;
  StPoint PointCtrl3;
} DpSpeedPoints;

#define Rte_TypeDef_rt_Array_SsmCurvatureType_20
typedef SsmCurvatureType rt_Array_SsmCurvatureType_20[20];

#define Rte_TypeDef_rt_Array_SsmObsType_12  // hgz 32->12
typedef SsmObsType rt_Array_SsmObsType_12[12];

#define Rte_TypeDef_rt_Array_SsmRefPtType_100
typedef SsmRefPtType rt_Array_SsmRefPtType_100[100];

#define Rte_TypeDef_rt_Array_SsmSpeedRangeType_10
typedef SsmSpeedRangeType rt_Array_SsmSpeedRangeType_10[10];

#define Rte_TypeDef_SeldTarsTyp
typedef struct {
  TsObjTyp CipObj;
  TsObjTyp SipObj;
  TsObjTyp FrntLeObj;
  TsObjTyp FrntRiObj;
  TsObjTyp SecFrntLeObj;
  TsObjTyp SecFrntRiObj;
} SeldTarsTyp;

#define Rte_TypeDef_SsmLaneType
typedef struct {
  UInt8 valid_flag;
  SInt8 index;
  rt_Array_SsmSpeedRangeType_10 speed_ranges_st;
  rt_Array_SsmCurvatureType_20 curvature_st;
  rt_Array_SsmRefPtType_100 ref_line_pts;
} SsmLaneType;

#define Rte_TypeDef_SsmObjType
typedef struct {
  UInt8 obj_num;
  UInt8 source;
  UInt8 is_normal;
  UInt8 is_fusion;
  UInt32 ts_high;
  UInt32 ts_low;
  rt_Array_SsmObsType_12 obj_lists;
} SsmObjType;

/* Exported Global Signals for Dataset Calibration */
extern uint8 KbPASP_SpdPlanSwitch;
extern float32 KfPASP_CtrlPoint_s;

/*
#define Rte_TypeDef_SsmParamType
typedef struct {
  UInt8 valid_flag;
  UInt8 type;
  UInt8 tag;
  Float start_x;
  Float end_x;
  rt_Array_Float_4 coeffs;
} SsmParamType;

#define Rte_TypeDef_rt_Array_SsmLaneType_3
typedef SsmLaneType rt_Array_SsmLaneType_3[3];

#define Rte_TypeDef_rt_Array_SsmParamType_3
typedef SsmParamType rt_Array_SsmParamType_3[3];

#define Rte_TypeDef_SsmTrafficLineType
typedef struct {
  SInt32 index;
  UInt8 valid_flag;
  UInt8 color;
  UInt8 line_source;
  UInt32 id;
  UInt8 make_up;
  UInt32 ts_high;
  UInt32 ts_low;
  UInt8 side_type;
  rt_Array_SsmParamType_3 params;
} SsmTrafficLineType;

#define Rte_TypeDef_rt_Array_SsmTrafficLineType_6
typedef SsmTrafficLineType rt_Array_SsmTrafficLineType_6[6];

#define Rte_TypeDef_SsmFrameType
typedef struct {
  UInt32 seq;
  UInt8 ssm_source;
  SsmHeaderType Ssm_Header_st;
  rt_Array_SsmLaneType_3 Ssm_Lanes_st;
  rt_Array_SsmTrafficLineType_6 Ssm_TrafficLine_st;
  SsmObjType Ssm_Objs_Frame_st;
} SsmFrameType;
*/

#endif /* RTE_TYPE_SPD_H */

/**********************************************************************************************************************
 MISRA 2012 violations and justifications
 *********************************************************************************************************************/

/* module specific MISRA deviations:
   MD_Rte_1039:  MISRA rule: Rule1.2
     Reason:     Same macro and function names are required to meet AUTOSAR
   spec. Risk:       No functional risk. Macro will be undefined before function
   definition. Prevention: Not required.

   MD_Rte_3408:  MISRA rule: Rule8.4
     Reason:     For the purpose of monitoring during calibration or debugging
   it is necessary to use non-static declarations. This is covered in the MISRA
   C compliance section of the Rte specification. Risk:       No functional
   risk. Prevention: Not required.

*/
