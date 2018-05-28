
//===========================================================================
//========================= B2X300 Printer ==================================
//===========================================================================
// Please select the correct options for your B2X300
//
// B2X300
 #define BEEVC_B2X300
//
//
//===========================================================================
//=========================== Experimental/Beta =============================
//===========================================================================
// Back mounted Y endstop, uncomment this if your Y endstop is next to the Y motor
// #define BEEVC_B2X300_YMINSTOP
//
//
// New spring loaded extruder, black plastic
// #define BEEVC_ReverseE
//
//
// If your helloBEEprusa has Trinamic TMC2208/2100 stepper drivers please uncomment the correct line
//
// -Trinamic TMC2208/2100 on the extruder
// #define BEEVC_TMC2208ext
//
// -Trinamic TMC2208/2100 on all axis
// #define BEEVC_TMC2208all
//
//
// If your B2X300 has Trinamic TMC2130 stepper drivers please uncomment the correct line
//
// -Trinamic TMC2130 on all axis
 #define BEEVC_TMC2130
//
// -Trinamic TMC2130 only on XY
// #define BEEVC_TMC2130XY
//
//
#define BEEVC_SG2_DEBUG_SAMPLES 400
//
//===========================================================================
//============================= B2X300 re-define ============================
//===========================================================================

#define BEEVC_A4988all
#define BEEVC_ReverseX
#define BEEVC_ReverseY
#define BEEVC_Trapezoidal
#define BEEVC_Extendedbed
#define BEEVC_Autolevel
#define BEEVC_Bowden
#define BEEVC_MKS_MINI_12864
#define BEEVC_Restore
#define FILAMENT_RUNOUT_SENSOR
#define FILAMENT_RUNOUT_DUAL
//#define BEEVC_Restore_Move_X
#define BEEVC_Restore_Move_Y



#define BEEVC_SG2_DEBUG_HALF_SAMPLES (BEEVC_SG2_DEBUG_SAMPLES/2)
