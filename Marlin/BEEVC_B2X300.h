
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
// Pre-Release (BETA) extruder drive gears and hotbed
// #define B2X300_PRERELEASE
//
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
//#define BEEVC_SG2_DEBUG_STEPPER_X
//#define BEEVC_SG2_DEBUG_STEPPER_Y
//#define BEEVC_SG2_DEBUG_STEPPER_E
//


//===========================================================================
//============================= B2X300 settings ============================
//===========================================================================
#define BEEVC_COLD_PULL_SPEED                 120
#define BEEVC_COLD_PULL_LENGTH                -100

// defining the following will override automatic values
//#define BEEVC_FILAMENT_CHANGE_UNLOAD_LENGTH   800
//#define BEEVC_FILAMENT_CHANGE_LOAD_FEEDRATE   60
//#define BEEVC_FILAMENT_CHANGE_LOAD_LENGTH     560
//#define BEEVC_ADVANCED_PAUSE_EXTRUDE_FEEDRATE 4
//#define BEEVC_ADVANCED_PAUSE_EXTRUDE_LENGTH   100


//===========================================================================
//============================= B2X300 re-define ============================
//===========================================================================

#define BEEVC_A4988all
#define BEEVC_ReverseX
#define BEEVC_ReverseY
#define BEEVC_Trapezoidal
//#define BEEVC_Addon_bed
#define BEEVC_Autolevel
#define BEEVC_Bowden
#define BEEVC_MKS_MINI_12864
#define BEEVC_Restore
//#define SERIAL_DEBUG
#define FILAMENT_RUNOUT_SENSOR
#define FILAMENT_RUNOUT_DUAL
//#define BEEVC_Restore_Move_X
#define BEEVC_Restore_Move_Y

// Drive gear revisions
#define B2X300_PRE_PRODUCTION_GEAR_STEPS 100
#define B2X300_PRODUCTION_GEAR_STEPS 101.02

// Fan speed Minimum
#define B2X300_MIN_FAN  140

#ifdef B2X300_PRERELEASE
  #define B2X300_E_STEPS B2X300_PRE_PRODUCTION_GEAR_STEPS
#else
  #define B2X300_E_STEPS B2X300_PRODUCTION_GEAR_STEPS
#endif

#if (ENABLED(BEEVC_TMC2130) || ENABLED(BEEVC_TMC2130XY))
  #define BEEVC_TMC2130READSG
  #define BEEVC_TMC2130HOMEXREVERSE
  #define BEEVC_TMC2130SGFILTER

  #define BEEVC_HOMEXCURRENT  500
  #define BEEVC_HOMEYCURRENT  500

  // #ifdef BEEVC_TMC2130SGFILTER
  //   #define BEEVC_TMC2130HOMESGTX      5
  //   #define BEEVC_TMC2130HOMESGTY      6
  //   #define BEEVC_TMC2130STEPLOSSSGT   5
  // #else
  //   #define BEEVC_TMC2130HOMESGTX      9
  //   #define BEEVC_TMC2130HOMESGTY      9
  //   #define BEEVC_TMC2130STEPLOSSSGT   9
  // #endif

  //#define BEEVC_TMC2130STEPLOSS
  //#define BEEVC_TMC2130RUNOUT
  //#define BEEVC_TMC2130SGDEBUG
#endif

#if ENABLED(BEEVC_SG2_DEBUG_STEPPER_X) || ENABLED(BEEVC_SG2_DEBUG_STEPPER_Y) || ENABLED(BEEVC_SG2_DEBUG_STEPPER_E)
  #define BEEVC_SG2_DEBUG_HALF_SAMPLES (BEEVC_SG2_DEBUG_SAMPLES/2)
  #define BEEVC_SG2_DEBUG_SAMPLES 300
#endif

// Utility defines that allow easy integration of debug messages
#ifdef SERIAL_DEBUG
    #define SERIAL_DEBUG_MESSAGE(str)             SERIAL_PROTOCOLLN(str)
    #define SERIAL_DEBUG_MESSAGE_VALUE(str, val)  SERIAL_PROTOCOLLNPAIR(str, val)
    #define SERIAL_DEBUG_RUN(funct)               funct
  #else
    #define SERIAL_DEBUG_MESSAGE(str) 
    #define SERIAL_DEBUG_MESSAGE_VALUE(str, val)
    #define SERIAL_DEBUG_RUN(funct)
  #endif
