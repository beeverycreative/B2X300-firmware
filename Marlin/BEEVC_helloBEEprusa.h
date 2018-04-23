
//===========================================================================
//========================= helloBEEprusa Printer ===========================
//===========================================================================
// Please select the correct options for your helloBEEprusa
// To activate an option please uncomment the correct line, delete the // before the #define.
//
//
//
// If your helloBEEprusa has auto bed leveling please uncomment the following line.
// #define BEEVC_Autolevel
//
//
//
// If your helloBEEprusa has Allegro A4988 stepper drivers (Recognizable by the green colored PCB instead of purple)
//
// -Allegro A4988 on the Extruders and DRV8825 on the X, Y and Z axis
// #define BEEVC_A4988ext
//
// -Allegro A4988 on all axis
// #define BEEVC_A4988all
//
//
//
//===========================================================================
//=========================== Experimental/Beta =============================
//===========================================================================
//
//
//
// If your helloBEEprusa has the extended bed please uncomment the following line.
// #define BEEVC_Extendedbed
//
//
//
// If your helloBEEprusa has bowden extruders please uncomment the following line.
// #define BEEVC_Bowden
//
//
//
// If your helloBEEprusa has trapezoidal Z threaded rods please uncomment the following line.
// #define BEEVC_Trapezoidal
//
//
//
// Bowden with smaller PTFE tube, less 100mm than the original kit size
// #define BEEVC_Bowden_500
//
//
//
// Center pulled X carriage
// #define BEEVC_ReverseX
//
//
//
// Center pulled Y carriage
// #define BEEVC_ReverseY
//
//
//
// New spring loaded extruder, black plastic
// #define BEEVC_ReverseE
//
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
//
// Sets the screen as MKS_MINI_12864 regardless of the other options
// #define BEEVC_MKS_MINI_12864
//
//
//
//===========================================================================
//===========================================================================

// The include needs to be here to avoid define problems
#include "BEEVC.h"
