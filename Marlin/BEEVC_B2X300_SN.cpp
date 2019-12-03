//===========================================================================
//========================= B2X300 Printer ==================================
//===========================================================================
// This file stores different configuration for the B2X300, deppending on the printer's SN
/**
 * 
 * The changeable settings by serial number are: 
 * Steps/mm - X Y Z E
 * Trinamic current settings
 * PID settings bed and hotend
 * Servo angles
 * Bed PWM max
 * 
 */
#include "BEEVC_B2X300_SN.h"

// Only valid for B2X300
#ifdef BEEVC_B2X300

    #define PIDReturn(character, valP, valI, valD) \
        if (character == 'P') return valP;         \
        if (character == 'I') return valI;         \
        if (character == 'D') return valD;         

    /**
     * getSteps: return the steps/mm for a given axis and serial number
     *
     * @param {AxisEnum}    axis        The selected axis: X_AXIS, Y_AXIS, Z_AXIS, E_AXIS
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {float}      steps       The steps/mm for the given axis and SN
     */
    float getSteps(AxisEnum axis, uint32_t serial){
        // Necessary to avoid warnings as serial is not required for X Y Z now
        (void)serial;

        // Returns the correct steps/mm
        if(axis == X_AXIS || axis == Y_AXIS)
            return 80.0;
        else if (axis == Z_AXIS)
            return 1600.0;
        else if (axis >= E_AXIS){
            // Pre- Release
            if (serial < BEEVC_B2X300_PROD1_SN_START) //PRE- RELEASE
                return 100;
            // Release 1
            else
                return 101.02;
        }

        // Should not reach this
        return 0;
    }

    /**
     * getTrinamicCurrent: return the default current for a given axis and serial number
     *
     * @param {AxisEnum}    axis        The selected axis: X_AXIS, Y_AXIS, Z_AXIS, E_AXIS
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {uint16_t}   current     The steps/mm for the given axis and SN
     */
    uint16_t getTrinamicCurrent(AxisEnum axis, uint32_t serial){
        // Necessary to avoid warnings as serial is not required for now
        (void)serial;

        // Returns the correct current
        if(axis == X_AXIS || axis == Y_AXIS)
            return 800;
        else if (axis == Z_AXIS)
            return 925;
        else if (axis == E_AXIS)
            return 1100;

        // Should not reach this
        return 0;
    }

    /**
     * getPID: return the default PID settings for a given axis and serial number
     *
     * @param {uint8_t}     hotend      0 for bed, 1 and 2 for extruders
     * @param {uint32_t}    serial      The serial number of the requested data
     * @param {char}        info        Requested data from PID, char can be P I or D
     * 
     * @return {float}      data        Returns the requested data
     * 
     */
    float getPID(uint8_t hotend, uint32_t serial, char info){
        // Necessary to avoid warnings as serial is not required for extruder for now
        (void)serial;

        // BED
        if (hotend == 0){
            // Handle exceptions first 
            if (serial == 1212400076)
                PIDReturn(info,607.85,83.37,1107.97)

            // Pre release
            if (serial < BEEVC_B2X300_PROD1_SN_START)
                PIDReturn(info,300,5,350)
            // Release prior to series 3
            else if (serial < BEEVC_B2X300_PROD3_SN_START)
                PIDReturn(info,85.49,7.67,238.12)
            // Latest - Series 3 forward
            else 
                PIDReturn(info,607.85,83.37,1107.97)
        }
        // Extruder 1
        if (hotend == 1){
            PIDReturn(info,52.00,10.24,66.04)
        }
        // Extruder 2
        if (hotend == 2){
            PIDReturn(info,52.00,10.24,66.04)
        }

        // Should not reach this
        return 0;
    }

    /**
     * getServoAngles: return the default servo angles
     *
     * @param {bool}        state       The state of the requested data, false for stow true for engage     
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {uint8_t}    angle       Returns the angle for either stow or engage
     */
    uint16_t getServoAngles(bool state, uint32_t serial){
        // Necessary to avoid warnings as serial is not required for now
        (void)serial;

        // Engage
        if(state)   return 10;
        // Stow
        else        return 85;
    }

    /**
     * getBedPWM: return the default servo angles
     *
     *  @param {uint32_t}    serial      The serial number of the requested data
     * 
     *  @return {uint8_t}    PWM_max     Returns the max pwm value for bed
     */
    uint16_t getBedPWM(uint32_t serial){
        // Handle exceptions first
        if (serial == 1212400076)
            return 255;
        
        // Pre release
        if (serial < BEEVC_B2X300_PROD1_SN_START)
            return 255;
        // Release prior to series 3
        else if (serial < BEEVC_B2X300_PROD3_SN_START)
            return 205;
        // Latest - Series 3 forward
        else 
            return 255;
    }

    /**
     * validateSerial: checks if a serial is valid
     *
     *  @param {uint32_t}    serial      The serial number of the requested data
     * 
     *  @return {bool}       result      True if valid
     */
    bool validateSerial(uint32_t serial){
        
        // Check if within BETA units
        if (serial >= BEEVC_B2X300_BETA_SN_START && serial <= BEEVC_B2X300_BETA_SN_END)
            return true;

        // Check if within first internal units
        if (serial >= BEEVC_B2X300_INTERNAL1_SN_START && serial <= BEEVC_B2X300_INTERNAL1_SN_END)
            return true;
    
        // Check if within second internal units
        if (serial >= BEEVC_B2X300_INTERNAL2_SN_START && serial <= BEEVC_B2X300_INTERNAL2_SN_END)
            return true;

        // Check if within first production
        if (serial >= BEEVC_B2X300_PROD1_SN_START && serial <= BEEVC_B2X300_PROD1_SN_END)
            return true;

        // Check if within second production
        if (serial >= BEEVC_B2X300_PROD2_SN_START && serial <= BEEVC_B2X300_PROD2_SN_END)
            return true;

        // Check if within third production
        if (serial >= BEEVC_B2X300_PROD3_SN_START && serial <= BEEVC_B2X300_PROD3_SN_END)
            return true;

        // Check if within fourth production
        if (serial >= BEEVC_B2X300_PROD4_SN_START && serial <= BEEVC_B2X300_PROD4_SN_END)
            return true;

        return false;
    }

#endif //BEEVC_B2X300

