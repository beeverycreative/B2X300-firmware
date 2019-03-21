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

/**
 * Version history
 *      SN              |   Description
 * -------------------------------------------------
 * < 1212300000         |   Beta testing
 * 1212300000-1212400000|   First production series
 * 1212400000-1212500000|   Second production series
 * 
 * Revision history
 *      SN              |   Changes
 * -------------------------------------------------
 * < 1212300000         |   None
 * 1212300000-1212400000|   New extruder hobbed gear, release bed
 * 1212400000-1212500000|   None
 */

#include "enum.h"
#include "Configuration.h"

#ifndef BEEVC_B2X300_SN_H
#define BEEVC_B2X300_SN_H

// Only valid for B2X300
#ifdef BEEVC_B2X300


    #define BEEVC_B2X300_BETA_SN        1212000001
    #define BEEVC_B2X300_INTERNAL1_SN   1212100001
    #define BEEVC_B2X300_INTERNAL2_SN   1212200001
    #define BEEVC_B2X300_PROD1_SN       1212300001   
    #define BEEVC_B2X300_PROD2_SN       1212400051 
    #define BEEVC_B2X300_LATEST_SN BEEVC_B2X300_PROD2_SN

    /**
     * getSteps: return the steps/mm for a given axis and serial number
     *
     * @param {AxisEnum}    axis        The selected axis: X_AXIS, Y_AXIS, Z_AXIS, E_AXIS
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {float}      steps       The steps/mm for the given axis and SN
     */
    float getSteps(AxisEnum axis, uint32_t serial);

    /**
     * getTrinamicCurrent: return the default current for a given axis and serial number
     *
     * @param {AxisEnum}    axis        The selected axis: X_AXIS, Y_AXIS, Z_AXIS, E_AXIS
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {uint16_t}   current     The steps/mm for the given axis and SN
     */
    uint16_t getTrinamicCurrent(AxisEnum axis, uint32_t serial);

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
    float getPID(uint8_t hotend, uint32_t serial, char info);

    /**
     * getServoAngles: return the default servo angles
     *
     * @param {bool}        state       The state of the requested data, false for stow true for engage     
     * @param {uint32_t}    serial      The serial number of the requested data
     * 
     * @return {uint8_t}    angle       Returns the angle for either stow or engage
     */
    uint16_t getServoAngles(bool state, uint32_t serial);

    /**
     * getBedPWM: return the default servo angles
     *
     *  @param {uint32_t}    serial      The serial number of the requested data
     * 
     *  @return {uint8_t}    PWM_max     Returns the max pwm value for bed
     */
    uint16_t getBedPWM(uint32_t serial);

    /**
     * validateSerial: checks if a serial is valid
     *
     *  @param {uint32_t}    serial      The serial number of the requested data
     * 
     *  @return {bool}       result      True if valid
     */
    bool validateSerial(uint32_t serial);

#endif //BEEVC_B2X300

#endif //BEEVC_B2X300_SN_H

