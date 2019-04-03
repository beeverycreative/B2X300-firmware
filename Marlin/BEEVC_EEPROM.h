/** 
 * This file stores different defines for the B2X300 EEPROM access
 * 
 * B02 EEPROM map byte address
 *  Adress  Bytes   Type      Description
 *  0       4       float     SN
 *  4       4       float     Z position
 *  8       4       float     X position
 *  12      4       float     Y position
 *  16      1       uint8_t   Active Extruder,Extruder mode, acceleration
 *  17      1       bool      Extruder mode
 *  18      4       uint32_t  Acceleration
 *  22      4       float     E position
 *  26      2       uint16_t  Fan Speed
 *  28      2       uint16_t  E0 temp
 *  30      2       uint16_t  E1 temp
 *  32      2       uint16_t  Bed temp
 *  34      4       uint32_t  Sdcard file adress
 *  38      70                SD File path
 *  108     1       uint8_t   X sensorless homing calibration
 *  109     1       uint8_t   Y sensorless homing calibration
 *  110     1       uint8_t   Startup wizard flag
 *  111     1       uint8_t   Reserved
 *  112     1       uint8_t   Reserved
 *  113     37                Free space
 * 
 */

#ifndef BEEVC_EEPROM_H
#define BEEVC_EEPROM_H
    
    // Variable size defines
    #define SIZE_SN         4
    #define SIZE_Z          4
    #define SIZE_X          4
    #define SIZE_Y          4
    #define SIZE_ACTIVE_EXT 1
    #define SIZE_EXT_MODE   1
    #define SIZE_ACC        4
    #define SIZE_E          4
    #define SIZE_FAN        2
    #define SIZE_T_E0       2
    #define SIZE_T_E1       2
    #define SIZE_T_BED      2
    #define SIZE_SD_POS     4
    #define SIZE_SD_PATH    70
    #define SIZE_X_CAL      1
    #define SIZE_Y_CAL      1
    #define SIZE_W_FLAG     1
    
    // Variable EEPROM address defines
    #define ADDRESS_SN         0
    #define ADDRESS_Z          ADDRESS_SN+ SIZE_SN
    #define ADDRESS_X          ADDRESS_Z+ SIZE_Z 
    #define ADDRESS_Y          ADDRESS_X+ SIZE_X 
    #define ADDRESS_ACTIVE_EXT ADDRESS_Y+ SIZE_Y 
    #define ADDRESS_EXT_MODE   ADDRESS_ACTIVE_EXT + SIZE_ACTIVE_EXT
    #define ADDRESS_ACC        ADDRESS_EXT_MODE + SIZE_EXT_MODE
    #define ADDRESS_E          ADDRESS_ACC+ SIZE_ACC 
    #define ADDRESS_FAN        ADDRESS_E+ SIZE_E 
    #define ADDRESS_T_E0       ADDRESS_FAN+ SIZE_FAN 
    #define ADDRESS_T_E1       ADDRESS_T_E0+ SIZE_T_E0 
    #define ADDRESS_T_BED      ADDRESS_T_E1+ SIZE_T_E1 
    #define ADDRESS_SD_POS     ADDRESS_T_BED+ SIZE_T_BED 
    #define ADDRESS_SD_PATH    ADDRESS_SD_POS+ SIZE_SD_POS
    #define ADDRESS_X_CAL      ADDRESS_SD_PATH+ SIZE_SD_PATH
    #define ADDRESS_Y_CAL      ADDRESS_X_CAL+ SIZE_X_CAL
    #define ADDRESS_W_FLAG     ADDRESS_Y_CAL+ SIZE_Y_CAL

    // NON CRC Version
    // Necessary to write to eeprom
    inline void EEPROM_write(int &pos, const uint8_t *value, uint16_t size) {
        while (size--) {
            uint8_t * const p = (uint8_t * const)pos;
            uint8_t v = *value;
            // EEPROM has only ~100,000 write cycles,
            // so only write bytes that have changed!
            if (v != eeprom_read_byte(p)) {
                eeprom_write_byte(p, v);
                if (eeprom_read_byte(p) != v) {
                    SERIAL_ECHO_START();
                    SERIAL_ECHOLNPGM(MSG_ERR_EEPROM_WRITE);
                    return;
                }
            }
            pos++;
            value++;
        };
    }
    inline void EEPROM_read(int &pos, uint8_t* value, uint16_t size) {
        do {
            uint8_t c = eeprom_read_byte((unsigned char*)pos);
            *value = c;
            pos++;
            value++;
        } while (--size);
    }

    #ifdef SERIAL_DEBUG
        #define SERIAL_DEBUG_WRITE_EEPROM_MESSAGE(name, pointer) {  \
            int address = ADDRESS_##name;                           \
            SERIAL_ECHO("Saved ");                                  \
            SERIAL_ECHO(#name);                                     \
            SERIAL_ECHOPAIR(": ", pointer);                         \
            SERIAL_ECHOLNPAIR(" @ ", address);                      \
        }
            
            
        #define SERIAL_DEBUG_READ_EEPROM_MESSAGE(name, pointer) {   \
            int address = ADDRESS_##name;                           \
            SERIAL_ECHO("Read ");                                   \
            SERIAL_ECHO(#name);                                     \
            SERIAL_ECHOPAIR(": ", pointer);                         \
            SERIAL_ECHOLNPAIR(" @ ", address);                      \
        }
            
    #else
        #define SERIAL_DEBUG_WRITE_EEPROM_MESSAGE(name, pointer)
        #define SERIAL_DEBUG_READ_EEPROM_MESSAGE(name, pointer)
    #endif

    #define BEEVC_READ_EEPROM(name, pointer) {                  \
        int address = ADDRESS_##name;                           \
        EEPROM_read(address, (uint8_t*)&pointer, SIZE_##name);  \
        SERIAL_DEBUG_READ_EEPROM_MESSAGE(name, pointer);        \
    }

    #define BEEVC_WRITE_EEPROM(name, pointer) {                 \
        int address = ADDRESS_##name;                           \
        EEPROM_write(address, (uint8_t*)&pointer, SIZE_##name); \
        SERIAL_DEBUG_WRITE_EEPROM_MESSAGE(name, pointer);       \
    }

    
#endif //BEEVC_EEPROM_H

