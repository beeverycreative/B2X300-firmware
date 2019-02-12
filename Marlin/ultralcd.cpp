/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "MarlinConfig.h"

#if ENABLED(ULTRA_LCD)

#include "ultralcd.h"
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "planner.h"
#include "stepper.h"
#include "configuration_store.h"
#include "utility.h"
#include "gcode.h"

#if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
  #include "buzzer.h"
#endif

#if ENABLED(PRINTCOUNTER)
  #include "printcounter.h"
  #include "duration_t.h"
#endif

#if ENABLED(BLTOUCH)
  #include "endstops.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "ubl.h"
#elif HAS_ABL
  #include "planner.h"
#elif ENABLED(MESH_BED_LEVELING) && ENABLED(LCD_BED_LEVELING)
  #include "mesh_bed_leveling.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
  bool lcd_external_control; // = false
#endif

// Initialized by settings.load()
int16_t lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];

#if ENABLED(FILAMENT_LCD_DISPLAY) && ENABLED(SDSUPPORT)
  millis_t previous_lcd_status_ms = 0;
#endif

#if ENABLED(BABYSTEPPING)
  long babysteps_done = 0;
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    static void lcd_babystep_zoffset();
  #else
    static void lcd_babystep_z();
  #endif
#endif

uint8_t lcd_status_update_delay = 1, // First update one loop delayed
        lcd_status_message_level;    // Higher level blocks lower level
char lcd_status_message[3 * (LCD_WIDTH) + 1] = WELCOME_MSG; // worst case is kana with up to 3*LCD_WIDTH+1

#if ENABLED(STATUS_MESSAGE_SCROLLING)
  uint8_t status_scroll_pos = 0;
#endif

#if ENABLED(SCROLL_LONG_FILENAMES)
  uint8_t filename_scroll_pos, filename_scroll_max, filename_scroll_hash;
#endif

#if ENABLED(LCD_SET_PROGRESS_MANUALLY)
  uint8_t progress_bar_percent;
#endif

#if ENABLED(DOGLCD)
  #include "ultralcd_impl_DOGM.h"
  #include <U8glib.h>
#else
  #include "ultralcd_impl_HD44780.h"
#endif

#if ENABLED(ULTIPANEL)
  #define DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(_type, _name, _strFunc) \
    inline void lcd_implementation_drawmenu_setting_edit_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type * const data, ...) { \
      UNUSED(pstr2); \
      DRAWMENU_SETTING_EDIT_GENERIC(_strFunc(*(data))); \
    } \
    inline void lcd_implementation_drawmenu_setting_edit_callback_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type * const data, ...) { \
      UNUSED(pstr2); \
      DRAWMENU_SETTING_EDIT_GENERIC(_strFunc(*(data))); \
    } \
    inline void lcd_implementation_drawmenu_setting_edit_accessor_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type (*pget)(), void (*pset)(_type), ...) { \
      UNUSED(pstr2); UNUSED(pset); \
      DRAWMENU_SETTING_EDIT_GENERIC(_strFunc(pget())); \
    } \
    typedef void _name##_void
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(int16_t, int3, itostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(uint8_t, int8, i8tostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float3, ftostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float32, ftostr32);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float43, ftostr43sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float5, ftostr5rj);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float51, ftostr51sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float52, ftostr52sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float62, ftostr62rj);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(uint32_t, long5, ftostr5rj);
  #define lcd_implementation_drawmenu_setting_edit_bool(sel, row, pstr, pstr2, data)                    DRAW_BOOL_SETTING(sel, row, pstr, data)
  #define lcd_implementation_drawmenu_setting_edit_callback_bool(sel, row, pstr, pstr2, data, callback) DRAW_BOOL_SETTING(sel, row, pstr, data)
  #define lcd_implementation_drawmenu_setting_edit_accessor_bool(sel, row, pstr, pstr2, pget, pset)     DRAW_BOOL_SETTING(sel, row, pstr, data)
#endif // ULTIPANEL

// The main status screen
void lcd_status_screen();

millis_t next_lcd_update_ms;

uint8_t lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; // Set when the LCD needs to draw, decrements after every draw. Set to 2 in LCD routines so the LCD gets at least 1 full redraw (first redraw is partial)
uint16_t max_display_update_time = 0;

#if ENABLED(DOGLCD)
  bool drawing_screen = false;
#endif

#if ENABLED(DAC_STEPPER_CURRENT)
  #include "stepper_dac.h" //was dac_mcp4728.h MarlinMain uses stepper dac for the m-codes
  uint8_t driverPercent[XYZE];
#endif

#if ENABLED(ULTIPANEL)

  #ifndef TALL_FONT_CORRECTION
    #define TALL_FONT_CORRECTION 0
  #endif

  // Function pointer to menu functions.
  typedef void (*screenFunc_t)();
  typedef void (*menuAction_t)();

  #if HAS_POWER_SWITCH
    extern bool powersupply_on;
  #endif

  #define _PID_BASE_MENU_ITEMS(ELABEL, eindex) \
    raw_Ki = unscalePID_i(PID_PARAM(Ki, eindex)); \
    raw_Kd = unscalePID_d(PID_PARAM(Kd, eindex)); \
    MENU_ITEM_EDIT(float52, MSG_PID_P ELABEL, &PID_PARAM(Kp, eindex), 1, 9990); \
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I ELABEL, &raw_Ki, 0.01, 9990, copy_and_scalePID_i_E ## eindex); \
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D ELABEL, &raw_Kd, 1, 9990, copy_and_scalePID_d_E ## eindex)

  #define _PID_MENU_ITEMS(ELABEL, eindex) _PID_BASE_MENU_ITEMS(ELABEL, eindex)

  #if ENABLED(PID_AUTOTUNE_MENU)
    #define PID_MENU_ITEMS(ELABEL, eindex) \
      _PID_MENU_ITEMS(ELABEL, eindex); \
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE ELABEL, &autotune_temp[eindex], 150, heater_maxtemp[eindex] - 15, lcd_autotune_callback_E ## eindex)
  #else
    #define PID_MENU_ITEMS(ELABEL, eindex) _PID_MENU_ITEMS(ELABEL, eindex)
  #endif

  ////////////////////////////////////////////
  ///////////////// Menu Tree ////////////////
  ////////////////////////////////////////////

  // BEEVC
  void beevc_main_menu();
  void beevc_print_menu();
  void beevc_print_settings_menu();
  void beevc_maintenance_menu();
  void beevc_machine_menu();
  void beevc_machine_motion_menu();
  void beevc_machine_motion_offset_menu();
  void beevc_machine_temperature_menu();
  void beevc_about_menu();
  void beevc_recover_screen();
  void beevc_recover_now_screen();

  // void lcd_main_menu();
  // void lcd_tune_menu();
  // void lcd_prepare_menu();
  void lcd_move_menu();
  void _lcd_z_offset_start_bed_homing();
  // void lcd_control_menu();
  // void lcd_control_temperature_menu();
  void lcd_control_temperature_preheat_material1_settings_menu();
  void lcd_control_temperature_preheat_material2_settings_menu();
  void lcd_control_motion_menu();
  void lcd_control_filament_menu();


  #if ENABLED(LCD_INFO_MENU)
    #if ENABLED(PRINTCOUNTER)
      void lcd_info_stats_menu();
    #endif
    void lcd_info_thermistors_menu();
    void lcd_info_board_menu();
    void lcd_info_menu();
  #endif // LCD_INFO_MENU

  #if ENABLED(LED_CONTROL_MENU)
    #include "leds.h"
    void lcd_led_menu();
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void lcd_advanced_pause_toocold_menu();
    void lcd_advanced_pause_option_menu();
    void lcd_advanced_pause_init_message();
    void lcd_advanced_pause_unload_message();
    void lcd_advanced_pause_insert_message();
    void lcd_advanced_pause_load_message();
    void lcd_advanced_pause_heat_nozzle();
    void lcd_advanced_pause_extrude_message();
    void lcd_advanced_pause_resume_message();
	void lcd_enqueue_filament_change();

  #endif // ADVANCED_PAUSE_FEATURE

  //Bed leveling - DR
  #if ENABLED(LCD_BED_LEVELING)
    void _lcd_level_goto_next_point();
    void _lcd_level_bed_done() ;
    void _lcd_level_bed_moving() ;
    void _lcd_level_bed_get_z() ;
    void _lcd_level_goto_next_point() ;
    void _lcd_level_bed_homing_done() ;
    void _lcd_level_bed_homing() ;
    void _lcd_level_bed_continue();
	#endif // LCD_BED_LEVELING

	////////////   Power recovery feature    //////////////
	#ifdef BEEVC_Restore
		void recover_print();
	#endif // BEEVC_Restore
	///////////////////////////////////////////////////////

  ////////////   Self-test Wizard    //////////////
	#ifdef BEEVC_B2X300
		void beevc_machine_setup();
    uint8_t trinamic_ok = 0;
	#endif // BEEVC_Restore
	///////////////////////////////////////////////////////

  ////////////   Auxiliary functions    //////////////
	#ifdef BEEVC_B2X300
    uint8_t beevc_screen_header = 0;
    bool beevc_continue = 0;
    uint32_t next_update =  0;
    float old_hotend_offset = 0;
    bool isX = 0 ;
    bool beevc_screen_constant_update = false;
    long beevc_screen_constant_update_time = 0;
	#endif
	///////////////////////////////////////////////////////

  ////////////   Pause/Filament runout   //////////////
	bool pause_filament_runout = false;
  enum pause {
    pause_init,
    pause_unload,
    pause_wait,
    pause_heatup,
    pause_insert,
    pause_load,
    pause_extrude,
    pause_resume,
    pause_option
  };
  uint8_t pause_status = pause_init;
	///////////////////////////////////////////////////////

  ////////////   Filament Change   //////////////
	uint16_t filament_change_temp = 0;
  uint8_t filament_change_extruder = 0;
  bool filament_change_load = false;
  bool filament_change_manual = false;

  static void lcd_filament_change();
  static void lcd_filament_change_start();
  static void lcd_filament_change_extruder_0();
  static void lcd_filament_change_extruder_1();
  static void lcd_filament_change_choose_temp ();
  static void lcd_filament_change_pla ();
  static void lcd_filament_change_petg ();
  static void lcd_filament_change_abs ();
  static void lcd_filament_change_pc ();
  static void lcd_filament_change_choose_action ();
  static void lcd_filament_change_action_load ();
  static void lcd_filament_change_action_unload ();
  static void lcd_filament_change_unload_load (uint16_t changetemp, bool manual_extrude, bool unload_load);
  static void manual_move_to_current(AxisEnum axis
    #if E_MANUAL > 1
      , int8_t eindex
    #endif
  );
	///////////////////////////////////////////////////////
  // Trinamic menu
  #if HAS_TRINAMIC
    // Variables
    #if ENABLED(X_IS_TMC2130)
      uint32_t trinamic_tempXcurrent = 0;
      uint8_t trinamic_tempXsgt = 0;
    #endif
    #if ENABLED(Y_IS_TMC2130)
      uint32_t trinamic_tempYcurrent = 0;
      uint8_t trinamic_tempYsgt = 0;
    #endif
    #if ENABLED(Z_IS_TMC2130)
      uint32_t trinamic_tempZcurrent = 0;
    #endif
    #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
      uint32_t trinamic_tempEcurrent = 0;
      uint8_t trinamic_tempEsgt = 0;
    #endif

    void beevc_trinamic_settings_menu();
    void lcd_trinamic_current();
    // Only shown if X or Y are TMC2130 otherwise sensorless homing wont be used
    #if (ENABLED(X_IS_TMC2130) || ENABLED(Y_IS_TMC2130))
      void lcd_trinamic_sensorless();
    #endif
    // Only shown if X, Y or E are TMC2130 otherwise stallguard measure aren't useful
    #if (ENABLED(X_IS_TMC2130) || ENABLED(Y_IS_TMC2130) || ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
      void lcd_trinamic_stallguard2();
    #endif
  #endif // HAS_TRINAMIC

  #if HAS_ABL
  	//Calibrate Z offset
  	void _lcd_calibrate_z_offset();
  	void _lcd_menu_z_offset();
  	void _lcd_reset_z_offset();
    bool z_offset_finished = false;
  #endif // HAS_ABL

  #if ENABLED(DAC_STEPPER_CURRENT)
    void dac_driver_commit();
    void dac_driver_getValues();
    void lcd_dac_menu();
    void lcd_dac_write_eeprom();
  #endif // DAC_STEPPER_CURRENT

  #if ENABLED(FWRETRACT)
    void lcd_control_retract_menu();
  #endif // FWRETRACT

  #if ENABLED(DELTA_CALIBRATION_MENU) || ENABLED(DELTA_AUTO_CALIBRATION)
    void lcd_delta_calibrate_menu();
  #endif

  ////////////////////////////////////////////
  //////////// Menu System Actions ///////////
  ////////////////////////////////////////////

  #define menu_action_back(dummy) _menu_action_back()
  void _menu_action_back();
  void menu_action_submenu(screenFunc_t data);
  void menu_action_gcode(const char* pgcode);
  void menu_action_function(menuAction_t data);

  #define DECLARE_MENU_EDIT_TYPE(_type, _name) \
    bool _menu_edit_ ## _name(); \
    void menu_edit_ ## _name(); \
    void menu_edit_callback_ ## _name(); \
    void _menu_action_setting_edit_ ## _name(const char * const pstr, _type* const ptr, const _type minValue, const _type maxValue); \
    void menu_action_setting_edit_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue); \
    void menu_action_setting_edit_callback_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue, const screenFunc_t callback, const bool live=false); \
    typedef void _name##_void

  DECLARE_MENU_EDIT_TYPE(int16_t, int3);
  DECLARE_MENU_EDIT_TYPE(uint8_t, int8);
  DECLARE_MENU_EDIT_TYPE(float, float3);
  DECLARE_MENU_EDIT_TYPE(float, float32);
  DECLARE_MENU_EDIT_TYPE(float, float43);
  DECLARE_MENU_EDIT_TYPE(float, float5);
  DECLARE_MENU_EDIT_TYPE(float, float51);
  DECLARE_MENU_EDIT_TYPE(float, float52);
  DECLARE_MENU_EDIT_TYPE(float, float62);
  DECLARE_MENU_EDIT_TYPE(uint32_t, long5);

  void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
  void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callbackFunc);

  #if ENABLED(SDSUPPORT)
    void lcd_sdcard_menu();
    void menu_action_sdfile(const char* filename, char* longFilename);
    void menu_action_sddirectory(const char* filename, char* longFilename);
  #endif // SDSUPPORT

  ////////////////////////////////////////////
  //////////// Menu System Macros ////////////
  ////////////////////////////////////////////

  /**
   * MENU_ITEM generates draw & handler code for a menu item, potentially calling:
   *
   *   lcd_implementation_drawmenu_[type](sel, row, label, arg3...)
   *   menu_action_[type](arg3...)
   *
   * Examples:
   *   MENU_ITEM(back, MSG_WATCH, 0 [dummy parameter] )
   *   or
   *   MENU_BACK(MSG_WATCH)
   *     lcd_implementation_drawmenu_back(sel, row, PSTR(MSG_WATCH))
   *     menu_action_back()
   *
   *   MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause)
   *     lcd_implementation_drawmenu_function(sel, row, PSTR(MSG_PAUSE_PRINT), lcd_sdcard_pause)
   *     menu_action_function(lcd_sdcard_pause)
   *
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     lcd_implementation_drawmenu_setting_edit_int3(sel, row, PSTR(MSG_SPEED), PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   */
  #define _MENU_ITEM_PART_1(TYPE, ...) \
    if (_menuLineNr == _thisItemNr) { \
      if (lcd_clicked && encoderLine == _thisItemNr) {

  #define _MENU_ITEM_PART_2(TYPE, LABEL, ...) \
        menu_action_ ## TYPE(__VA_ARGS__); \
        if (screen_changed) return; \
      } \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_ ## TYPE(encoderLine == _thisItemNr, _lcdLineNr, PSTR(LABEL), ## __VA_ARGS__); \
    } \
    ++_thisItemNr

  #define MENU_ITEM(TYPE, LABEL, ...) do { \
      _skipStatic = false; \
      _MENU_ITEM_PART_1(TYPE, ## __VA_ARGS__); \
      _MENU_ITEM_PART_2(TYPE, LABEL, ## __VA_ARGS__); \
    }while(0)

  #define MENU_ITEM_MIX(TYPE, LABEL, ...) do { \
      _MENU_ITEM_PART_1(TYPE, ## __VA_ARGS__); \
      _MENU_ITEM_PART_2(TYPE, LABEL, ## __VA_ARGS__); \
    }while(0)

  #define MENU_BACK(LABEL) MENU_ITEM(back, _UxGT("Back"), 0)

  // Used to print static text with no visible cursor.
  // Parameters: label [, bool center [, bool invert [, char *value] ] ]
  #define STATIC_ITEM(LABEL, ...) \
    if (_menuLineNr == _thisItemNr) { \
      if (_skipStatic && encoderLine <= _thisItemNr) { \
        encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
        ++encoderLine; \
      } \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_static(_lcdLineNr, PSTR(LABEL), ## __VA_ARGS__); \
    } \
    ++_thisItemNr

  #if ENABLED(ENCODER_RATE_MULTIPLIER)

    bool encoderRateMultiplierEnabled;
    #define ENCODER_RATE_MULTIPLY(F) (encoderRateMultiplierEnabled = F)

    //#define ENCODER_RATE_MULTIPLIER_DEBUG  // If defined, output the encoder steps per second value

    /**
     * MENU_MULTIPLIER_ITEM generates drawing and handling code for a multiplier menu item
     */
    #define MENU_MULTIPLIER_ITEM(type, label, ...) do { \
        _MENU_ITEM_PART_1(type, ## __VA_ARGS__); \
        encoderRateMultiplierEnabled = true; \
        lastEncoderMovementMillis = 0; \
        _MENU_ITEM_PART_2(type, label, ## __VA_ARGS__); \
      }while(0)

  #else // !ENCODER_RATE_MULTIPLIER
    #define ENCODER_RATE_MULTIPLY(F) NOOP
  #endif // !ENCODER_RATE_MULTIPLIER

  #define MENU_ITEM_DUMMY() do { _thisItemNr++; }while(0)
  #define MENU_ITEM_EDIT(type, label, ...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #define MENU_ITEM_EDIT_CALLBACK(type, label, ...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, ...) MENU_MULTIPLIER_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, ...) MENU_MULTIPLIER_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #else // !ENCODER_RATE_MULTIPLIER
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, ...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, ...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #endif // !ENCODER_RATE_MULTIPLIER

  /**
   * START_SCREEN_OR_MENU generates init code for a screen or menu
   *
   *   encoderLine is the position based on the encoder
   *   encoderTopLine is the top menu line to display
   *   _lcdLineNr is the index of the LCD line (e.g., 0-3)
   *   _menuLineNr is the menu item to draw and process
   *   _thisItemNr is the index of each MENU_ITEM or STATIC_ITEM
   *   _countedItems is the total number of items in the menu (after one call)
   */
  #define START_SCREEN_OR_MENU(LIMIT) \
    ENCODER_DIRECTION_MENUS(); \
    ENCODER_RATE_MULTIPLY(false); \
    if (encoderPosition > 0x8000) encoderPosition = 0; \
    static int8_t _countedItems = 0; \
    int8_t encoderLine = encoderPosition / (ENCODER_STEPS_PER_MENU_ITEM); \
    if (_countedItems > 0 && encoderLine >= _countedItems - (LIMIT)) { \
      encoderLine = max(0, _countedItems - (LIMIT)); \
      encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM); \
    }

  #define SCREEN_OR_MENU_LOOP() \
    int8_t _menuLineNr = encoderTopLine, _thisItemNr; \
    for (int8_t _lcdLineNr = 0; _lcdLineNr < LCD_HEIGHT - (TALL_FONT_CORRECTION); _lcdLineNr++, _menuLineNr++) { \
      _thisItemNr = 0

  /**
   * START_SCREEN  Opening code for a screen having only static items.
   *               Do simplified scrolling of the entire screen.
   *
   * START_MENU    Opening code for a screen with menu items.
   *               Scroll as-needed to keep the selected line in view.
   */
  #define START_SCREEN() \
    START_SCREEN_OR_MENU(LCD_HEIGHT - (TALL_FONT_CORRECTION)); \
    encoderTopLine = encoderLine; \
    bool _skipStatic = false; \
    SCREEN_OR_MENU_LOOP()

  #define START_MENU() \
    START_SCREEN_OR_MENU(1); \
    screen_changed = false; \
    NOMORE(encoderTopLine, encoderLine); \
    if (encoderLine >= encoderTopLine + LCD_HEIGHT - (TALL_FONT_CORRECTION)) { \
      encoderTopLine = encoderLine - (LCD_HEIGHT - (TALL_FONT_CORRECTION) - 1); \
    } \
    bool _skipStatic = true; \
    SCREEN_OR_MENU_LOOP()

  #define END_SCREEN() \
    } \
    _countedItems = _thisItemNr

  #define END_MENU() \
    } \
    _countedItems = _thisItemNr; \
    UNUSED(_skipStatic)

  ////////////////////////////////////////////
  ///////////// Global Variables /////////////
  ////////////////////////////////////////////

  /**
   * REVERSE_MENU_DIRECTION
   *
   * To reverse the menu direction we need a general way to reverse
   * the direction of the encoder everywhere. So encoderDirection is
   * added to allow the encoder to go the other way.
   *
   * This behavior is limited to scrolling Menus and SD card listings,
   * and is disabled in other contexts.
   */
  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t encoderDirection = 1;
    #define ENCODER_DIRECTION_NORMAL() (encoderDirection = 1)
    #define ENCODER_DIRECTION_MENUS() (encoderDirection = -1)
  #else
    #define ENCODER_DIRECTION_NORMAL() ;
    #define ENCODER_DIRECTION_MENUS() ;
  #endif

  // Encoder Movement
  volatile int8_t encoderDiff; // Updated in lcd_buttons_update, added to encoderPosition every LCD update
  uint32_t encoderPosition;
  millis_t lastEncoderMovementMillis = 0;

  // Button States
  bool lcd_clicked, wait_for_unclick;
  volatile uint8_t buttons;
  millis_t next_button_update_ms;
  #if ENABLED(REPRAPWORLD_KEYPAD)
    volatile uint8_t buttons_reprapworld_keypad;
  #endif
  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    volatile uint8_t slow_buttons;
  #endif

  // Menu System Navigation
  screenFunc_t currentScreen = lcd_status_screen;
  int8_t encoderTopLine;
  typedef struct {
    screenFunc_t menu_function;
    uint32_t encoder_position;
  } menuPosition;
  menuPosition screen_history[6];
  uint8_t screen_history_depth = 0;
  bool screen_changed, defer_return_to_status;

  // Value Editing
  const char *editLabel;
  void *editValue;
  int32_t minEditValue, maxEditValue;
  screenFunc_t callbackFunc;
  bool liveEdit;

  // Manual Moves
  const float manual_feedrate_mm_m[] = MANUAL_FEEDRATE;
  millis_t manual_move_start_time = 0;
  int8_t manual_move_axis = (int8_t)NO_AXIS;
  #if EXTRUDERS > 1
    int8_t manual_move_e_index = 0;
  #else
    #define manual_move_e_index 0
  #endif

  #if IS_KINEMATIC
    bool processing_manual_move = false;
    float manual_move_offset = 0.0;
  #else
    constexpr bool processing_manual_move = false;
  #endif

  #if PIN_EXISTS(SD_DETECT)
    uint8_t lcd_sd_status;
  #endif

  #if ENABLED(PIDTEMP)
    float raw_Ki, raw_Kd; // place-holders for Ki and Kd edits
  #endif

  /**
   * General function to go directly to a screen
   */
  void lcd_goto_screen(screenFunc_t screen, const uint32_t encoder = 0) {
    if (currentScreen != screen) {

      #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING) && ENABLED(BABYSTEPPING)
        static millis_t doubleclick_expire_ms = 0;
        // Going to lcd_main_menu from status screen? Remember first click time.
        // Going back to status screen within a very short time? Go to Z babystepping.
        if (screen == beevc_main_menu) {
          if (currentScreen == lcd_status_screen)
            doubleclick_expire_ms = millis() + DOUBLECLICK_MAX_INTERVAL;
        }
        else if (screen == lcd_status_screen && currentScreen == beevc_main_menu && PENDING(millis(), doubleclick_expire_ms))
          screen =
            #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
              lcd_babystep_zoffset
            #else
              lcd_babystep_z
            #endif
          ;
      #endif

      currentScreen = screen;
      encoderPosition = encoder;
      if (screen == lcd_status_screen) {
        defer_return_to_status = false;
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          ubl.lcd_map_control = false;
        #endif
        screen_history_depth = 0;
      }
      lcd_implementation_clear();
      // Re-initialize custom characters that may be re-used
      #if DISABLED(DOGLCD) && ENABLED(AUTO_BED_LEVELING_UBL)
        if (!ubl.lcd_map_control) {
          lcd_set_custom_characters(
            #if ENABLED(LCD_PROGRESS_BAR)
              screen == lcd_status_screen ? CHARSET_INFO : CHARSET_MENU
            #endif
          );
        }
      #elif ENABLED(LCD_PROGRESS_BAR)
        lcd_set_custom_characters(screen == lcd_status_screen ? CHARSET_INFO : CHARSET_MENU);
      #endif
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      screen_changed = true;
      #if ENABLED(DOGLCD)
        drawing_screen = false;
      #endif
    }
  }

  /**
   * Show "Moving..." till moves are done, then revert to previous display.
   */
  static const char moving[] PROGMEM = MSG_MOVING;
  static const char *sync_message = moving;

  //
  // Display the synchronize screen until moves are
  // finished, and don't return to the caller until
  // done. ** This blocks the command queue! **
  //
  void _lcd_synchronize() {
    static bool no_reentry = false;
    if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT >= 4 ? 1 : 0, sync_message);
    if (no_reentry) return;
    // Make this the current handler till all moves are done
    no_reentry = true;
    const screenFunc_t old_screen = currentScreen;
    lcd_goto_screen(_lcd_synchronize);
    stepper.synchronize();
    no_reentry = false;
    lcd_goto_screen(old_screen);
  }

  // Display the synchronize screen with a custom message
  // ** This blocks the command queue! **
  void lcd_synchronize(const char * const msg=NULL) {
    sync_message = msg ? msg : moving;
    _lcd_synchronize();
  }

  void lcd_return_to_status() { lcd_goto_screen(lcd_status_screen); }

  void lcd_save_previous_screen() {
    if (screen_history_depth < COUNT(screen_history)) {
      screen_history[screen_history_depth].menu_function = currentScreen;
      screen_history[screen_history_depth].encoder_position = encoderPosition;
      ++screen_history_depth;
    }
  }

  void lcd_goto_previous_menu() {
    if (screen_history_depth > 0) {
      --screen_history_depth;
      lcd_goto_screen(
        screen_history[screen_history_depth].menu_function,
        screen_history[screen_history_depth].encoder_position
      );
    }
    else
      lcd_return_to_status();
  }

  void lcd_goto_previous_menu_no_defer() {
    defer_return_to_status = false;
    lcd_goto_previous_menu();
  }

#endif // ULTIPANEL

/**
 *
 * "Auxiliary function"
 *
 *
 */

 #ifdef BEEVC_B2X300

  #define ACTIVE_FILAMENT_SENSOR_WAITING (((READ(FIL_RUNOUT_PIN2) == FIL_RUNOUT_INVERTING) && (active_extruder == 1)) || ((READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING)  && (active_extruder == 0))) 

  #ifdef SERIAL_DEBUG
    #define SERIAL_DEBUG_MESSAGE(str)             SERIAL_PROTOCOLLN(str)
    #define SERIAL_DEBUG_MESSAGE_VALUE(str, val)  SERIAL_PROTOCOLLNPAIR(str, val)
  #else
    #define SERIAL_DEBUG_MESSAGE(str) 
    #define SERIAL_DEBUG_MESSAGE_VALUE(str, val)
  #endif

   void beevc_buzz(){
     lcd_buzz(100, 659);
     lcd_buzz(100, 698);
   }

   void beevc_wait_click() {
     wait_for_user = true;    // LCD click or M108 will clear this
     while(wait_for_user){
       // Avoid returning to status screen
       defer_return_to_status = true;

       // Manage idle time
       idle(true);
     }
   }

   void beevc_wait(uint16_t milliseconds, bool display_timeout = false) {
     wait_for_user = true;    // LCD click or M108 will clear this
     uint32_t temptime= millis() + milliseconds;
     while((temptime > millis()) && wait_for_user){
       // Avoid returning to status screen
       defer_return_to_status = true;

       // Display remaining time if requested
       if(display_timeout){
        beevc_screen_constant_update = true;
        u8g.setPrintPos(0, 36);
        u8g.print("in ");
        u8g.print((int)((temptime-millis())/1000));
        u8g.print(" seconds");
       }
       
       // Manage idle time
       idle(true);
     }
   }

   void beevc_force_screen_update(){
     unsigned long next_update = millis() + 1000;
     lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
     while(millis()< next_update)
       idle(true);
   }
 #endif

 #define LCD_PRINT_EXT_TEMP() \
    STATIC_ITEM(_UxGT(" ")); \
    u8g.setPrintPos(0, 26);\
    u8g.print("Extruder ");\
    u8g.print(active_extruder+1);\
    u8g.print(": ");\
    u8g.print(itostr3((thermalManager.degHotend(active_extruder))));\
    u8g.print("/");\
    u8g.print(itostr3(thermalManager.degTargetHotend(active_extruder)));\
    lcd_printPGM(PSTR(LCD_STR_DEGREE));\
    u8g.print("C")

  #define LCD_PRINT_EXT_TEMP_STABLE() \
  STATIC_ITEM(_UxGT(" ")); \
  u8g.setPrintPos(0, 26);\
  u8g.print("Extruder ");\
  u8g.print(active_extruder+1);\
  u8g.print(": ");\
  if(round(thermalManager.degTargetHotend(active_extruder)) <100) u8g.print(" ");\
  u8g.print(round(thermalManager.degTargetHotend(active_extruder)));\
  u8g.print("/");\
  u8g.print(round(thermalManager.degTargetHotend(active_extruder)));\
  lcd_printPGM(PSTR(LCD_STR_DEGREE));\
  u8g.print("C")

/**
 *
 * "Dual Nozzle Z offset Assistant"
 *
 *
 */

  void lcd_screen_dual_z_offset_start() {
    START_SCREEN();
    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
    STATIC_ITEM("Test the height");
    STATIC_ITEM("offset between both");
    STATIC_ITEM("nozzles.");
    STATIC_ITEM("(scroll to read more)");
    STATIC_ITEM(" ");
    STATIC_ITEM("This procedure will");
    STATIC_ITEM("help you assert if");
    STATIC_ITEM("the 2 nozzles are at");
    STATIC_ITEM("the same height");
    STATIC_ITEM("relative to the bed.");
    STATIC_ITEM(" ");
    STATIC_ITEM("Please check the");
    STATIC_ITEM("\"Other Codes\" chapter");
    STATIC_ITEM("on the User Manual");
    STATIC_ITEM("for more information.");
    STATIC_ITEM(" ");
    STATIC_ITEM("Proc. code: OC02 ");
    STATIC_ITEM(" ");
    STATIC_ITEM("Click to continue.");
    END_SCREEN();
  }

  void lcd_nozzle_z_offset_hotend_status() {
		START_SCREEN();
      STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
      STATIC_ITEM("  ");
      STATIC_ITEM("  ");
      STATIC_ITEM("Status: heating E1+E2 ");
      #ifdef MSG_FILAMENT_CHANGE_HEATING_2
        STATIC_ITEM("Please wait. ");
      #endif


	  #ifndef DOGLCD
			lcd.setCursor(2, 3);
			lcd.print("Nozzle E1: ");

			if(round(thermalManager.degHotend(0)) <100)
			lcd.print(" ");

			lcd.print(round(thermalManager.degHotend(0)));
			lcd.print("/");
			lcd.print(round(thermalManager.degTargetHotend(0)));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");

      lcd.setCursor(2, 4);
			lcd.print("Nozzle E2: ");

			if(round(thermalManager.degHotend(1)) <100)
			lcd.print(" ");

			lcd.print(round(thermalManager.degHotend(1)));
			lcd.print("/");
			lcd.print(round(thermalManager.degTargetHotend(1)));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");
	  #else
		  u8g.setPrintPos(0, 24);
			u8g.print("Nozzle E1: ");

      if(round(thermalManager.degHotend(0)) <100)
			u8g.print(" ");

			u8g.print(round(thermalManager.degHotend(0)));
			u8g.print("/");
			u8g.print(round(thermalManager.degTargetHotend(0)));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");

      u8g.setPrintPos(0, 36);
			u8g.print("Nozzle E2: ");

      if(round(thermalManager.degHotend(1)) <100)
			u8g.print(" ");

			u8g.print(round(thermalManager.degHotend(1)));
			u8g.print("/");
			u8g.print(round(thermalManager.degTargetHotend(1)));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");
	  #endif

      END_SCREEN();
	}

  void lcd_nozzle_z_offset_bed_status() {
		START_SCREEN();
      STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
      STATIC_ITEM("  ", true, false);
      STATIC_ITEM(_UxGT("Status: heating bed"));
      STATIC_ITEM("  ", true, false);
      #ifdef MSG_FILAMENT_CHANGE_HEATING_2
        STATIC_ITEM("Please wait. ");
      #endif


	  #ifndef DOGLCD
			lcd.setCursor(2, 3);
			lcd.print("Bed: ");

			if(round(thermalManager.degBed()) <100)
			lcd.print(" ");

			lcd.print(round(thermalManager.degBed()));
			lcd.print("/");
			lcd.print(round(thermalManager.degTargetBed()));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");

	  #else
		  u8g.setPrintPos(16, 24);
			u8g.print("Bed: ");

			if(round(thermalManager.degBed()) <100)
			u8g.print(" ");

			u8g.print(round(thermalManager.degBed()));
			u8g.print("/");
			u8g.print(round(thermalManager.degTargetBed()));
      lcd_printPGM(PSTR(LCD_STR_DEGREE));
      u8g.print("C");
	  #endif

      END_SCREEN();
	}

  void lcd_nozzle_z_offset_homing() {
    START_SCREEN();
    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
    STATIC_ITEM(_UxGT("Status: finding mesh"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Please wait."));

    END_SCREEN();
  }

  void lcd_nozzle_z_offset_printing() {
    START_SCREEN();
    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
    STATIC_ITEM(_UxGT("Status: printing"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Please wait."));

    END_SCREEN();
  }

  void lcd_nozzle_z_offset_choose_result_ok() {
    START_SCREEN();
    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
    STATIC_ITEM(_UxGT("Status: successful"));
    STATIC_ITEM(_UxGT("You can now calibrate"));
    STATIC_ITEM(_UxGT("the XY offset. "));
    STATIC_ITEM(_UxGT("Click to exit."));

    END_SCREEN();

    // Sets variable as finished
    beevc_continue = true;
  }

  void lcd_nozzle_z_offset_choose_result_nok() {
    START_SCREEN();
    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);
    STATIC_ITEM(_UxGT("Status: bad offset"));
    STATIC_ITEM(_UxGT("Nozzles are not "));
    STATIC_ITEM(_UxGT("parallel to the bed!"));
    STATIC_ITEM(_UxGT("(scroll to read more)"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Please recalibrate "));
    STATIC_ITEM(_UxGT("according to the"));
    STATIC_ITEM(_UxGT("following procedure!"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Proc. code: OC02"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Click to exit."));

    END_SCREEN();

    // Sets variable as finished
    beevc_continue = true;
  }

  void lcd_nozzle_z_offset_choose_result(){
    START_MENU();

    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);

    STATIC_ITEM(_UxGT("Lines are similar?"), true, false);

    MENU_ITEM(submenu, _UxGT(" - Yes"), lcd_nozzle_z_offset_choose_result_ok);
    MENU_ITEM(submenu, _UxGT(" - No"), lcd_nozzle_z_offset_choose_result_nok);

    END_MENU();
  }

  void lcd_nozzle_z_offset_run () {
    // Shows moving screen
    lcd_goto_screen(lcd_nozzle_z_offset_printing);

    // Force screen update
    beevc_force_screen_update();

    // Run mcode
    gcode_M730();

    // Sets variable as waiting
    beevc_continue = false;

    // Go to question screen
    lcd_goto_screen(lcd_nozzle_z_offset_choose_result);

    // Waits for continue
    while (!beevc_continue){
      idle();
    }

    // Waits for click
    beevc_wait_click();

    // Disable heaters
    thermalManager.disable_all_heaters();

    // Return to status screen
    lcd_return_to_status();

  }

  void lcd_nozzle_z_offset_home_level() {
    // Show finding mesh screen
    lcd_goto_screen(lcd_nozzle_z_offset_homing);

    // Force screen update
    beevc_force_screen_update();

    // Homes and autoleves axes
    gcode_G28(1);
    gcode_G29();

    // Next screen
    lcd_nozzle_z_offset_run();
  }

  void lcd_nozzle_z_offset_wait_for_temp_hotend() {
    // Show "wait for heating" Hotends
    lcd_goto_screen(lcd_nozzle_z_offset_hotend_status);

    // Prepares the temporary variables
    unsigned long next_update = millis() + 200;
    bool update = true;

      while (update) {
         if (next_update < millis()) {
      	   update = false;
      	   idle(true);
      	   HOTEND_LOOP() {
             if ((abs(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > 10) || (abs(thermalManager.degHotend(1) - thermalManager.degTargetHotend(1)) > 10)) {
               update = true;
               break;
             }
      	   }
           // updates the lcd in each cycle
      	   lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      	   // sets next screen update
           next_update = millis() + 100;
          }
      }

      // Show "wait for heating" bed
      lcd_goto_screen(lcd_nozzle_z_offset_bed_status);

      // Force screen update
      beevc_force_screen_update();

      // Prepares the temporary variables
      next_update = millis() + 200;
      update = true;

        while (update) {
           if (next_update < millis()) {
        	   update = false;
        	   idle(true);
        	   HOTEND_LOOP() {
               if ((abs(thermalManager.degBed() - thermalManager.degTargetBed()) > 2)) {
                 update = true;
                 break;
               }
        	   }
             // updates the lcd in each cycle
        	   lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        	   // sets next screen update
             next_update = millis() + 100;
            }
        }

    lcd_nozzle_z_offset_home_level();
  }

  void lcd_nozzle_z_offset_pla(){
    filament_change_temp = 210;
    thermalManager.setTargetHotend(filament_change_temp, 0);
    thermalManager.setTargetHotend(filament_change_temp, 1);
    thermalManager.setTargetBed(50);
    lcd_goto_screen(lcd_nozzle_z_offset_wait_for_temp_hotend);
  }

  void lcd_nozzle_z_offset_petg(){
    filament_change_temp = 230;
    thermalManager.setTargetHotend(filament_change_temp, 0);
    thermalManager.setTargetHotend(filament_change_temp, 1);
    thermalManager.setTargetBed(70);
    lcd_goto_screen(lcd_nozzle_z_offset_wait_for_temp_hotend);
  }

  void lcd_nozzle_z_offset_abs(){
    filament_change_temp = 240;
    thermalManager.setTargetHotend(filament_change_temp, 0);
    thermalManager.setTargetHotend(filament_change_temp, 1);
    thermalManager.setTargetBed(100);
    lcd_goto_screen(lcd_nozzle_z_offset_wait_for_temp_hotend);
  }

  void lcd_nozzle_z_offset_choose_material(){
    START_MENU();

    STATIC_ITEM(_UxGT("Nozzle Z-offset test"), true, true);

    // Go back to previous menu
    MENU_BACK(MSG_BACK);

    MENU_ITEM(submenu, _UxGT("PLA   210ºC"), lcd_nozzle_z_offset_pla);
    MENU_ITEM(submenu, _UxGT("PETG  230ºC"), lcd_nozzle_z_offset_petg);
    MENU_ITEM(submenu, _UxGT("ABS   240ºC"), lcd_nozzle_z_offset_abs);

    END_MENU();
  }

  void lcd_nozzle_z_offset_start(){
    // Shows start screen
    lcd_goto_screen(lcd_screen_dual_z_offset_start);

    // Waits for click
    beevc_wait_click();

    // Shows start screen
    lcd_goto_screen(lcd_nozzle_z_offset_choose_material);
  }


/**
 *
 * "Set nozzle height"
 *
 *
 */
 #ifdef BEEVC_B2X300

 void beevc_set_offset_home_complete(){
   START_SCREEN();
   STATIC_ITEM(_UxGT("Set nozzle height"), true, true);
   STATIC_ITEM(_UxGT("Leveling done!"));
   STATIC_ITEM(_UxGT("Moving to calibration"));
   STATIC_ITEM(_UxGT("position"));
   END_SCREEN();
 }

 void beevc_set_offset_explain(){
   START_SCREEN();
   STATIC_ITEM(_UxGT("Set nozzle height"), true, true);
   STATIC_ITEM(_UxGT("Please, adjust the Z"));
   STATIC_ITEM(_UxGT("height by turning the"));
   STATIC_ITEM(_UxGT("LCD knob."));
   STATIC_ITEM(_UxGT("(scroll to read more)"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Place a paper sheet "));
   STATIC_ITEM(_UxGT("between the nozzle "));
   STATIC_ITEM(_UxGT("and the print surface"));
   STATIC_ITEM(_UxGT(". Adjust until the "));
   STATIC_ITEM(_UxGT("paper is neither free"));
   STATIC_ITEM(_UxGT("nor completely stuck"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Please, check the"));
   STATIC_ITEM(_UxGT("\"Other Codes\" chapter"));
   STATIC_ITEM(_UxGT("on the User Manual "));
   STATIC_ITEM(_UxGT("for more information."));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Proc. code: OC01"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Click to continue."));
   END_SCREEN();
 }

 void beevc_set_offset_homing(){
   START_SCREEN();
   STATIC_ITEM(_UxGT("Set nozzle height"), true, true);
   STATIC_ITEM(_UxGT("Nozzle height: -----"));
   STATIC_ITEM(_UxGT("Status: finding mesh"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Please wait."));
   END_SCREEN();
 }

 void beevc_set_offset_moving(){
   START_SCREEN();
   STATIC_ITEM(_UxGT("Set nozzle height"), true, true);
   STATIC_ITEM(_UxGT("Nozzle height: -----"));
   STATIC_ITEM(_UxGT("Status: moving"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Please wait."));
   END_SCREEN();
 }

 void beevc_set_offset_complete(){
   START_SCREEN();
   STATIC_ITEM(_UxGT("Set nozzle height"), true, true);
   STATIC_ITEM(_UxGT(" "));
   lcd_implementation_drawmenu_setting_edit_generic(false, 1,PSTR("Nozzle height"),ftostr42sign(zprobe_zoffset));
   STATIC_ITEM(_UxGT("Status: OK!"));
   STATIC_ITEM(_UxGT(" "));
   STATIC_ITEM(_UxGT("Click to continue."));
   END_SCREEN();
 }

 void beevc_set_offset_calibrate(){
   if (lcd_clicked)
     {
       zprobe_zoffset = (float)(round((current_position[Z_AXIS] + zprobe_zoffset)*100))/100;
       lcd_completion_feedback(settings.save());
       z_offset_finished = true;

       // Lifts nozzle 5mm, homes XY and moves X carriage 20mm to the left
       enqueue_and_echo_commands_P(PSTR("G91\nG1 Z5\nG28 X Y\nG1 X-20\nG90\nM84"));

       lcd_goto_screen(beevc_set_offset_complete);
     }

   ENCODER_DIRECTION_NORMAL();

   if (encoderPosition) {
     refresh_cmd_timeout();

     float min = current_position[Z_AXIS] - 1000,
           max = current_position[Z_AXIS] + 1000;

     // Get the new position
     current_position[Z_AXIS] -= float((int32_t)encoderPosition) * 0.05;


     // Limit only when trying to move towards the limit
     if ((int32_t)encoderPosition < 0) NOLESS(current_position[Z_AXIS], min);
     if ((int32_t)encoderPosition > 0) NOMORE(current_position[Z_AXIS], max);

     manual_move_to_current(Z_AXIS,0);

     encoderPosition = 0;
     lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
   }
   if (lcdDrawUpdate) {
     START_SCREEN();
     STATIC_ITEM(_UxGT("Set nozzle height"), true, true);

     lcd_implementation_drawmenu_setting_edit_generic(false, 1,PSTR("Nozzle height"),ftostr42sign((float)(round((current_position[Z_AXIS] + zprobe_zoffset)*100))/100));
     lcd_implementation_drawmenu_static(2,PSTR("Status: please adjust"));
     lcd_implementation_drawmenu_static(4,PSTR("Click to save.       "));

     END_SCREEN();
   }
 }

 void beevc_set_offset(){
   // Initializes required variable
   z_offset_finished = false;

   // Show finding mesh screen
   lcd_goto_screen(beevc_set_offset_homing);

   // Waits for click or timeout
   beevc_wait(1000);

   // Homes and autoleves axes
   gcode_G28(1);
   gcode_G29();

   // Show moving screen
   lcd_goto_screen(beevc_set_offset_moving);

   // Moves the carriage and bed to the offset adjust position
   do_blocking_move_to_xy(((X_BED_SIZE) / 2),((Y_BED_SIZE) / 2), 120);

   // Lowers Z axis
   current_position[Z_AXIS] = (float)(round((current_position[Z_AXIS]-8)*10))/10;
   manual_move_to_current(Z_AXIS,0);

   // Waits a few seconds to allow movement to finish
   uint32_t temp_time = millis() + 3200;
   while(millis() < temp_time){
     lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
     idle(true);
   }

   // Shows the help screen
   lcd_goto_screen(beevc_set_offset_explain);
   beevc_wait(20000);

   // Shows the leveling screen
   lcd_goto_screen(beevc_set_offset_calibrate);

   // Waits for conclusion of Adjustment
   while(!z_offset_finished){
     idle(true);
   }

   //Beep
   beevc_buzz();

   //Wait for 5sec or click
   beevc_wait(5000);

   //Return to status screen
   lcd_return_to_status();
 }

 #endif

/**
 *
 * "Info Screen"
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

void lcd_status_screen() {

  // If Self-test wizard flag is set launch it
  if (toCalibrate <= 1){
    beevc_machine_setup();
  }

  // If there is a print to restore and the bed temperature target (previously set when loading the flag)
  // is less than 5 degree away from current bed temperature or 0, starts the recovery on it's own
  if (toRecoverNow){
    beevc_recover_now_screen();
  }

  // If there is a print to restore and the bed temperature is more than 5 degree aways from set temp shows recover screen
  if (toRecover){
    beevc_recover_screen();
  }

  // Ensures the LCD is alive, re-initializing it every 10s
  if (next_update > millis()) {
    lcd_implementation_init();
    next_update = millis() + 10000;
  }

  else{
    #if ENABLED(ULTIPANEL)
      ENCODER_DIRECTION_NORMAL();
      ENCODER_RATE_MULTIPLY(false);
    #endif

    #if ENABLED(LCD_PROGRESS_BAR)

      //
      // HD44780 implements the following message blinking and
      // message expiration because Status Line and Progress Bar
      // share the same line on the display.
      //

      // Set current percentage from SD when actively printing
      #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
        if (IS_SD_PRINTING)
          progress_bar_percent = card.percentDone();
      #endif

      millis_t ms = millis();

      // If the message will blink rather than expire...
      #if DISABLED(PROGRESS_MSG_ONCE)
        if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME))
          progress_bar_ms = ms;
      #endif

      #if PROGRESS_MSG_EXPIRE > 0

        // Handle message expire
        if (expire_status_ms > 0) {

          #if DISABLED(LCD_SET_PROGRESS_MANUALLY)
            const uint8_t progress_bar_percent = card.percentDone();
          #endif

          // Expire the message if a job is active and the bar has ticks
          if (progress_bar_percent > 2 && !print_job_timer.isPaused()) {
            if (ELAPSED(ms, expire_status_ms)) {
              lcd_status_message[0] = '\0';
              expire_status_ms = 0;
            }
          }
          else {
            // Defer message expiration before bar appears
            // and during any pause (not just SD)
            expire_status_ms += LCD_UPDATE_INTERVAL;
          }
        }

      #endif // PROGRESS_MSG_EXPIRE

    #endif // LCD_PROGRESS_BAR

    #if ENABLED(ULTIPANEL)

      if (lcd_clicked) {
        #if ENABLED(FILAMENT_LCD_DISPLAY) && ENABLED(SDSUPPORT)
          previous_lcd_status_ms = millis();  // get status message to show up for a while
        #endif
        lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
          #if ENABLED(LCD_PROGRESS_BAR)
            CHARSET_MENU
          #endif
        );
        lcd_goto_screen(beevc_main_menu);
        return;
      }

      #if ENABLED(ULTIPANEL_FEEDMULTIPLY)
        const int16_t new_frm = feedrate_percentage + (int32_t)encoderPosition;
        // Dead zone at 100% feedrate
        if ((feedrate_percentage < 100 && new_frm > 100) || (feedrate_percentage > 100 && new_frm < 100)) {
          feedrate_percentage = 100;
          encoderPosition = 0;
        }
        else if (feedrate_percentage == 100) {
          if ((int32_t)encoderPosition > ENCODER_FEEDRATE_DEADZONE) {
            feedrate_percentage += (int32_t)encoderPosition - (ENCODER_FEEDRATE_DEADZONE);
            encoderPosition = 0;
          }
          else if ((int32_t)encoderPosition < -(ENCODER_FEEDRATE_DEADZONE)) {
            feedrate_percentage += (int32_t)encoderPosition + ENCODER_FEEDRATE_DEADZONE;
            encoderPosition = 0;
          }
        }
        else {
          feedrate_percentage = new_frm;
          encoderPosition = 0;
        }
      #endif // ULTIPANEL_FEEDMULTIPLY

      feedrate_percentage = constrain(feedrate_percentage, 10, 999);

    #endif // ULTIPANEL

    lcd_implementation_status_screen();
  }
}

void lcd_reset_status() { lcd_setstatusPGM(PSTR(""), -1); }

/**
 *
 * draw the kill screen
 *
 */
void kill_screen(const char* lcd_msg) {
  lcd_init();
  lcd_setalertstatusPGM(lcd_msg);
  lcd_kill_screen();
}

#if ENABLED(ULTIPANEL)

  /**
   *
   * Audio feedback for controller clicks
   *
   */
  void lcd_buzz(const long duration, const uint16_t freq) {
    #if ENABLED(LCD_USE_I2C_BUZZER)
      lcd.buzz(duration, freq);
    #elif PIN_EXISTS(BEEPER)
      buzzer.tone(duration, freq);
    #else
      UNUSED(duration); UNUSED(freq);
    #endif
  }

  void lcd_quick_feedback() {
    lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    buttons = 0;
    next_button_update_ms = millis() + 500;

    // Buzz and wait. The delay is needed for buttons to settle!
    lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
    #if ENABLED(LCD_USE_I2C_BUZZER)
      delay(10);
    #elif PIN_EXISTS(BEEPER)
      for (int8_t i = 5; i--;) { buzzer.tick(); delay(2); }
    #endif
  }

  void lcd_completion_feedback(const bool good/*=true*/) {
    if (good) {
      lcd_buzz(100, 659);
      lcd_buzz(100, 698);
    }
    else lcd_buzz(20, 440);
  }

  inline void line_to_current_z() {
    planner.buffer_line_kinematic(current_position, MMM_TO_MMS(manual_feedrate_mm_m[Z_AXIS]), active_extruder);
  }

  inline void line_to_z(const float &z) {
    current_position[Z_AXIS] = z;
    line_to_current_z();
  }

  #if ENABLED(SDSUPPORT)

    void lcd_sdcard_pause() {
      card.pauseSDPrint();
      print_job_timer.pause();
      #if ENABLED(PARK_HEAD_ON_PAUSE)
        enqueue_and_echo_commands_P(PSTR("M125"));
      #endif
      lcd_setstatusPGM(PSTR(MSG_PRINT_PAUSED), -1);
    }

    void lcd_sdcard_resume() {
      #if ENABLED(PARK_HEAD_ON_PAUSE)
        enqueue_and_echo_commands_P(PSTR("M24"));
      #else
        card.startFileprint();
        print_job_timer.start();
      #endif
      lcd_reset_status();
    }

    void lcd_sdcard_stop() {
      card.stopSDPrint();
      clear_command_queue();
      quickstop_stepper();

      print_job_timer.stop();
      thermalManager.disable_all_heaters();
      #if FAN_COUNT > 0
        for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
      #endif
      wait_for_heatup = false;
	     lcd_setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);
      lcd_return_to_status();

      enqueue_and_echo_commands_P(PSTR("G91\nG1 Z5\nG28 X Y\nG1 X-20\nG90\nM84"));
    }

  #endif // SDSUPPORT

  #if ENABLED(LCD_PROGRESS_BAR_TEST)

    static void progress_bar_test() {
      static int8_t bar_percent = 0;
      if (lcd_clicked) {
        lcd_goto_previous_menu();
        lcd_set_custom_characters(CHARSET_MENU);
        return;
      }
      bar_percent += (int8_t)encoderPosition;
      bar_percent = constrain(bar_percent, 0, 100);
      encoderPosition = 0;
      lcd_implementation_drawmenu_static(0, PSTR(MSG_PROGRESS_BAR_TEST), true, true);
      lcd.setCursor((LCD_WIDTH) / 2 - 2, LCD_HEIGHT - 2);
      lcd.print(itostr3(bar_percent)); lcd.write('%');
      lcd.setCursor(0, LCD_HEIGHT - 1); lcd_draw_progress_bar(bar_percent);
    }

    void _progress_bar_test() {
      lcd_goto_screen(progress_bar_test);
      lcd_set_custom_characters();
    }

  #endif // LCD_PROGRESS_BAR_TEST

  /**
   * BEEVC
   *
   * "Recover" screen
   *
   */
  void beevc_recover_screen_yes(){
    // Enables return to status on timeout
    defer_return_to_status = false;

    // Return to status
    lcd_return_to_status();
    beevc_force_screen_update();

    // Enqueues restore print command
    enqueue_and_echo_commands_P(PSTR("M710"));
    toRecover = false;

    // Allows return to status
    beevc_continue = true ;
  }
  void beevc_recover_screen_no(){
    // Enables return to status on timeout
    defer_return_to_status = false;

    // Clears lcd text
    lcd_setstatus("B2X300 ready.");

    // Disables heating
    thermalManager.disable_all_heaters();

    // Return to status
    lcd_return_to_status();
    beevc_force_screen_update();

    // Clears recover flag
    enqueue_and_echo_commands_P(PSTR("M712"));
    toRecover = false;

    // Allows return to status
    beevc_continue = true ;
  }

  void beevc_recover_screen_display(){
    START_MENU();
    STATIC_ITEM(_UxGT("Powerloss detected"), true, true);
    STATIC_ITEM(_UxGT("Recover print?"));
    MENU_ITEM_MIX(submenu, _UxGT(" - Yes"), beevc_recover_screen_yes);
    MENU_ITEM_MIX(submenu, _UxGT(" - No"), beevc_recover_screen_no);
    END_SCREEN();
  }

  void beevc_recover_screen() {
    // Unset recovery flag
    toRecoverNow = false;
    toRecover = false;

    // Disabes return to status on timeout
    defer_return_to_status = true;

    // Buzz
    beevc_buzz();

    // Set so a loop can be executed while selecting
    beevc_continue = 0;

    // Shows screen
    lcd_goto_screen(beevc_recover_screen_display);

    // Waits for choice
    while(!beevc_continue){
      idle(true);
    }
  }

  void beevc_recover_now_screen_display(){
    START_SCREEN();
    STATIC_ITEM(_UxGT("Powerloss detected"), true, true);
    STATIC_ITEM(_UxGT("Auto-recovering print"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Click to cancel."));
    END_SCREEN();
  }

  void beevc_recover_now_screen() {
    // Unset recovery flag
    toRecoverNow = false;
    toRecover = false;

    // Disabes return to status on timeout
    defer_return_to_status = true;

    // Buzz
    beevc_buzz();

    // Shows screen
    lcd_goto_screen(beevc_recover_now_screen_display);

    // Waits for 10 seconds or click
    beevc_wait(10000,true);

    beevc_screen_constant_update = false;

    // Waits for choice
    if(wait_for_user) {
      // Did not click so restores print
      beevc_recover_screen_yes();
    }
    else {
      // Clicked so restore is canceled
      beevc_recover_screen_no();
    }
  }

  /**
   * BEEVC
   *
   * "Main" menu
   *
   */

  void beevc_main_menu() {
    START_MENU();
    MENU_BACK(MSG_WATCH);

    // Only shows when not printing and still
    if(!(planner.movesplanned() || IS_SD_PRINTING || IS_SD_FILE_OPEN)){
      MENU_ITEM(submenu, _UxGT("Print"), lcd_sdcard_menu);
      MENU_ITEM(submenu, _UxGT("Maintenance"), beevc_maintenance_menu);
    }
    // When printing or moving
    else{
      // If the SD card is readable
      if (card.cardOK) {
        if ((card.isFileOpen())) {
          MENU_ITEM(submenu, _UxGT("Print"), beevc_print_menu);
        }
      }
      // SD card read error
      else {
          MENU_ITEM(gcode, _UxGT("Retry SD card"), PSTR("M21")); // Manually initialize the SD-card via user interface
      }
    }

    MENU_ITEM(submenu, _UxGT("Machine settings"), beevc_machine_menu);
    MENU_ITEM(submenu, _UxGT("About"), beevc_about_menu);
    END_MENU();
  }

  /**
   *  BEEVC
   *
   * "Print" submenu
   *
   */
   void beevc_print_menu(){
     START_MENU();
     MENU_BACK(MSG_WATCH);

     // Pause/Resume/Cancel SD card print
     if (card.isFileOpen()) {
       if (card.sdprinting)
         MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
       else
         MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
      MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
     }

     // Change filament
     // Ensures the hotend isn'too cold
     if (!thermalManager.tooColdToExtrude(active_extruder))
       MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_enqueue_filament_change);

	 // Adjust Print settings
     MENU_ITEM(submenu, _UxGT("Print settings"), beevc_print_settings_menu);

     END_MENU();
   }

   /**
    *  BEEVC
    *
    * "Maintenance" submenu
    *
    */
    void beevc_maintenance_menu(){
      START_MENU();
      MENU_BACK(MSG_WATCH);

      // Change filament
        MENU_ITEM(submenu, MSG_FILAMENTCHANGE, lcd_filament_change_start);
      // Move axis
        MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);
      // Auto Home
        MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
      // Set nozzle height / Z offset
    	  #if HAS_ABL
          #ifdef BEEVC_B2X300
            MENU_ITEM(submenu, _UxGT("Set nozzle height"), beevc_set_offset);
          #else
            MENU_ITEM(submenu, _UxGT("Set nozzle height"), _lcd_z_offset_start_bed_homing);
          #endif
    	  #endif

      // Cold pull
        //MENU_ITEM(submenu, _UxGT("Cold pull"), _lcd_menu_z_offset);

      // Calibrate Dual nozzle Z offset
        MENU_ITEM(submenu, _UxGT("Nozzle Z-offset test"), lcd_nozzle_z_offset_start);

      // Auto Home/ Level Bed
        // Leveling only appears when automatic bed leveling method exists
        #if HAS_ABL
    		  MENU_ITEM(gcode, MSG_LEVEL_BED, PSTR("G29\nG28 X Y\nM500\nG91\nG1 X-20\nG90\nG4 P200\n M300 S4000 P200\nG4 P500\n M300 S4000 P200"));
    	  #endif
        // helloBEEprusa - LCD leveling
    	  #if ENABLED(LCD_BED_LEVELING)
    		  MENU_ITEM(function, MSG_LEVEL_BED, _lcd_level_bed_continue);
    	  #endif

      END_MENU();
    }

    /**
     *  BEEVC
     *
     * "About" submenu
     *
     */
    void beevc_about_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }

      START_SCREEN();
      STATIC_ITEM("Marlin-fork", true, true);
      STATIC_ITEM("open source firmware", true, false);
      STATIC_ITEM("engineered by", true, false);
      STATIC_ITEM("BEEVERYCREATIVE", true, false);

      char about_string[22];
      about_string[0] = '\0';

      strncat(about_string, BUILDDATE,6);
      strcat(about_string, "-");
      strncat(about_string, BUILDBRANCH,6);
      strcat(about_string, "-");
      strncat(about_string, BUILDCOMMIT,7);
      STATIC_ITEM("", true, false, about_string);
      STATIC_ITEM(" ");
      STATIC_ITEM("Diagnostic Info.", false, true);

      about_string[0] = '\0';
      strncat(about_string, i8tostr3(thermalManager.sg2_homing_x_calibration),3);
      strcat(about_string, "/");
      strncat(about_string, i8tostr3(thermalManager.sg2_homing_y_calibration),4);
      STATIC_ITEM("Homing X/Y: ", false, false, about_string);

      STATIC_ITEM(_UxGT("Nozzle height: "), false, false, ftostr42sign(zprobe_zoffset));

      about_string[0] = '\0';
      strncat(about_string, itostr4sign(stepperX.getCurrent()),4);
      strcat(about_string, "/");
      strncat(about_string, itostr4sign(stepperY.getCurrent()),5);
      STATIC_ITEM("Curr. X/Y: ", false, false, about_string);

      about_string[0] = '\0';
      strncat(about_string, itostr4sign(stepperZ.getCurrent()),4);
      strcat(about_string, "/");
      strncat(about_string, itostr4sign(stepperE0.getCurrent()),5);
      STATIC_ITEM("Curr. Z/E: ", false, false, about_string);

      END_SCREEN();
    }



  /**
   *
   * "Tune" submenu items
   *
   */

  #if HAS_M206_COMMAND
    /**
     * Set the home offset based on the current_position
     */
    void lcd_set_home_offsets() {
      // M428 Command
      enqueue_and_echo_commands_P(PSTR("M428"));
      lcd_return_to_status();
    }
  #endif

  #if ENABLED(BABYSTEP_ZPROBE_GFX_OVERLAY) || ENABLED(MESH_EDIT_GFX_OVERLAY)

    void _lcd_zoffset_overlay_gfx(const float zvalue) {
      // Determine whether the user is raising or lowering the nozzle.
      static int8_t dir;
      static float old_zvalue;
      if (zvalue != old_zvalue) {
        dir = zvalue ? zvalue < old_zvalue ? -1 : 1 : 0;
        old_zvalue = zvalue;
      }

      #if ENABLED(OVERLAY_GFX_REVERSE)
        const unsigned char *rot_up = ccw_bmp, *rot_down = cw_bmp;
      #else
        const unsigned char *rot_up = cw_bmp, *rot_down = ccw_bmp;
      #endif

      #if ENABLED(USE_BIG_EDIT_FONT)
        const int left = 0, right = 45, nozzle = 95;
      #else
        const int left = 5, right = 90, nozzle = 60;
      #endif

      // Draw a representation of the nozzle
      if (PAGE_CONTAINS(3, 16))  u8g.drawBitmapP(nozzle + 6, 4 - dir, 2, 12, nozzle_bmp);
      if (PAGE_CONTAINS(20, 20)) u8g.drawBitmapP(nozzle + 0, 20, 3, 1, offset_bedline_bmp);

      // Draw cw/ccw indicator and up/down arrows.
      if (PAGE_CONTAINS(47, 62)) {
        u8g.drawBitmapP(left  + 0, 47, 3, 16, rot_down);
        u8g.drawBitmapP(right + 0, 47, 3, 16, rot_up);
        u8g.drawBitmapP(right + 20, 48 - dir, 2, 13, up_arrow_bmp);
        u8g.drawBitmapP(left  + 20, 49 - dir, 2, 13, down_arrow_bmp);
      }
    }

  #endif // BABYSTEP_ZPROBE_GFX_OVERLAY || MESH_EDIT_GFX_OVERLAY

  void lcd_babystep_zoffset() {
    if (lcd_clicked) {
      // Save the Z offset to the EEPROM
      lcd_completion_feedback(settings.save());

      return lcd_goto_previous_menu_no_defer();
    }
    defer_return_to_status = true;
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      const int16_t babystep_increment = -((int32_t)encoderPosition * (BABYSTEP_MULTIPLICATOR));
      encoderPosition = 0;

      const float new_zoffset = zprobe_zoffset + planner.steps_to_mm[Z_AXIS] * babystep_increment;
      if (WITHIN(new_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {

        if (planner.leveling_active)
          thermalManager.babystep_axis(Z_AXIS, babystep_increment);

        zprobe_zoffset = (float)(round(new_zoffset*100))/100;
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      }
    }
    if (lcdDrawUpdate) {
      START_SCREEN();
      STATIC_ITEM(_UxGT("Live nozzle height"), true, true);

      lcd_implementation_drawmenu_setting_edit_generic(false, 1,PSTR("Nozzle height"),ftostr42sign(zprobe_zoffset));
      lcd_implementation_drawmenu_static(2,PSTR("Status: please adjust"));
      lcd_implementation_drawmenu_static(4,PSTR("Click to save.       "));

      END_SCREEN();
      
      #if ENABLED(BABYSTEP_ZPROBE_GFX_OVERLAY)
        _lcd_zoffset_overlay_gfx(zprobe_zoffset);
      #endif
    }
  }

  #if ENABLED(BABYSTEPPING)

    void _lcd_babystep(const AxisEnum axis, const char* msg) {
      if (lcd_clicked) { return lcd_goto_previous_menu_no_defer(); }
      ENCODER_DIRECTION_NORMAL();
      if (encoderPosition) {
        const int16_t babystep_increment = (int32_t)encoderPosition * (BABYSTEP_MULTIPLICATOR);
        encoderPosition = 0;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        thermalManager.babystep_axis(axis, babystep_increment);
        babysteps_done += babystep_increment;
      }
      if (lcdDrawUpdate)
        lcd_implementation_drawedit(msg, ftostr43sign(planner.steps_to_mm[axis] * babysteps_done));
    }

    #if ENABLED(BABYSTEP_XY)
      void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEP_X)); }
      void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEP_Y)); }
      void lcd_babystep_x() { lcd_goto_screen(_lcd_babystep_x); babysteps_done = 0; defer_return_to_status = true; }
      void lcd_babystep_y() { lcd_goto_screen(_lcd_babystep_y); babysteps_done = 0; defer_return_to_status = true; }
    #endif

      void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEP_Z)); }
      void lcd_babystep_z() { lcd_goto_screen(_lcd_babystep_z); babysteps_done = 0; defer_return_to_status = true; }

  #endif // BABYSTEPPING

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    float mesh_edit_value, mesh_edit_accumulator; // We round mesh_edit_value to 2.5 decimal places. So we keep a
                                                  // separate value that doesn't lose precision.
    static int16_t ubl_encoderPosition = 0;

    static void _lcd_mesh_fine_tune(const char* msg) {
      defer_return_to_status = true;
      if (ubl.encoder_diff) {
        ubl_encoderPosition = (ubl.encoder_diff > 0) ? 1 : -1;
        ubl.encoder_diff = 0;

        mesh_edit_accumulator += float(ubl_encoderPosition) * 0.005 / 2.0;
        mesh_edit_value = mesh_edit_accumulator;
        encoderPosition = 0;
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;

        const int32_t rounded = (int32_t)(mesh_edit_value * 1000.0);
        mesh_edit_value = float(rounded - (rounded % 5L)) / 1000.0;
      }

      if (lcdDrawUpdate) {
        lcd_implementation_drawedit(msg, ftostr43sign(mesh_edit_value));
        #if ENABLED(MESH_EDIT_GFX_OVERLAY)
          _lcd_zoffset_overlay_gfx(mesh_edit_value);
        #endif
      }
    }

    void _lcd_mesh_edit_NOP() {
      defer_return_to_status = true;
    }

    float lcd_mesh_edit() {
      lcd_goto_screen(_lcd_mesh_edit_NOP);
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      _lcd_mesh_fine_tune(PSTR("Mesh Editor"));
      return mesh_edit_value;
    }

    void lcd_mesh_edit_setup(const float initial) {
      mesh_edit_value = mesh_edit_accumulator = initial;
      lcd_goto_screen(_lcd_mesh_edit_NOP);
    }

    void _lcd_z_offset_edit() {
      _lcd_mesh_fine_tune(PSTR("Z-Offset: "));
    }

    float lcd_z_offset_edit() {
      lcd_goto_screen(_lcd_z_offset_edit);
      return mesh_edit_value;
    }

    void lcd_z_offset_edit_setup(float initial) {
      mesh_edit_value = mesh_edit_accumulator = initial;
      lcd_goto_screen(_lcd_z_offset_edit);
    }

  #endif // AUTO_BED_LEVELING_UBL


  /**
   * Watch temperature callbacks
   */
  #if HAS_TEMP_HOTEND
    #if WATCH_HOTENDS
      #define _WATCH_FUNC(N) thermalManager.start_watching_heater(N)
    #else
      #define _WATCH_FUNC(N) NOOP
    #endif
    void watch_temp_callback_E0() { _WATCH_FUNC(0); }
    #if HOTENDS > 1
      void watch_temp_callback_E1() { _WATCH_FUNC(1); }
      #if HOTENDS > 2
        void watch_temp_callback_E2() { _WATCH_FUNC(2); }
        #if HOTENDS > 3
          void watch_temp_callback_E3() { _WATCH_FUNC(3); }
          #if HOTENDS > 4
            void watch_temp_callback_E4() { _WATCH_FUNC(4); }
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1
  #endif // HAS_TEMP_HOTEND

  void watch_temp_callback_bed() {
    #if WATCH_THE_BED
      thermalManager.start_watching_bed();
    #endif
  }

	////////////   Power recovery feature    //////////////
	#ifdef BEEVC_Restore

		void recover_print()
		{
			enqueue_and_echo_commands_P(PSTR("M710"));
			toRecover = false;
			lcd_return_to_status();
		}

	#endif
	///////////////////////////////////////////////////////


  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    // Sensorless homing

    void lcd_sensorless_homing_calibration_x()
      {
        START_SCREEN();
        STATIC_ITEM(_UxGT("Self-test Wizard"), true, true);

        STATIC_ITEM(_UxGT("Sensorless-homing: X"));
        STATIC_ITEM(_UxGT("Noise and impacts are"));
        STATIC_ITEM(_UxGT("expected."));

        if(sensorless_homing_progress == 0) {
          STATIC_ITEM(_UxGT("Working"));
        }
        else if (sensorless_homing_progress == 1){
          STATIC_ITEM(_UxGT("Working."));
        }
        else if (sensorless_homing_progress == 2){
          STATIC_ITEM(_UxGT("Working.."));
        }
        else if (sensorless_homing_progress >= 3){
          STATIC_ITEM(_UxGT("Working..."));
        }

        END_SCREEN();
      }
    void lcd_sensorless_homing_calibration_y()
      {
        START_SCREEN();
        STATIC_ITEM(_UxGT("Self-test Wizard"), true, true);

        STATIC_ITEM(_UxGT("Sensorless-homing: Y"));
        STATIC_ITEM(_UxGT("Noise and impacts are"));
        STATIC_ITEM(_UxGT("expected."));

        if(sensorless_homing_progress == 0) {
          STATIC_ITEM(_UxGT("Working"));
        }
        else if (sensorless_homing_progress == 1){
          STATIC_ITEM(_UxGT("Working."));
        }
        else if (sensorless_homing_progress == 2){
          STATIC_ITEM(_UxGT("Working.."));
        }
        else if (sensorless_homing_progress >= 3){
          STATIC_ITEM(_UxGT("Working..."));
        }

        END_SCREEN();
      }
    void lcd_sensorless_homing_calibration_x_done()
      {
        START_SCREEN();
        STATIC_ITEM(_UxGT("Self-test Wizard"), true, true);
        STATIC_ITEM(_UxGT("Sensorless-homing: X"));
        STATIC_ITEM(_UxGT("Status: OK!"));
        STATIC_ITEM(_UxGT(" "));
        STATIC_ITEM(_UxGT("Please wait."));
        END_SCREEN();
      }
    void lcd_sensorless_homing_calibration_y_done()
      {
        START_SCREEN();
        STATIC_ITEM(_UxGT("Self-test Wizard"), true, true);
        STATIC_ITEM(_UxGT("Sensorless-homing: Y"));
        STATIC_ITEM(_UxGT("Status: OK!"));
        STATIC_ITEM(_UxGT(" "));
        STATIC_ITEM(_UxGT("Please wait."));
        END_SCREEN();
      }
    void lcd_sensorless_homing_calibration_homing()
      {
        START_SCREEN();
        STATIC_ITEM(_UxGT("Self-test Wizard"), true, true);
        STATIC_ITEM(_UxGT("Sensorless-homing: XY"));
        STATIC_ITEM(_UxGT("Status: homing X+Y"));
        STATIC_ITEM(_UxGT(" "));
        STATIC_ITEM(_UxGT("Please wait."));
        END_SCREEN();
      }

    ////////////////////////////////////////////////////////////////////////
    //					Filament Change Feature							//
    ////////////////////////////////////////////////////////////////////////

    void lcd_filament_change_hotendStatus() {
      START_SCREEN();
        STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);
        
      #ifndef DOGLCD
        STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_1 "  ", true, false);
        #ifdef MSG_FILAMENT_CHANGE_HEATING_2
          STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_2 "  ", true, false);
          #define _FC_LINES_C 3
        #else
          #define _FC_LINES_C 2
        #endif

        lcd.setCursor(2, 3);
        lcd.print("Nozzle: ");

        if(round(thermalManager.degHotend(active_extruder)) <100)
        lcd.print(" ");

        lcd.print(round(thermalManager.degHotend(active_extruder)));
        lcd.print("/");
        lcd.print(round(thermalManager.degTargetHotend(active_extruder)));
      #else

        LCD_PRINT_EXT_TEMP();
        STATIC_ITEM(_UxGT("Status:heating nozzle"));
        STATIC_ITEM(_UxGT(" "));
        STATIC_ITEM(_UxGT("Please wait."));
        
      #endif

        END_SCREEN();
    }

    void lcd_filament_change_resume_print() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
    }

    void lcd_filament_change_extrude_more() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
    }

    void lcd_filament_change_load() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_LOAD;
    }

    void lcd_filament_change_press(){
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);

      LCD_PRINT_EXT_TEMP_STABLE();
      STATIC_ITEM(_UxGT("Status:  heating done"));
      STATIC_ITEM(_UxGT(" "));
      STATIC_ITEM(_UxGT("Click to continue."));
      END_SCREEN();
    }

    void lcd_filament_change_press_insert(){
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);

      LCD_PRINT_EXT_TEMP_STABLE();
      STATIC_ITEM(_UxGT("Status:  heating done"));
      //STATIC_ITEM(_UxGT(" "));
      STATIC_ITEM(_UxGT("Click or insert "));
      STATIC_ITEM(_UxGT("filament to continue."));
      END_SCREEN();
    }


    void lcd_filament_change_loading(){
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);

      LCD_PRINT_EXT_TEMP_STABLE();
      STATIC_ITEM(_UxGT("Status: loading"));
      STATIC_ITEM(_UxGT(" "));
      STATIC_ITEM(_UxGT("Please wait."));
      END_SCREEN();
    }

    void lcd_filament_change_unloading(){
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);

      LCD_PRINT_EXT_TEMP_STABLE();
      STATIC_ITEM(_UxGT("Status: unloading"));
      STATIC_ITEM(_UxGT(" "));
      STATIC_ITEM(_UxGT("Please wait."));
      END_SCREEN();
    }

    void lcd_filament_change_moving(){
      // Ensure the correct extruder is set
      if (active_extruder != filament_change_extruder)
        active_extruder = filament_change_extruder;

      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);

      if(round(thermalManager.degHotend(active_extruder)) > (round(thermalManager.degTargetHotend(active_extruder))-10)) {
        LCD_PRINT_EXT_TEMP_STABLE();
      } 
      else {
        LCD_PRINT_EXT_TEMP();
      } 

      STATIC_ITEM(_UxGT("Status: moving"));
      STATIC_ITEM(_UxGT(" "));
      STATIC_ITEM(_UxGT("Please wait."));
      END_SCREEN();
    }

    void lcd_filament_change_option_menu() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);
      #endif
      MENU_ITEM(function, _UxGT("Back"), lcd_filament_change_resume_print);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_EXTRUDE, lcd_filament_change_extrude_more);
      END_MENU();
    }

    void lcd_filament_change_unload_option_menu() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);
      #endif
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_LOAD, lcd_filament_change_load);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_RESUME, lcd_filament_change_resume_print);
      END_MENU();
    }

    void lcd_filament_change_unload_menu() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_FILAMENTCHANGE, true, true);
      #endif

      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_RESUME, lcd_filament_change_resume_print);
      MENU_ITEM(function, _UxGT("Load filament"), lcd_filament_change_extrude_more);
      END_MENU();
    }

    void lcd_filament_change_move_e_screen(){
      START_SCREEN();
      STATIC_ITEM(_UxGT("Manual extrusion"), true, true);
      if(active_extruder == 0){
        STATIC_ITEM("Extruder 1:");
      }
      else{
        STATIC_ITEM("Extruder 2:");
      }

      u8g.setPrintPos(80, 24);
      u8g.print(ftostr41sign(current_position[E_AXIS]));

      if(planner.movesplanned() > 0){
        STATIC_ITEM("Status: moving");
        STATIC_ITEM("");
        STATIC_ITEM("Please wait.");
      } 
      else{
        STATIC_ITEM("Status: standby");
        STATIC_ITEM("");
        STATIC_ITEM("Click to exit.");
      } 
      END_SCREEN();
    }

    void beevc_move_axis(AxisEnum axis,float move_mm, float feed_mms){
      // Sets the motion ammount and executes movement at requested speed
      current_position[axis] += move_mm;
      planner.buffer_line_kinematic(current_position, feed_mms, active_extruder);
    }

    void beevc_move_axis_blocking(AxisEnum axis,float move_mm, float feed_mms){
      // Plans the motion
      beevc_move_axis(axis, move_mm, feed_mms);

      // Waits for movement to finish
      while(planner.movesplanned() > 0) idle();
    }
  
    void lcd_filament_change_move_e() {
        defer_return_to_status = true;
        beevc_screen_constant_update = true;
        ENCODER_DIRECTION_NORMAL();
        if (encoderPosition && (!processing_manual_move)) {
          float diff = float((int32_t)encoderPosition) * 10;
          NOMORE(diff,5);
          NOLESS(diff,-5);

          current_position[E_AXIS] += diff;
          manual_move_to_current(E_AXIS, active_extruder);
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
          encoderPosition = 0;
        }
        if (lcdDrawUpdate) lcd_filament_change_move_e_screen();
    }

    
    void beevc_load_unload( bool load){
      // Forces constant screen updates
      beevc_screen_constant_update = true; 

      // Show moving screen
      if(load)
        lcd_goto_screen(lcd_filament_change_loading);
      else
        lcd_goto_screen(lcd_filament_change_unloading);

      // Ensures the screen has changed before movement
      beevc_force_screen_update();

      // Extrudes a small ammount to fluidify the tip of the filament
      beevc_move_axis_blocking(E_AXIS,15,ADVANCED_PAUSE_EXTRUDE_FEEDRATE);

      // Unload
      if(!load) {
        // Unload filament
        beevc_move_axis_blocking(E_AXIS,-(FILAMENT_CHANGE_UNLOAD_LENGTH),FILAMENT_CHANGE_UNLOAD_FEEDRATE);
      }

      // Load
      if (load)
      {
        // Ensures the screen is updated even when unloading first
        lcd_goto_screen(lcd_filament_change_loading);

        // //Checks if Bowden to apply the correct 3 phase load process
        // //Direct drive
        // #ifndef BEEVC_Bowden
        //   // Load filament
        //   destination[E_AXIS] += FILAMENT_CHANGE_LOAD_LENGTH;
        //   RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
        //   stepper.synchronize();

        // #else

        //Bowden
        // Load filament slowly into PTFE tube
        beevc_move_axis_blocking(E_AXIS,50,ADVANCED_PAUSE_EXTRUDE_FEEDRATE);

        // Load filament quickly into PTFE tube
        beevc_move_axis_blocking(E_AXIS,FILAMENT_CHANGE_LOAD_LENGTH,FILAMENT_CHANGE_LOAD_FEEDRATE);

        // #endif

        // Extrude filament
        do {
          // Show loading
          lcd_goto_screen(lcd_filament_change_loading);

          // Forces screen refresh
          beevc_force_screen_update();

          // Extrude filament to get into hotend
          beevc_move_axis_blocking(E_AXIS,ADVANCED_PAUSE_EXTRUDE_LENGTH,ADVANCED_PAUSE_EXTRUDE_FEEDRATE);

          // Show "Extrude More" / "Resume" menu and wait for reply
          KEEPALIVE_STATE(PAUSED_FOR_USER);
          wait_for_user = false;
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION);
          while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) idle(true);
          KEEPALIVE_STATE(IN_HANDLER);

          // Keep looping if "Extrude More" was selected
        } while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE);
      }

      KEEPALIVE_STATE(IN_HANDLER);
    }

    static void lcd_filament_change_unload_load (uint16_t changetemp, bool manual_extrude, bool unload_load){
      #ifdef FILAMENT_RUNOUT_DUAL
        // Store the filament sensor state
        bool filament_sensor_waiting = manual_extrude ?0:ACTIVE_FILAMENT_SENSOR_WAITING;
      #endif //FILAMENT_RUNOUT_DUAL

      SERIAL_DEBUG_MESSAGE_VALUE("Filament sensor state: ",ACTIVE_FILAMENT_SENSOR_WAITING);

      // Ensure the correct extruder is set
      if (active_extruder != filament_change_extruder)
        active_extruder = filament_change_extruder;

      // Clears extruder value
      current_position[E_AXIS] = 0;
      planner.set_e_position_mm(current_position[E_AXIS]);

      // Starts heating
      HOTEND_LOOP() thermalManager.setTargetHotend(changetemp, filament_change_extruder);

      //Disables the timeout to status screen
      defer_return_to_status = true;

      // Show "wait for heating"
      lcd_goto_screen(lcd_filament_change_hotendStatus);

      // Prepares the temporary variables
      unsigned long next_update = millis() + 200;
      bool update = true;

      #ifndef BEEVC_B2X300
        //This helps hBp speed up the temperature stabilization process without changing the PID
        //changetemp -= 10;
      #endif

      while (update){
        if (next_update < millis()){
          update = false;
          idle(true);
          HOTEND_LOOP() {
            #ifdef BEEVC_B2X300
              if (thermalManager.degHotend(filament_change_extruder) < (changetemp-5))
            #else
              if (abs(thermalManager.degHotend(active_extruder) - changetemp) > 10)
            #endif
            {
              update = true;
              break;
            }
          }
          // updates the lcd in each cycle
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
          // sets next screen update
          next_update = millis() + 200;
        }
      }

      //Shows "Press to continue" and beeps while waiting if not unloading
      if (unload_load){
        // Chooses the correct screen if there is a filament sensor
        #ifdef FILAMENT_RUNOUT_DUAL
          // If the sensor is ready to read filament insertion show press or insert screen
          if (filament_sensor_waiting)
            lcd_goto_screen(lcd_filament_change_press_insert);
          // Otherwise show normal screen
          else 
            lcd_goto_screen(lcd_filament_change_press);
        #else
          lcd_goto_screen(lcd_filament_change_press);
        #endif  //FILAMENT_RUNOUT_DUAL

        KEEPALIVE_STATE(PAUSED_FOR_USER);
        wait_for_user = true;    // LCD click or M108 will clear this
        next_update = millis();
        while (wait_for_user ) {
          if(next_update < millis()) {
              #if HAS_BUZZER
                buzzer.tone(100, 2000);
              #endif
              next_update = millis() + 1000;
          }

          #ifdef FILAMENT_RUNOUT_DUAL
            //Detects if the filament sensor is activated
            //Only works if filament sensor started out empty
            if(filament_sensor_waiting){
              if(!ACTIVE_FILAMENT_SENSOR_WAITING){
                SERIAL_DEBUG_MESSAGE_VALUE("Filament sensor triggered state: ",ACTIVE_FILAMENT_SENSOR_WAITING);
                

                // short beep and wait, total of 2s
                buzzer.tone(200, 2000);
                beevc_wait(2000);

                break;
              }
            }
          #endif //FILAMENT_RUNOUT_DUAL

          idle(true);
        }
      }

      KEEPALIVE_STATE(IN_HANDLER);

      if(! manual_extrude){
        // Buzz to inform start of load/unload
        buzzer.tone(1000, 2000);

        // Load/unload
        beevc_load_unload(unload_load);
      }
      else{
        lcd_goto_screen(lcd_filament_change_move_e);
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        wait_for_user = true;    // LCD click or M108 will clear this
        while (wait_for_user ) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);
      }

      //Ensures the motion has finished
      while(planner.movesplanned() > 0 ) idle();

      // Goes back to the action selection
      lcd_goto_screen(lcd_filament_change_choose_action);
    }

    static void lcd_filament_change_finish_movement () {
      lcd_filament_change_moving();
      beevc_screen_constant_update = true;

      // Checks if necessary movements have been made or if unloading
      if (((current_position[Z_AXIS] == 50) && axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) || !filament_change_load){
        //beevc_screen_constant_update = false;
        lcd_filament_change_unload_load (filament_change_temp,filament_change_manual, filament_change_load);
      }
    }

    static void lcd_filament_change_home_move () {
      // only moves if loading or manually extruding
      if (filament_change_load){
        // if not homed homes axis and lifts Z
        if (!(axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])){
          // Forces a home on all the axes
          axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
          // Home
          enqueue_and_echo_commands_P(PSTR("G28"));
          enqueue_and_echo_commands_P(PSTR("G1 Z50 F3000"));
        }

        // if homed only moves to load position
        else if (!((current_position[X_AXIS] == 146) && (current_position[Y_AXIS] == 59) && (current_position[Z_AXIS] == 50)) && filament_change_load) {
          enqueue_and_echo_commands_P(PSTR("G1 X146 Y59 Z50 F8000"));
        }
      }

      lcd_goto_screen(lcd_filament_change_finish_movement);

      beevc_force_screen_update();
    }


    static void lcd_filament_change_action_unload () {
      filament_change_load = false ;
      filament_change_manual = false ;
      lcd_goto_screen(lcd_filament_change_home_move);
    }

    static void lcd_filament_change_action_load () {
      filament_change_load = true ;
      filament_change_manual = false ;
      lcd_goto_screen(lcd_filament_change_home_move);
    }

    static void lcd_filament_change_action_move () {
      filament_change_load = true ;
      filament_change_manual = true ;
      
      lcd_goto_screen(lcd_filament_change_home_move);
    }

    static void lcd_filament_change_choose_action () {
      START_MENU();
      MENU_BACK(MSG_BACK);
      MENU_ITEM(function, _UxGT("Auto Load"), lcd_filament_change_action_load);
      MENU_ITEM(function, _UxGT("Auto Unload"), lcd_filament_change_action_unload);
      MENU_ITEM(function, _UxGT("Manual Extrusion"), lcd_filament_change_action_move);
      MENU_ITEM(function, "Exit", lcd_return_to_status);
      END_MENU();
    }

    static void lcd_filament_change_pla ()
    {
      filament_change_temp = 210;
      thermalManager.setTargetHotend(filament_change_temp, filament_change_extruder);
      lcd_goto_screen(lcd_filament_change_choose_action);
    }

    static void lcd_filament_change_petg ()
    {
      filament_change_temp = 230;
      thermalManager.setTargetHotend(filament_change_temp, filament_change_extruder);
      lcd_goto_screen(lcd_filament_change_choose_action);
    }

    static void lcd_filament_change_abs ()
    {
      filament_change_temp = 240;
      thermalManager.setTargetHotend(filament_change_temp, filament_change_extruder);
      lcd_goto_screen(lcd_filament_change_choose_action);
    }

    static void lcd_filament_change_pc ()
    {
      filament_change_temp = 260;
      thermalManager.setTargetHotend(filament_change_temp, filament_change_extruder);
      lcd_goto_screen(lcd_filament_change_choose_action);
    }

    static void lcd_filament_change_choose_temp() {
      START_MENU();

      // Go back to previous menu
      MENU_BACK(MSG_BACK);

      // \x09 is degree sign \x43 is 'C'
      MENU_ITEM(submenu, _UxGT("PLA   210\x09\x43"), lcd_filament_change_pla); 
      MENU_ITEM(submenu, _UxGT("PETG  230\x09\x43"), lcd_filament_change_petg);
      MENU_ITEM(submenu, _UxGT("ABS   240\x09\x43"), lcd_filament_change_abs);
      MENU_ITEM(submenu, _UxGT("PC    260\x09\x43"), lcd_filament_change_pc);

      END_MENU();
    }

    static void lcd_filament_change_extruder_0()
    {
      filament_change_extruder = 0;
      active_extruder = 0;

      lcd_goto_screen(lcd_filament_change_choose_temp);

      // Force screen update
      beevc_force_screen_update();
    }

    static void lcd_filament_change_extruder_1()
    {
      filament_change_extruder = 1;
      active_extruder = 1;

      lcd_goto_screen(lcd_filament_change_choose_temp);

      // Force screen update
      beevc_force_screen_update();
    }

    static void lcd_filament_change_exit(){
      // Allows returning to status
      defer_return_to_status = false;
      beevc_screen_constant_update = false;

      // Back action
      menu_action_back();
      menu_action_back();
    }


    static void lcd_filament_change()
    {
      START_MENU();

      // Go back to previous menu
      MENU_ITEM(submenu, _UxGT("Back"), lcd_filament_change_exit);

      MENU_ITEM(submenu, _UxGT("Extruder 1"), lcd_filament_change_extruder_0);
      MENU_ITEM(submenu, _UxGT("Extruder 2"), lcd_filament_change_extruder_1);

      END_MENU();
    }

    static void lcd_filament_change_start()
    {
      defer_return_to_status = true;
      beevc_screen_constant_update = true;

      lcd_goto_screen(lcd_filament_change);

      // Force screen update
      beevc_force_screen_update();
    }

    void lcd_enqueue_filament_change() {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (!DEBUGGING(DRYRUN) && !thermalManager.allow_cold_extrude &&
            thermalManager.degTargetHotend(active_extruder) < thermalManager.extrude_min_temp) {
          lcd_save_previous_screen();
          lcd_goto_screen(lcd_advanced_pause_toocold_menu);
          return;
        }
      #endif

      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
      enqueue_and_echo_commands_P(PSTR("M600 B0"));
    }

  #endif // ADVANCED_PAUSE_FEATURE

  // First Fan Speed title in "Tune" and "Control>Temperature" menus
  #if FAN_COUNT > 0 && HAS_FAN0
    #if FAN_COUNT > 1
      #define FAN_SPEED_1_SUFFIX " 1"
    #else
      #define FAN_SPEED_1_SUFFIX ""
    #endif
  #endif

  // Refresh the E factor after changing flow
  inline void _lcd_refresh_e_factor_0() { planner.refresh_e_factor(0); }
  #if EXTRUDERS > 1
    inline void _lcd_refresh_e_factor() { planner.refresh_e_factor(active_extruder); }
    inline void _lcd_refresh_e_factor_1() { planner.refresh_e_factor(1); }
    #if EXTRUDERS > 2
      inline void _lcd_refresh_e_factor_2() { planner.refresh_e_factor(2); }
      #if EXTRUDERS > 3
        inline void _lcd_refresh_e_factor_3() { planner.refresh_e_factor(3); }
        #if EXTRUDERS > 4
          inline void _lcd_refresh_e_factor_4() { planner.refresh_e_factor(4); }
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  /**
   *  BEEVC
   *
   * "Print settings" submenu items
   *
   */
  void beevc_print_settings_menu() {
    START_MENU();

    // Back
    MENU_BACK(MSG_MAIN);

    // Babystep Z offset
    MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);

    // Nozzle:
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);

    // Bed:
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3,MSG_TEMPERATURE MSG_SBED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);

    // Fan Speed:
    MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED FAN_SPEED_1_SUFFIX, &fanSpeeds[0], 0, 255);

    // Makes sure the fan isn't set for a speed at which it can't spin
    if (fanSpeeds[0] > 0 && fanSpeeds[0] < B2X300_MIN_FAN)
      fanSpeeds[0]=B2X300_MIN_FAN;

    // Feedrate:
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999);

    // Flow:
    //MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW, &planner.flow_percentage[active_extruder], 10, 999, _lcd_refresh_e_factor);
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_SE1, &planner.flow_percentage[0], 10, 999, _lcd_refresh_e_factor_0);
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_SE2, &planner.flow_percentage[1], 10, 999, _lcd_refresh_e_factor_1);

    END_MENU();
  }

  /**
   *
   * "Tune" submenu
   *
   */
  void lcd_tune_menu() {
    START_MENU();

    //
    // ^ Main
    //
    MENU_BACK(MSG_MAIN);

    //
    // Speed:
    //
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999);

    // Manual bed leveling, Bed Z:
    #if ENABLED(MESH_BED_LEVELING) && ENABLED(LCD_BED_LEVELING)
      MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
    #endif

    //
    // Nozzle:
    // Nozzle [1-2]:
    //
    #if HOTENDS == 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #else // HOTENDS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
      #if HOTENDS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &thermalManager.target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
        #if HOTENDS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &thermalManager.target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
          #if HOTENDS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N5, &thermalManager.target_temperature[4], 0, HEATER_4_MAXTEMP - 15, watch_temp_callback_E4);
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if HAS_TEMP_BED
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3,MSG_TEMPERATURE MSG_SBED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);
    #endif

    //
    // Fan Speed:
    //
    #if FAN_COUNT > 0
      #if HAS_FAN0
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED FAN_SPEED_1_SUFFIX, &fanSpeeds[0], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED FAN_SPEED_1_SUFFIX, &new_fanSpeeds[0], 3, 255);
        #endif
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED " 2", &new_fanSpeeds[1], 3, 255);
        #endif
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED " 3", &new_fanSpeeds[2], 3, 255);
        #endif
      #endif
    #endif // FAN_COUNT > 0

    //
    // Flow:
    // Flow [1-5]:
    //
    #if EXTRUDERS == 1
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW, &planner.flow_percentage[0], 10, 999, _lcd_refresh_e_factor_0);
    #else // EXTRUDERS > 1
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW, &planner.flow_percentage[active_extruder], 10, 999, _lcd_refresh_e_factor);
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_SE1, &planner.flow_percentage[0], 10, 999, _lcd_refresh_e_factor_0);
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_SE2, &planner.flow_percentage[1], 10, 999, _lcd_refresh_e_factor_1);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N3, &planner.flow_percentage[2], 10, 999, _lcd_refresh_e_factor_2);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N4, &planner.flow_percentage[3], 10, 999, _lcd_refresh_e_factor_3);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N5, &planner.flow_percentage[4], 10, 999, _lcd_refresh_e_factor_4);
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #endif // EXTRUDERS > 1

    //
    // Babystep X:
    // Babystep Y:
    // Babystep Z:
    //
    #if ENABLED(BABYSTEPPING)
      #if ENABLED(BABYSTEP_XY)
        MENU_ITEM(submenu, MSG_BABYSTEP_X, lcd_babystep_x);
        MENU_ITEM(submenu, MSG_BABYSTEP_Y, lcd_babystep_y);
      #endif
      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
        MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
      #else
        MENU_ITEM(submenu, MSG_BABYSTEP_Z, lcd_babystep_z);
      #endif
    #endif

    //
    // Change filament
    //
	/*
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      if (!thermalManager.tooColdToExtrude(active_extruder))
        MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_enqueue_filament_change);
    #endif
	*/

    END_MENU();
  }

  /**
   *
   * "Driver current control" submenu items
   *
   */
  #if ENABLED(DAC_STEPPER_CURRENT)

    void dac_driver_getValues() { LOOP_XYZE(i) driverPercent[i] = dac_current_get_percent((AxisEnum)i); }

    void dac_driver_commit() { dac_current_set_percents(driverPercent); }

    void dac_driver_eeprom_write() { dac_commit_eeprom(); }

    void lcd_dac_menu() {
      dac_driver_getValues();
      START_MENU();
      MENU_BACK(MSG_CONTROL);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_X " " MSG_DAC_PERCENT, &driverPercent[X_AXIS], 0, 100, dac_driver_commit);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_Y " " MSG_DAC_PERCENT, &driverPercent[Y_AXIS], 0, 100, dac_driver_commit);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_Z " " MSG_DAC_PERCENT, &driverPercent[Z_AXIS], 0, 100, dac_driver_commit);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_E " " MSG_DAC_PERCENT, &driverPercent[E_AXIS], 0, 100, dac_driver_commit);
      MENU_ITEM(function, MSG_DAC_EEPROM_WRITE, dac_driver_eeprom_write);
      END_MENU();
    }

  #endif // DAC_STEPPER_CURRENT

  #if HAS_MOTOR_CURRENT_PWM

    void lcd_pwm_menu() {
      START_MENU();
      MENU_BACK(MSG_CONTROL);
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_X MSG_Y, &stepper.motor_current_setting[0], 100, 2000, Stepper::refresh_motor_power);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_Z, &stepper.motor_current_setting[1], 100, 2000, Stepper::refresh_motor_power);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_E, &stepper.motor_current_setting[2], 100, 2000, Stepper::refresh_motor_power);
      #endif
      END_MENU();
    }

  #endif // HAS_MOTOR_CURRENT_PWM

  constexpr int16_t heater_maxtemp[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP, HEATER_4_MAXTEMP);

  /**
   *
   * "Prepare" submenu items
   *
   */
  void _lcd_preheat(const int16_t endnum, const int16_t temph, const int16_t tempb, const int16_t fan) {
    if (temph > 0) thermalManager.setTargetHotend(min(heater_maxtemp[endnum], temph), endnum);
    #if TEMP_SENSOR_BED != 0
      if (tempb >= 0) thermalManager.setTargetBed(tempb);
    #else
      UNUSED(tempb);
    #endif
    #if FAN_COUNT > 0
      #if FAN_COUNT > 1
        fanSpeeds[active_extruder < FAN_COUNT ? active_extruder : 0] = fan;
      #else
        fanSpeeds[0] = fan;
      #endif
    #else
      UNUSED(fan);
    #endif
    lcd_return_to_status();
  }

  #if TEMP_SENSOR_0 != 0
    void lcd_preheat_m1_e0_only() { _lcd_preheat(0, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_e0_only() { _lcd_preheat(0, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
    #if TEMP_SENSOR_BED != 0
      void lcd_preheat_m1_e0() { _lcd_preheat(0, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_e0() { _lcd_preheat(0, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
    #endif
  #endif

  #if HOTENDS > 1
    void lcd_preheat_m1_e1_only() { _lcd_preheat(1, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_e1_only() { _lcd_preheat(1, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
    #if TEMP_SENSOR_BED != 0
      void lcd_preheat_m1_e1() { _lcd_preheat(1, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_e1() { _lcd_preheat(1, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
    #endif
    #if HOTENDS > 2
      void lcd_preheat_m1_e2_only() { _lcd_preheat(2, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_e2_only() { _lcd_preheat(2, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
      #if TEMP_SENSOR_BED != 0
        void lcd_preheat_m1_e2() { _lcd_preheat(2, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
        void lcd_preheat_m2_e2() { _lcd_preheat(2, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
      #endif
      #if HOTENDS > 3
        void lcd_preheat_m1_e3_only() { _lcd_preheat(3, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
        void lcd_preheat_m2_e3_only() { _lcd_preheat(3, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
        #if TEMP_SENSOR_BED != 0
          void lcd_preheat_m1_e3() { _lcd_preheat(3, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
          void lcd_preheat_m2_e3() { _lcd_preheat(3, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
        #endif
        #if HOTENDS > 4
          void lcd_preheat_m1_e4_only() { _lcd_preheat(4, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
          void lcd_preheat_m2_e4_only() { _lcd_preheat(4, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
          #if TEMP_SENSOR_BED != 0
            void lcd_preheat_m1_e4() { _lcd_preheat(4, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
            void lcd_preheat_m2_e4() { _lcd_preheat(4, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
          #endif
        #endif // HOTENDS > 4
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2

    void lcd_preheat_m1_all() {
      #if HOTENDS > 1
        thermalManager.setTargetHotend(lcd_preheat_hotend_temp[0], 1);
        #if HOTENDS > 2
          thermalManager.setTargetHotend(lcd_preheat_hotend_temp[0], 2);
          #if HOTENDS > 3
            thermalManager.setTargetHotend(lcd_preheat_hotend_temp[0], 3);
            #if HOTENDS > 4
              thermalManager.setTargetHotend(lcd_preheat_hotend_temp[0], 4);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
      #if TEMP_SENSOR_BED != 0
        lcd_preheat_m1_e0();
      #else
        lcd_preheat_m1_e0_only();
      #endif
    }
    void lcd_preheat_m2_all() {
      #if HOTENDS > 1
        thermalManager.setTargetHotend(lcd_preheat_hotend_temp[1], 1);
        #if HOTENDS > 2
          thermalManager.setTargetHotend(lcd_preheat_hotend_temp[1], 2);
          #if HOTENDS > 3
            thermalManager.setTargetHotend(lcd_preheat_hotend_temp[1], 3);
            #if HOTENDS > 4
              thermalManager.setTargetHotend(lcd_preheat_hotend_temp[1], 4);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
      #if TEMP_SENSOR_BED != 0
        lcd_preheat_m2_e0();
      #else
        lcd_preheat_m2_e0_only();
      #endif
    }

  #endif // HOTENDS > 1

  #if TEMP_SENSOR_BED != 0
    void lcd_preheat_m1_bedonly() { _lcd_preheat(0, 0, lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_bedonly() { _lcd_preheat(0, 0, lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
  #endif

  #if TEMP_SENSOR_0 != 0 && (TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_4 != 0 || TEMP_SENSOR_BED != 0)

    void lcd_preheat_m1_menu() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      #if HOTENDS == 1
        #if TEMP_SENSOR_BED != 0
          MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_e0);
          MENU_ITEM(function, MSG_PREHEAT_1_END, lcd_preheat_m1_e0_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_e0_only);
        #endif
      #else
        #if TEMP_SENSOR_BED != 0
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E1, lcd_preheat_m1_e0);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E1 " End", lcd_preheat_m1_e0_only);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E2, lcd_preheat_m1_e1);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E2 " End", lcd_preheat_m1_e1_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E1, lcd_preheat_m1_e0_only);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_E2, lcd_preheat_m1_e1_only);
        #endif
        #if HOTENDS > 2
          #if TEMP_SENSOR_BED != 0
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_e2);
            MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_E3, lcd_preheat_m1_e2_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_e2_only);
          #endif
          #if HOTENDS > 3
            #if TEMP_SENSOR_BED != 0
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_e3);
              MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_E4, lcd_preheat_m1_e3_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_e3_only);
            #endif
            #if HOTENDS > 4
              #if TEMP_SENSOR_BED != 0
                MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_e4);
                MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_E5, lcd_preheat_m1_e4_only);
              #else
                MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_e4_only);
              #endif
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
        MENU_ITEM(function, MSG_PREHEAT_1_ALL, lcd_preheat_m1_all);
      #endif // HOTENDS > 1
      #if TEMP_SENSOR_BED != 0
        MENU_ITEM(function, MSG_PREHEAT_1_BEDONLY, lcd_preheat_m1_bedonly);
      #endif
      END_MENU();
    }

    void lcd_preheat_m2_menu() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      #if HOTENDS == 1
        #if TEMP_SENSOR_BED != 0
          MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_e0);
          MENU_ITEM(function, MSG_PREHEAT_2_END, lcd_preheat_m2_e0_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_e0_only);
        #endif
      #else
        #if TEMP_SENSOR_BED != 0
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E1, lcd_preheat_m1_e0);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E1 " End", lcd_preheat_m1_e0_only);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E2, lcd_preheat_m1_e1);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E2 " End", lcd_preheat_m1_e1_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E1, lcd_preheat_m1_e0_only);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_E2, lcd_preheat_m1_e1_only);
        #endif
        #if HOTENDS > 2
          #if TEMP_SENSOR_BED != 0
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_e2);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_E3, lcd_preheat_m2_e2_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_e2_only);
          #endif
          #if HOTENDS > 3
            #if TEMP_SENSOR_BED != 0
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_e3);
              MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_E4, lcd_preheat_m2_e3_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_e3_only);
            #endif
            #if HOTENDS > 4
              #if TEMP_SENSOR_BED != 0
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_e4);
                MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_E5, lcd_preheat_m2_e4_only);
              #else
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_e4_only);
              #endif
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
        MENU_ITEM(function, MSG_PREHEAT_2_ALL, lcd_preheat_m2_all);
      #endif // HOTENDS > 1
      #if TEMP_SENSOR_BED != 0
        MENU_ITEM(function, MSG_PREHEAT_2_BEDONLY, lcd_preheat_m2_bedonly);
      #endif
      END_MENU();
    }

  #endif // TEMP_SENSOR_0 && (TEMP_SENSOR_1 || TEMP_SENSOR_2 || TEMP_SENSOR_3 || TEMP_SENSOR_4 || TEMP_SENSOR_BED)

  void lcd_cooldown() {
    #if FAN_COUNT > 0
      for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #endif
    thermalManager.disable_all_heaters();
    lcd_return_to_status();
  }

  #if ENABLED(SDSUPPORT) && ENABLED(MENU_ADDAUTOSTART)

    void lcd_autostart_sd() {
      card.autostart_index = 0;
      card.setroot();
      card.checkautostart(true);
    }

  #endif

  #if ENABLED(EEPROM_SETTINGS)
    static void lcd_store_settings()   { lcd_completion_feedback(settings.save()); }
    static void lcd_load_settings()    { lcd_completion_feedback(settings.load()); }
  #endif

  #if ENABLED(LEVEL_BED_CORNERS)

    /**
     * Level corners, starting in the front-left corner.
     */
    static int8_t bed_corner;
    void _lcd_goto_next_corner() {
      line_to_z(4.0);
      switch (bed_corner) {
        case 0:
          current_position[X_AXIS] = X_MIN_BED + 10;
          current_position[Y_AXIS] = Y_MIN_BED + 10;
          break;
        case 1:
          current_position[X_AXIS] = X_MAX_BED - 10;
          break;
        case 2:
          current_position[Y_AXIS] = Y_MAX_BED - 10;
          break;
        case 3:
          current_position[X_AXIS] = X_MIN_BED + 10;
          break;
      }
      planner.buffer_line_kinematic(current_position, MMM_TO_MMS(manual_feedrate_mm_m[X_AXIS]), active_extruder);
      line_to_z(0.0);
      if (++bed_corner > 3) bed_corner = 0;
    }

    void _lcd_corner_submenu() {
      START_MENU();
      MENU_ITEM(function, MSG_NEXT_CORNER, _lcd_goto_next_corner);
      MENU_ITEM(function, MSG_BACK, lcd_goto_previous_menu_no_defer);
      END_MENU();
    }

    void _lcd_level_bed_corners() {
      defer_return_to_status = true;
      lcd_goto_screen(_lcd_corner_submenu);
      bed_corner = 0;
      _lcd_goto_next_corner();
    }

  #endif // LEVEL_BED_CORNERS

  #if ENABLED(LCD_BED_LEVELING)

    /**
     *
     * "Prepare" > "Level Bed" handlers
     *
     */

    static uint8_t manual_probe_index;

    // LCD probed points are from defaults
    constexpr uint8_t total_probe_points = (
      #if ENABLED(AUTO_BED_LEVELING_3POINT)
        3
      #elif ABL_GRID || ENABLED(MESH_BED_LEVELING)
        GRID_MAX_POINTS
      #endif
    );

    bool lcd_wait_for_move;

    //
    // Bed leveling is done. Wait for G29 to complete.
    // A flag is used so that this can release control
    // and allow the command queue to be processed.
    //
    // When G29 finishes the last move:
    // - Raise Z to the "manual probe height"
    // - Don't return until done.
    //
    // ** This blocks the command queue! **
    //
    void _lcd_level_bed_done() {
      if (!lcd_wait_for_move) {
        #if MANUAL_PROBE_HEIGHT > 0 && DISABLED(MESH_BED_LEVELING)
          // Display "Done" screen and wait for moves to complete
          line_to_z(Z_MIN_POS + MANUAL_PROBE_HEIGHT);
          lcd_synchronize(PSTR(MSG_LEVEL_BED_DONE));
        #endif
        lcd_goto_previous_menu();
        lcd_completion_feedback();
        defer_return_to_status = false;
      }
      if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT >= 4 ? 1 : 0, PSTR(MSG_LEVEL_BED_DONE));
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
    }


    #if ENABLED(MESH_BED_LEVELING) || ENABLED(PROBE_MANUALLY)
      void _lcd_level_goto_next_point();
    #endif

	void lcd_bed_level_homing() {
      START_SCREEN();
      STATIC_ITEM("Homing XYZ axis ", true, true);
      STATIC_ITEM("Allow movement to");
	  STATIC_ITEM("finish and press ");
	  STATIC_ITEM("to continue...  ");
      END_SCREEN();
    }

	void lcd_bed_level_homed() {
      START_SCREEN();
      STATIC_ITEM("Homing finished ", true, true);
	  STATIC_ITEM("Press to continue");
      END_SCREEN();
    }



    void _lcd_level_goto_next_point();

    /**
     * Step 7: Get the Z coordinate, click goes to the next point or exits
     */
    void _lcd_level_bed_get_z() {
      ENCODER_DIRECTION_NORMAL();

      if (lcd_clicked) {

        //
        // Save the current Z position and move
        //

        // If done...
        if (++manual_probe_index >= total_probe_points) {
          //
          // The last G29 records the point and enables bed leveling
          //
          lcd_wait_for_move = true;
          lcd_goto_screen(_lcd_level_bed_done);
          #if ENABLED(PROBE_MANUALLY)
            enqueue_and_echo_commands_P(PSTR("G29 V1"));
          #elif ENABLED(MESH_BED_LEVELING)
            enqueue_and_echo_commands_P(PSTR("G29 S2"));
          #endif

		  //DR 29-01-18 Saves the leveling mesh
		  enqueue_and_echo_commands_P(PSTR("M500"));
        }
        else
          _lcd_level_goto_next_point();

        return;
      }

      //
      // Encoder knob or keypad buttons adjust the Z position
      //
      if (encoderPosition) {
        refresh_cmd_timeout();
        const float z = current_position[Z_AXIS] + float((int32_t)encoderPosition) * (MBL_Z_STEP);
        line_to_z(constrain(z, -(LCD_PROBE_Z_RANGE) * 0.5, (LCD_PROBE_Z_RANGE) * 0.5));
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
        encoderPosition = 0;
      }

      //
      // Draw on first display, then only on Z change
      //
      if (lcdDrawUpdate) {
        const float v = current_position[Z_AXIS];
        lcd_implementation_drawedit(PSTR(MSG_MOVE_Z), ftostr43sign(v + (v < 0 ? -0.0001 : 0.0001), '+'));
      }
    }

    /**
     * Step 6: Display "Next point: 1 / 9" while waiting for move to finish
     */
    void _lcd_level_bed_moving() {
      if (lcdDrawUpdate) {
        char msg[10];
        sprintf_P(msg, PSTR("%i / %u"), (int)(manual_probe_index + 1), total_probe_points);
        lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_NEXT_POINT), msg);
      }
      lcdDrawUpdate = LCDVIEW_CALL_NO_REDRAW;
      if (!lcd_wait_for_move) lcd_goto_screen(_lcd_level_bed_get_z);
    }

    /**
     * Step 5: Initiate a move to the next point
     */
    void _lcd_level_goto_next_point() {
      // Set the menu to display ahead of blocking call
      lcd_goto_screen(_lcd_level_bed_moving);

      // G29 Records Z, moves, and signals when it pauses
      lcd_wait_for_move = true;
      #if ENABLED(PROBE_MANUALLY)
        enqueue_and_echo_commands_P(PSTR("G29 V1"));
      #elif ENABLED(MESH_BED_LEVELING)
        enqueue_and_echo_commands_P(manual_probe_index ? PSTR("G29 S2") : PSTR("G29 S1"));
      #endif
    }

    /**
     * Step 4: Display "Click to Begin", wait for click
     *         Move to the first probe position
     */
    void _lcd_level_bed_homing_done() {

	  lcdDrawUpdate = 1;

	  if (lcdDrawUpdate) {
		START_SCREEN();
      STATIC_ITEM("Homing finished ", true, true);
	  STATIC_ITEM("Press to continue");
      END_SCREEN();
	  }

	  //lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_WAITING));

      if (lcd_clicked) {
        manual_probe_index = 0;
        _lcd_level_goto_next_point();
      }
    }

    /**
     * Step 3: Display "Homing XYZ" - Wait for homing to finish
     */
    void _lcd_level_bed_homing() {
      if (lcdDrawUpdate)
	  {
	  START_SCREEN();
      STATIC_ITEM("Homing XYZ axis ", true, true);
      STATIC_ITEM("Allow movement to");
	  STATIC_ITEM("finish and press ");
	  STATIC_ITEM("twice to continue");
      END_SCREEN();
	  }
	  lcdDrawUpdate = LCDVIEW_CALL_NO_REDRAW;

      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
	  {
		//lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_WAITING));
		lcdDrawUpdate = 1;
		START_SCREEN();
        STATIC_ITEM("Homing finished ", true, true);
	    STATIC_ITEM("Press to continue");
        END_SCREEN();
        lcd_goto_screen(_lcd_level_bed_homing_done);

	  }


    }

    #if ENABLED(PROBE_MANUALLY)
      extern bool g29_in_progress;
    #endif

    /**
     * Step 2: Continue Bed Leveling...
     */
    void _lcd_level_bed_continue() {

      defer_return_to_status = true;
      axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
      lcd_goto_screen(_lcd_level_bed_homing);
      enqueue_and_echo_commands_P(PSTR("G28"));
    }

    static bool new_level_state;
    void _lcd_toggle_bed_leveling() { set_bed_leveling_enabled(new_level_state); }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      static float new_z_fade_height;
      void _lcd_set_z_fade_height() { set_z_fade_height(new_z_fade_height); }
    #endif

    /**
     * Step 1: Bed Level entry-point
     *
     * << Prepare
     *    Auto Home           (if homing needed)
     *    Leveling On/Off     (if data exists, and homed)
     *    Fade Height: ---    (Req: ENABLE_LEVELING_FADE_HEIGHT)
     *    Mesh Z Offset: ---  (Req: MESH_BED_LEVELING)
     *    Z Probe Offset: --- (Req: HAS_BED_PROBE, Opt: BABYSTEP_ZPROBE_OFFSET)
     *    Level Bed >
     *    Level Corners >     (if homed)
     *    Load Settings       (Req: EEPROM_SETTINGS)
     *    Save Settings       (Req: EEPROM_SETTINGS)
     */
    void lcd_bed_leveling() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);

      #if DISABLED(MESH_BED_LEVELING)
        if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS]))
          MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
        else
      #endif
        if (leveling_is_valid()) {
          new_level_state = planner.leveling_active;
          MENU_ITEM_EDIT_CALLBACK(bool, MSG_BED_LEVELING, &new_level_state, _lcd_toggle_bed_leveling);
        }

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_Z_FADE_HEIGHT, &new_z_fade_height, 0.0, 100.0, _lcd_set_z_fade_height);
      #endif

      //
      // MBL Z Offset
      //
      #if ENABLED(MESH_BED_LEVELING)
        MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
      #endif

      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
        MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
      #elif HAS_BED_PROBE
        MENU_ITEM_EDIT(float32, MSG_ZPROBE_ZOFFSET, &zprobe_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
      #endif

      MENU_ITEM(submenu, MSG_LEVEL_BED, _lcd_level_bed_continue);

      #if ENABLED(LEVEL_BED_CORNERS)
        // Move to the next corner for leveling
        if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
          MENU_ITEM(submenu, MSG_LEVEL_CORNERS, _lcd_level_bed_corners);
      #endif

      #if ENABLED(EEPROM_SETTINGS)
        MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
        MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
      #endif
      END_MENU();
    }

    void _lcd_goto_bed_leveling() {
      lcd_goto_screen(lcd_bed_leveling);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        new_z_fade_height = planner.z_fade_height;
      #endif
    }

  #elif ENABLED(AUTO_BED_LEVELING_UBL)

    void _lcd_ubl_level_bed();

    static int16_t ubl_storage_slot = 0,
               custom_hotend_temp = 190,
               side_points = 3,
               ubl_fillin_amount = 5,
               ubl_height_amount = 1,
               n_edit_pts = 1,
               x_plot = 0,
               y_plot = 0;

    #if HAS_TEMP_BED
      static int16_t custom_bed_temp = 50;
    #endif

    /**
     * UBL Build Custom Mesh Command
     */
    void _lcd_ubl_build_custom_mesh() {
      char UBL_LCD_GCODE[20];
      enqueue_and_echo_commands_P(PSTR("G28"));
      #if HAS_TEMP_BED
        sprintf_P(UBL_LCD_GCODE, PSTR("M190 S%i"), custom_bed_temp);
        enqueue_and_echo_command(UBL_LCD_GCODE);
      #endif
      sprintf_P(UBL_LCD_GCODE, PSTR("M109 S%i"), custom_hotend_temp);
      enqueue_and_echo_command(UBL_LCD_GCODE);
      enqueue_and_echo_commands_P(PSTR("G29 P1"));
    }

    /**
     * UBL Custom Mesh submenu
     *
     * << Build Mesh
     *    Hotend Temp: ---
     *    Bed Temp: ---
     *    Build Custom Mesh
     */
    void _lcd_ubl_custom_mesh() {
      START_MENU();
      MENU_BACK(MSG_UBL_BUILD_MESH_MENU);
      MENU_ITEM_EDIT(int3, MSG_UBL_CUSTOM_HOTEND_TEMP, &custom_hotend_temp, EXTRUDE_MINTEMP, (HEATER_0_MAXTEMP - 10));
      #if HAS_TEMP_BED
        MENU_ITEM_EDIT(int3, MSG_UBL_CUSTOM_BED_TEMP, &custom_bed_temp, BED_MINTEMP, (BED_MAXTEMP - 5));
      #endif
      MENU_ITEM(function, MSG_UBL_BUILD_CUSTOM_MESH, _lcd_ubl_build_custom_mesh);
      END_MENU();
    }

    /**
     * UBL Adjust Mesh Height Command
     */
    void _lcd_ubl_adjust_height_cmd() {
      char UBL_LCD_GCODE[16];
      const int ind = ubl_height_amount > 0 ? 9 : 10;
      strcpy_P(UBL_LCD_GCODE, PSTR("G29 P6 C -"));
      sprintf_P(&UBL_LCD_GCODE[ind], PSTR(".%i"), abs(ubl_height_amount));
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Adjust Mesh Height submenu
     *
     * << Edit Mesh
     *    Height Amount: ---
     *    Adjust Mesh Height
     * << Info Screen
     */
    void _lcd_ubl_height_adjust_menu() {
      START_MENU();
      MENU_BACK(MSG_UBL_EDIT_MESH_MENU);
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_UBL_MESH_HEIGHT_AMOUNT, &ubl_height_amount, -9, 9, _lcd_ubl_adjust_height_cmd);
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    /**
     * UBL Edit Mesh submenu
     *
     * << UBL Tools
     *    Fine Tune All
     *    Fine Tune Closest
     *  - Adjust Mesh Height >>
     * << Info Screen
     */
    void _lcd_ubl_edit_mesh() {
      START_MENU();
      MENU_BACK(MSG_UBL_TOOLS);
      MENU_ITEM(gcode, MSG_UBL_FINE_TUNE_ALL, PSTR("G29 P4 R999 T"));
      MENU_ITEM(gcode, MSG_UBL_FINE_TUNE_CLOSEST, PSTR("G29 P4 T"));
      MENU_ITEM(submenu, MSG_UBL_MESH_HEIGHT_ADJUST, _lcd_ubl_height_adjust_menu);
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    /**
     * UBL Validate Custom Mesh Command
     */
    void _lcd_ubl_validate_custom_mesh() {
      char UBL_LCD_GCODE[24];
      const int temp =
        #if HAS_TEMP_BED
          custom_bed_temp
        #else
          0
        #endif
      ;
      sprintf_P(UBL_LCD_GCODE, PSTR("G28\nG26 C B%i H%i P"), temp, custom_hotend_temp);
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Validate Mesh submenu
     *
     * << UBL Tools
     *    PLA Mesh Validation
     *    ABS Mesh Validation
     *    Validate Custom Mesh
     * << Info Screen
     */
    void _lcd_ubl_validate_mesh() {
      START_MENU();
      MENU_BACK(MSG_UBL_TOOLS);
      #if HAS_TEMP_BED
        MENU_ITEM(gcode, MSG_UBL_VALIDATE_PLA_MESH, PSTR("G28\nG26 C B" STRINGIFY(PREHEAT_1_TEMP_BED) " H" STRINGIFY(PREHEAT_1_TEMP_HOTEND) " P"));
        MENU_ITEM(gcode, MSG_UBL_VALIDATE_ABS_MESH, PSTR("G28\nG26 C B" STRINGIFY(PREHEAT_2_TEMP_BED) " H" STRINGIFY(PREHEAT_2_TEMP_HOTEND) " P"));
      #else
        MENU_ITEM(gcode, MSG_UBL_VALIDATE_PLA_MESH, PSTR("G28\nG26 C B0 H" STRINGIFY(PREHEAT_1_TEMP_HOTEND) " P"));
        MENU_ITEM(gcode, MSG_UBL_VALIDATE_ABS_MESH, PSTR("G28\nG26 C B0 H" STRINGIFY(PREHEAT_2_TEMP_HOTEND) " P"));
      #endif
      MENU_ITEM(function, MSG_UBL_VALIDATE_CUSTOM_MESH, _lcd_ubl_validate_custom_mesh);
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    /**
     * UBL Grid Leveling Command
     */
    void _lcd_ubl_grid_level_cmd() {
      char UBL_LCD_GCODE[10];
      sprintf_P(UBL_LCD_GCODE, PSTR("G29 J%i"), side_points);
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Grid Leveling submenu
     *
     * << UBL Tools
     *    Side points: ---
     *    Level Mesh
     */
    void _lcd_ubl_grid_level() {
      START_MENU();
      MENU_BACK(MSG_UBL_TOOLS);
      MENU_ITEM_EDIT(int3, MSG_UBL_SIDE_POINTS, &side_points, 2, 6);
      MENU_ITEM(function, MSG_UBL_MESH_LEVEL, _lcd_ubl_grid_level_cmd);
      END_MENU();
    }

    /**
     * UBL Mesh Leveling submenu
     *
     * << UBL Tools
     *    3-Point Mesh Leveling
     *  - Grid Mesh Leveling >>
     * << Info Screen
     */
    void _lcd_ubl_mesh_leveling() {
      START_MENU();
      MENU_BACK(MSG_UBL_TOOLS);
      MENU_ITEM(gcode, MSG_UBL_3POINT_MESH_LEVELING, PSTR("G29 J0"));
      MENU_ITEM(submenu, MSG_UBL_GRID_MESH_LEVELING, _lcd_ubl_grid_level);
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    /**
     * UBL Fill-in Amount Mesh Command
     */
    void _lcd_ubl_fillin_amount_cmd() {
      char UBL_LCD_GCODE[16];
      sprintf_P(UBL_LCD_GCODE, PSTR("G29 P3 R C.%i"), ubl_fillin_amount);
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Smart Fill-in Command
     */
    void _lcd_ubl_smart_fillin_cmd() {
      char UBL_LCD_GCODE[12];
      sprintf_P(UBL_LCD_GCODE, PSTR("G29 P3 T0"));
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Fill-in Mesh submenu
     *
     * << Build Mesh
     *    Fill-in Amount: ---
     *    Fill-in Mesh
     *    Smart Fill-in
     *    Manual Fill-in
     * << Info Screen
     */
    void _lcd_ubl_fillin_menu() {
      START_MENU();
      MENU_BACK(MSG_UBL_BUILD_MESH_MENU);
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_UBL_FILLIN_AMOUNT, &ubl_fillin_amount, 0, 9, _lcd_ubl_fillin_amount_cmd);
      MENU_ITEM(function, MSG_UBL_SMART_FILLIN, _lcd_ubl_smart_fillin_cmd);
      MENU_ITEM(gcode, MSG_UBL_MANUAL_FILLIN, PSTR("G29 P2 B T0"));
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    void _lcd_ubl_invalidate() {
      ubl.invalidate();
      SERIAL_PROTOCOLLNPGM("Mesh invalidated.");
    }

    /**
     * UBL Build Mesh submenu
     *
     * << UBL Tools
     *    Build PLA Mesh
     *    Build ABS Mesh
     *  - Build Custom Mesh >>
     *    Build Cold Mesh
     *  - Fill-in Mesh >>
     *    Continue Bed Mesh
     *    Invalidate All
     *    Invalidate Closest
     * << Info Screen
     */
    void _lcd_ubl_build_mesh() {
      START_MENU();
      MENU_BACK(MSG_UBL_TOOLS);
      #if HAS_TEMP_BED
        MENU_ITEM(gcode, MSG_UBL_BUILD_PLA_MESH, PSTR(
          "G28\n"
          "M190 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\n"
          "M109 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND) "\n"
          "G29 P1\n"
          "M104 S0\n"
          "M140 S0"
        ));
        MENU_ITEM(gcode, MSG_UBL_BUILD_ABS_MESH, PSTR(
          "G28\n"
          "M190 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\n"
          "M109 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND) "\n"
          "G29 P1\n"
          "M104 S0\n"
          "M140 S0"
        ));
      #else
        MENU_ITEM(gcode, MSG_UBL_BUILD_PLA_MESH, PSTR(
          "G28\n"
          "M109 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND) "\n"
          "G29 P1\n"
          "M104 S0"
        ));
        MENU_ITEM(gcode, MSG_UBL_BUILD_ABS_MESH, PSTR(
          "G28\n"
          "M109 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND) "\n"
          "G29 P1\n"
          "M104 S0"
        ));
      #endif
      MENU_ITEM(submenu, MSG_UBL_BUILD_CUSTOM_MESH, _lcd_ubl_custom_mesh);
      MENU_ITEM(gcode, MSG_UBL_BUILD_COLD_MESH, PSTR("G28\nG29 P1"));
      MENU_ITEM(submenu, MSG_UBL_FILLIN_MESH, _lcd_ubl_fillin_menu);
      MENU_ITEM(gcode, MSG_UBL_CONTINUE_MESH, PSTR("G29 P1 C"));
      MENU_ITEM(function, MSG_UBL_INVALIDATE_ALL, _lcd_ubl_invalidate);
      MENU_ITEM(gcode, MSG_UBL_INVALIDATE_CLOSEST, PSTR("G29 I"));
      MENU_ITEM(function, MSG_WATCH, lcd_return_to_status);
      END_MENU();
    }

    /**
     * UBL Load Mesh Command
     */
    void _lcd_ubl_load_mesh_cmd() {
      char UBL_LCD_GCODE[25];
      sprintf_P(UBL_LCD_GCODE, PSTR("G29 L%i"), ubl_storage_slot);
      enqueue_and_echo_command(UBL_LCD_GCODE);
      sprintf_P(UBL_LCD_GCODE, PSTR("M117 " MSG_MESH_LOADED "."), ubl_storage_slot);
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Save Mesh Command
     */
    void _lcd_ubl_save_mesh_cmd() {
      char UBL_LCD_GCODE[25];
      sprintf_P(UBL_LCD_GCODE, PSTR("G29 S%i"), ubl_storage_slot);
      enqueue_and_echo_command(UBL_LCD_GCODE);
      sprintf_P(UBL_LCD_GCODE, PSTR("M117 " MSG_MESH_SAVED "."), ubl_storage_slot);
      enqueue_and_echo_command(UBL_LCD_GCODE);
    }

    /**
     * UBL Mesh Storage submenu
     *
     * << Unified Bed Leveling
     *    Memory Slot: ---
     *    Load Bed Mesh
     *    Save Bed Mesh
     */
    void _lcd_ubl_storage_mesh() {
      int16_t a = settings.calc_num_meshes();
      START_MENU();
      MENU_BACK(MSG_UBL_LEVEL_BED);
      if (!WITHIN(ubl_storage_slot, 0, a - 1)) {
        STATIC_ITEM(MSG_NO_STORAGE);
        STATIC_ITEM(MSG_INIT_EEPROM);
      }
      else {
        MENU_ITEM_EDIT(int3, MSG_UBL_STORAGE_SLOT, &ubl_storage_slot, 0, a - 1);
        MENU_ITEM(function, MSG_UBL_LOAD_MESH, _lcd_ubl_load_mesh_cmd);
        MENU_ITEM(function, MSG_UBL_SAVE_MESH, _lcd_ubl_save_mesh_cmd);
      }
      END_MENU();
    }

    /**
     * UBL LCD "radar" map homing
     */
    void _lcd_ubl_output_map_lcd();

    void _lcd_ubl_map_homing() {
      defer_return_to_status = true;
      if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT < 3 ? 0 : (LCD_HEIGHT > 4 ? 2 : 1), PSTR(MSG_LEVEL_BED_HOMING));
      lcdDrawUpdate = LCDVIEW_CALL_NO_REDRAW;
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
        ubl.lcd_map_control = true; // Return to the map screen
        lcd_goto_screen(_lcd_ubl_output_map_lcd);
      }
    }

    /**
     * UBL LCD "radar" map point editing
     */
    void _lcd_ubl_map_lcd_edit_cmd() {
      char ubl_lcd_gcode [50], str[10], str2[10];

      dtostrf(pgm_read_float(&ubl._mesh_index_to_xpos[x_plot]), 0, 2, str);
      dtostrf(pgm_read_float(&ubl._mesh_index_to_ypos[y_plot]), 0, 2, str2);
      snprintf_P(ubl_lcd_gcode, sizeof(ubl_lcd_gcode), PSTR("G29 P4 X%s Y%s R%i"), str, str2, n_edit_pts);
      enqueue_and_echo_command(ubl_lcd_gcode);
    }

    /**
     * UBL LCD Map Movement
     */
    void ubl_map_move_to_xy() {
      current_position[X_AXIS] = pgm_read_float(&ubl._mesh_index_to_xpos[x_plot]);
      current_position[Y_AXIS] = pgm_read_float(&ubl._mesh_index_to_ypos[y_plot]);
      planner.buffer_line_kinematic(current_position, MMM_TO_MMS(XY_PROBE_SPEED), active_extruder);
    }

    /**
     * UBL LCD "radar" map
     */
    void set_current_from_steppers_for_axis(const AxisEnum axis);
    void sync_plan_position();

    void _lcd_do_nothing() {}
    void _lcd_hard_stop() {
      stepper.quick_stop();
      const screenFunc_t old_screen = currentScreen;
      currentScreen = _lcd_do_nothing;
      while (planner.movesplanned()) idle();
      currentScreen = old_screen;
      stepper.cleaning_buffer_counter = 0;
      set_current_from_steppers_for_axis(ALL_AXES);
      sync_plan_position();
      refresh_cmd_timeout();
    }

    void _lcd_ubl_output_map_lcd() {
      static int16_t step_scaler = 0;

      if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS]))
        return lcd_goto_screen(_lcd_ubl_map_homing);

      if (lcd_clicked) return _lcd_ubl_map_lcd_edit_cmd();
      ENCODER_DIRECTION_NORMAL();

      if (encoderPosition) {
        step_scaler += (int32_t)encoderPosition;
        x_plot += step_scaler / (ENCODER_STEPS_PER_MENU_ITEM);
        if (abs(step_scaler) >= ENCODER_STEPS_PER_MENU_ITEM)
          step_scaler = 0;
        refresh_cmd_timeout();

        encoderPosition = 0;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }

      // Encoder to the right (++)
      if (x_plot >= GRID_MAX_POINTS_X) { x_plot = 0; y_plot++; }
      if (y_plot >= GRID_MAX_POINTS_Y) y_plot = 0;

      // Encoder to the left (--)
      if (x_plot <= GRID_MAX_POINTS_X - (GRID_MAX_POINTS_X + 1)) { x_plot = GRID_MAX_POINTS_X - 1; y_plot--; }
      if (y_plot <= GRID_MAX_POINTS_Y - (GRID_MAX_POINTS_Y + 1)) y_plot = GRID_MAX_POINTS_Y - 1;

      // Prevent underrun/overrun of plot numbers
      x_plot = constrain(x_plot, GRID_MAX_POINTS_X - (GRID_MAX_POINTS_X + 1), GRID_MAX_POINTS_X + 1);
      y_plot = constrain(y_plot, GRID_MAX_POINTS_Y - (GRID_MAX_POINTS_Y + 1), GRID_MAX_POINTS_Y + 1);

      // Determine number of points to edit
      #if IS_KINEMATIC
        n_edit_pts = 9; //TODO: Delta accessible edit points
      #else
        const bool xc = WITHIN(x_plot, 1, GRID_MAX_POINTS_X - 2),
                   yc = WITHIN(y_plot, 1, GRID_MAX_POINTS_Y - 2);
        n_edit_pts = yc ? (xc ? 9 : 6) : (xc ? 6 : 4); // Corners
      #endif

      if (lcdDrawUpdate) {
        lcd_implementation_ubl_plot(x_plot, y_plot);

        if (planner.movesplanned()) // If the nozzle is already moving, cancel the move.
          _lcd_hard_stop();

        ubl_map_move_to_xy();       // Move to new location
      }
    }

    /**
     * UBL Homing before LCD map
     */
    void _lcd_ubl_output_map_lcd_cmd() {
      if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
        axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
        enqueue_and_echo_commands_P(PSTR("G28"));
      }
      lcd_goto_screen(_lcd_ubl_map_homing);
    }

    /**
     * UBL Output map submenu
     *
     * << Unified Bed Leveling
     *  Output for Host
     *  Output for CSV
     *  Off Printer Backup
     *  Output Mesh Map
     */
    void _lcd_ubl_output_map() {
      START_MENU();
      MENU_BACK(MSG_UBL_LEVEL_BED);
      MENU_ITEM(gcode, MSG_UBL_OUTPUT_MAP_HOST, PSTR("G29 T0"));
      MENU_ITEM(gcode, MSG_UBL_OUTPUT_MAP_CSV, PSTR("G29 T1"));
      MENU_ITEM(gcode, MSG_UBL_OUTPUT_MAP_BACKUP, PSTR("G29 S-1"));
      MENU_ITEM(function, MSG_UBL_OUTPUT_MAP, _lcd_ubl_output_map_lcd_cmd);
      END_MENU();
    }

    /**
     * UBL Tools submenu
     *
     * << Unified Bed Leveling
     *  - Build Mesh >>
     *  - Validate Mesh >>
     *  - Edit Mesh >>
     *  - Mesh Leveling >>
     */
    void _lcd_ubl_tools_menu() {
      START_MENU();
      MENU_BACK(MSG_UBL_LEVEL_BED);
      MENU_ITEM(submenu, MSG_UBL_BUILD_MESH_MENU, _lcd_ubl_build_mesh);
      MENU_ITEM(gcode, MSG_UBL_MANUAL_MESH, PSTR("G29 I999\nG29 P2 B T0"));
      MENU_ITEM(submenu, MSG_UBL_VALIDATE_MESH_MENU, _lcd_ubl_validate_mesh);
      MENU_ITEM(submenu, MSG_UBL_EDIT_MESH_MENU, _lcd_ubl_edit_mesh);
      MENU_ITEM(submenu, MSG_UBL_MESH_LEVELING, _lcd_ubl_mesh_leveling);
      END_MENU();
    }

    /**
     * UBL Step-By-Step submenu
     *
     * << Unified Bed Leveling
     *    1 Build Cold Mesh
     *    2 Smart Fill-in
     *  - 3 Validate Mesh >>
     *    4 Fine Tune All
     *  - 5 Validate Mesh >>
     *    6 Fine Tune All
     *    7 Save Bed Mesh
     */
    void _lcd_ubl_step_by_step() {
      START_MENU();
      MENU_BACK(MSG_UBL_LEVEL_BED);
      MENU_ITEM(gcode, "1 " MSG_UBL_BUILD_COLD_MESH, PSTR("G28\nG29 P1"));
      MENU_ITEM(function, "2 " MSG_UBL_SMART_FILLIN, _lcd_ubl_smart_fillin_cmd);
      MENU_ITEM(submenu, "3 " MSG_UBL_VALIDATE_MESH_MENU, _lcd_ubl_validate_mesh);
      MENU_ITEM(gcode, "4 " MSG_UBL_FINE_TUNE_ALL, PSTR("G29 P4 R999 T"));
      MENU_ITEM(submenu, "5 " MSG_UBL_VALIDATE_MESH_MENU, _lcd_ubl_validate_mesh);
      MENU_ITEM(gcode, "6 " MSG_UBL_FINE_TUNE_ALL, PSTR("G29 P4 R999 T"));
      MENU_ITEM(function, "7 " MSG_UBL_SAVE_MESH, _lcd_ubl_save_mesh_cmd);
      END_MENU();
    }

    /**
     * UBL System submenu
     *
     * << Prepare
     *  - Manually Build Mesh >>
     *  - Activate UBL >>
     *  - Deactivate UBL >>
     *  - Step-By-Step UBL >>
     *  - Mesh Storage >>
     *  - Output Map >>
     *  - UBL Tools >>
     *  - Output UBL Info >>
     */

    void _lcd_ubl_level_bed() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      MENU_ITEM(gcode, MSG_UBL_ACTIVATE_MESH, PSTR("G29 A"));
      MENU_ITEM(gcode, MSG_UBL_DEACTIVATE_MESH, PSTR("G29 D"));
      MENU_ITEM(submenu, MSG_UBL_STEP_BY_STEP_MENU, _lcd_ubl_step_by_step);
      MENU_ITEM(function, MSG_UBL_MESH_EDIT, _lcd_ubl_output_map_lcd_cmd);
      MENU_ITEM(submenu, MSG_UBL_STORAGE_MESH_MENU, _lcd_ubl_storage_mesh);
      MENU_ITEM(submenu, MSG_UBL_OUTPUT_MAP, _lcd_ubl_output_map);
      MENU_ITEM(submenu, MSG_UBL_TOOLS, _lcd_ubl_tools_menu);
      MENU_ITEM(gcode, MSG_UBL_INFO_UBL, PSTR("G29 W"));
      END_MENU();
    }

  #endif // AUTO_BED_LEVELING_UBL

  /**
   *
   * "Prepare" submenu
   *
   */

  void lcd_prepare_menu() {
    START_MENU();

    //
    // ^ Main
    //
    MENU_BACK(MSG_MAIN);

    //
    // Move Axis
    //
    #if ENABLED(DELTA)
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
    #endif
        MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);

    //
    // Auto Home
    //
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    #if ENABLED(INDIVIDUAL_AXIS_HOMING_MENU)
      MENU_ITEM(gcode, MSG_AUTO_HOME_X, PSTR("G28 X"));
      MENU_ITEM(gcode, MSG_AUTO_HOME_Y, PSTR("G28 Y"));
      MENU_ITEM(gcode, MSG_AUTO_HOME_Z, PSTR("G28 Z"));
    #endif

    //
    // Level Bed
    //
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      MENU_ITEM(submenu, MSG_UBL_LEVEL_BED, _lcd_ubl_level_bed);
    #elif ENABLED(LCD_BED_LEVELING)
      #if ENABLED(PROBE_MANUALLY)
        if (!g29_in_progress)
      #endif
          MENU_ITEM(submenu, MSG_BED_LEVELING,
            #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
              _lcd_goto_bed_leveling
            #else
              lcd_bed_leveling
            #endif
          );
    #elif PLANNER_LEVELING && DISABLED(PROBE_MANUALLY)
      //DR - Unnecessary as the bed leveling is already on the main menu
	  //MENU_ITEM(gcode, MSG_BED_LEVELING, PSTR("G28\nG29"));
    #endif

    #if ENABLED(LEVEL_BED_CORNERS) && DISABLED(LCD_BED_LEVELING)
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
        MENU_ITEM(function, MSG_LEVEL_CORNERS, _lcd_level_bed_corners);
    #endif

    #if HAS_M206_COMMAND
      //
      // Set Home Offsets
      //

	  //DR
      //MENU_ITEM(function, MSG_SET_HOME_OFFSETS, lcd_set_home_offsets);

      //MENU_ITEM(gcode, MSG_SET_ORIGIN, PSTR("G92 X0 Y0 Z0"));
    #endif

    //
    // Disable Steppers
    //
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));

    //
    // Change filament
    //
	// Removed as change filament already exists on the main menu
	/*
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      if (!thermalManager.tooColdToExtrude(active_extruder) && !IS_SD_FILE_OPEN)
        MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_enqueue_filament_change);
    #endif
	*/

    #if TEMP_SENSOR_0 != 0

      //
      // Cooldown
      //
      bool has_heat = false;
      HOTEND_LOOP() if (thermalManager.target_temperature[HOTEND_INDEX]) { has_heat = true; break; }
      #if HAS_TEMP_BED
        if (thermalManager.target_temperature_bed) has_heat = true;
      #endif
      if (has_heat) MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

      //
      // Preheat for Material 1 and 2
      //
      #if TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_4 != 0 || TEMP_SENSOR_BED != 0
        MENU_ITEM(submenu, MSG_PREHEAT_1, lcd_preheat_m1_menu);
        MENU_ITEM(submenu, MSG_PREHEAT_2, lcd_preheat_m2_menu);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_e0_only);
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_e0_only);
      #endif

    #endif // TEMP_SENSOR_0 != 0

    //
    // BLTouch Self-Test and Reset
    //
    #if ENABLED(BLTOUCH)
      MENU_ITEM(gcode, MSG_BLTOUCH_SELFTEST, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_SELFTEST)));
      if (!endstops.z_probe_enabled && TEST_BLTOUCH())
        MENU_ITEM(gcode, MSG_BLTOUCH_RESET, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_RESET)));
    #endif

    //
    // Switch power on/off
    //
    #if HAS_POWER_SWITCH
      if (powersupply_on)
        MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
      else
        MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
    #endif

    //
    // Autostart
    //
    #if ENABLED(SDSUPPORT) && ENABLED(MENU_ADDAUTOSTART)
      MENU_ITEM(function, MSG_AUTOSTART, lcd_autostart_sd);
    #endif

    //
    // Delta Calibration
    //
    #if ENABLED(DELTA_CALIBRATION_MENU) || ENABLED(DELTA_AUTO_CALIBRATION)
      MENU_ITEM(submenu, MSG_DELTA_CALIBRATE, lcd_delta_calibrate_menu);
    #endif

    END_MENU();
  }

  float move_menu_scale;

  #if ENABLED(DELTA_CALIBRATION_MENU) || (ENABLED(DELTA_AUTO_CALIBRATION) && !HAS_BED_PROBE)

    void lcd_move_z();

    void _man_probe_pt(const float &rx, const float &ry) {
      #if HAS_LEVELING
        reset_bed_level(); // After calibration bed-level data is no longer valid
      #endif

      line_to_z((Z_CLEARANCE_BETWEEN_PROBES) + (DELTA_PRINTABLE_RADIUS) / 5);
      current_position[X_AXIS] = rx;
      current_position[Y_AXIS] = ry;
      line_to_current_z();
      line_to_z(Z_CLEARANCE_BETWEEN_PROBES);

      lcd_synchronize();
      move_menu_scale = PROBE_MANUALLY_STEP;
      lcd_goto_screen(lcd_move_z);
    }

  #endif // DELTA_CALIBRATION_MENU || (DELTA_AUTO_CALIBRATION && !HAS_BED_PROBE)

  #if ENABLED(DELTA_AUTO_CALIBRATION) && !HAS_BED_PROBE

    float lcd_probe_pt(const float &rx, const float &ry) {
      _man_probe_pt(rx, ry);
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      defer_return_to_status = true;
      wait_for_user = true;
      while (wait_for_user) idle();
      KEEPALIVE_STATE(IN_HANDLER);
      lcd_goto_previous_menu_no_defer();
      return current_position[Z_AXIS];
    }

  #endif // DELTA_AUTO_CALIBRATION && !HAS_BED_PROBE

  #if ENABLED(DELTA_CALIBRATION_MENU)

    void _lcd_calibrate_homing() {
      if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT >= 4 ? 1 : 0, PSTR(MSG_LEVEL_BED_HOMING));
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
        lcd_goto_previous_menu();
    }

    void _lcd_delta_calibrate_home() {
      #if HAS_LEVELING
        reset_bed_level(); // After calibration bed-level data is no longer valid
      #endif

      enqueue_and_echo_commands_P(PSTR("G28"));
      lcd_goto_screen(_lcd_calibrate_homing);
    }

    void _goto_tower_x() { _man_probe_pt(cos(RADIANS(210)) * delta_calibration_radius, sin(RADIANS(210)) * delta_calibration_radius); }
    void _goto_tower_y() { _man_probe_pt(cos(RADIANS(330)) * delta_calibration_radius, sin(RADIANS(330)) * delta_calibration_radius); }
    void _goto_tower_z() { _man_probe_pt(cos(RADIANS( 90)) * delta_calibration_radius, sin(RADIANS( 90)) * delta_calibration_radius); }
    void _goto_center()  { _man_probe_pt(0,0); }

  #endif // DELTA_CALIBRATION_MENU

  #if ENABLED(DELTA_CALIBRATION_MENU) || ENABLED(DELTA_AUTO_CALIBRATION)

    void lcd_delta_settings() {
      START_MENU();
      MENU_BACK(MSG_DELTA_CALIBRATE);
      MENU_ITEM_EDIT_CALLBACK(float52, MSG_DELTA_DIAG_ROD, &delta_diagonal_rod, delta_diagonal_rod - 5.0, delta_diagonal_rod + 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float52, MSG_DELTA_HEIGHT, &delta_height, delta_height - 10.0, delta_height + 10.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Ex", &delta_endstop_adj[A_AXIS], -5.0, 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Ey", &delta_endstop_adj[B_AXIS], -5.0, 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Ez", &delta_endstop_adj[C_AXIS], -5.0, 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float52, MSG_DELTA_RADIUS, &delta_radius, delta_radius - 5.0, delta_radius + 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Tx", &delta_tower_angle_trim[A_AXIS], -5.0, 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Ty", &delta_tower_angle_trim[B_AXIS], -5.0, 5.0, recalc_delta_settings);
      MENU_ITEM_EDIT_CALLBACK(float43, "Tz", &delta_tower_angle_trim[C_AXIS], -5.0, 5.0, recalc_delta_settings);
      END_MENU();
    }

    void lcd_delta_calibrate_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      #if ENABLED(DELTA_AUTO_CALIBRATION)
        MENU_ITEM(gcode, MSG_DELTA_AUTO_CALIBRATE, PSTR("G33"));
        MENU_ITEM(gcode, MSG_DELTA_HEIGHT_CALIBRATE, PSTR("G33 P1"));
        #if ENABLED(EEPROM_SETTINGS)
          MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
          MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
        #endif
      #endif
      MENU_ITEM(submenu, MSG_DELTA_SETTINGS, lcd_delta_settings);
      #if ENABLED(DELTA_CALIBRATION_MENU)
        MENU_ITEM(submenu, MSG_AUTO_HOME, _lcd_delta_calibrate_home);
        if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
          MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_X, _goto_tower_x);
          MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Y, _goto_tower_y);
          MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Z, _goto_tower_z);
          MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_CENTER, _goto_center);
        }
      #endif
      END_MENU();
    }

  #endif // DELTA_CALIBRATION_MENU || DELTA_AUTO_CALIBRATION

  /**
   * If the most recent manual move hasn't been fed to the planner yet,
   * and the planner can accept one, send immediately
   */
  inline void manage_manual_move() {

    if (processing_manual_move) return;

    if (manual_move_axis != (int8_t)NO_AXIS && ELAPSED(millis(), manual_move_start_time) && !planner.is_full()) {

      #if IS_KINEMATIC

        const float old_feedrate = feedrate_mm_s;
        feedrate_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]);

        #if EXTRUDERS > 1
          const int8_t old_extruder = active_extruder;
          active_extruder = manual_move_e_index;
        #endif

        // Set movement on a single axis
        set_destination_from_current();
        destination[manual_move_axis] += manual_move_offset;

        // Reset for the next move
        manual_move_offset = 0.0;
        manual_move_axis = (int8_t)NO_AXIS;

        // Set a blocking flag so no new moves can be added until all segments are done
        processing_manual_move = true;
        prepare_move_to_destination(); // will call set_current_from_destination()
        processing_manual_move = false;

        feedrate_mm_s = old_feedrate;
        #if EXTRUDERS > 1
          active_extruder = old_extruder;
        #endif

      #else

        planner.buffer_line_kinematic(current_position, MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]), manual_move_e_index);
        manual_move_axis = (int8_t)NO_AXIS;

      #endif
    }
  }

  /**
   * Set a flag that lcd_update() should start a move
   * to "current_position" after a short delay.
   */
  inline void manual_move_to_current(AxisEnum axis
    #if E_MANUAL > 1
      , int8_t eindex=-1
    #endif
  ) {
    #if ENABLED(DUAL_X_CARRIAGE) || E_MANUAL > 1
      #if E_MANUAL > 1
        if (axis == E_AXIS)
      #endif
          manual_move_e_index = eindex >= 0 ? eindex : active_extruder;
    #endif
    manual_move_start_time = millis() + (move_menu_scale < 0.99 ? 0UL : 250UL); // delay for bigger moves
    manual_move_axis = (int8_t)axis;
  }

  #if HAS_ABL

    /**
   *
   * "Calibrate Z offset" submenu
   *
   */

	void _lcd_screen_calibrate_z_offset() { lcd_goto_screen(_lcd_calibrate_z_offset);}

	void _lcd_screen_reset_z_offset() { lcd_goto_screen(_lcd_reset_z_offset);}

    void _lcd_z_offset_completed()
	{
	  lcd_implementation_clear();

	  START_SCREEN();

	  STATIC_ITEM(_UxGT("Nozzle height"), true, true);
	  lcd_implementation_drawmenu_static(2, PSTR("Process completed"));
	  lcd_implementation_drawmenu_static(3, PSTR("  Press to exit  "));

      END_SCREEN();
	  if (lcd_clicked)
		{
			defer_return_to_status = false;
			lcd_implementation_clear();
			beevc_main_menu();
		}
	}

	void _lcd_reset_z_offset()
    {

		if (lcd_clicked)
		{
			zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
			lcd_completion_feedback(settings.save());

			return _lcd_z_offset_completed();
		}

		if (lcdDrawUpdate)
		{
			START_SCREEN();
			STATIC_ITEM(_UxGT("Nozzle height"), true, true);
			lcd_implementation_drawmenu_static(2, PSTR("Process completed"));
			lcd_implementation_drawmenu_static(3, PSTR("  Press to exit  "));
			END_SCREEN();
		}
	}

  void _lcd_calibrate_z_offset()
  {

    if (lcd_clicked)
	{
		zprobe_zoffset = (current_position[Z_AXIS] + zprobe_zoffset);
		lcd_completion_feedback(settings.save());

    enqueue_and_echo_commands_P(PSTR("G28 X Y"));
		lcd_goto_screen(_lcd_z_offset_completed);
	}

    ENCODER_DIRECTION_NORMAL();

    if (encoderPosition) {
      refresh_cmd_timeout();

      float min = current_position[Z_AXIS] - 1000,
            max = current_position[Z_AXIS] + 1000;

      // Get the new position
      current_position[Z_AXIS] -= float((int32_t)encoderPosition) * 0.05;


      // Limit only when trying to move towards the limit
      if ((int32_t)encoderPosition < 0) NOLESS(current_position[Z_AXIS], min);
      if ((int32_t)encoderPosition > 0) NOMORE(current_position[Z_AXIS], max);

      manual_move_to_current(Z_AXIS);

      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }
    if (lcdDrawUpdate)
	{
		START_SCREEN();
		STATIC_ITEM(_UxGT("Nozzle height"), true, true);
		lcd_implementation_drawedit(PSTR(_UxGT("Z height")), ftostr42sign((current_position[Z_AXIS] + zprobe_zoffset)));
		lcd_implementation_drawmenu_static(3,PSTR("Press to save"));

		END_SCREEN();
	}
  }

	void _lcd_z_offset_prepare_calibration()
	{
		do
		{
			START_SCREEN();
			STATIC_ITEM(_UxGT("Nozzle height"), true, true);
			lcd_implementation_drawedit(PSTR(_UxGT("Z height")), ftostr41sign((current_position[Z_AXIS] + zprobe_zoffset)));
			lcd_implementation_drawmenu_static(3,PSTR("Press to save"));
			END_SCREEN();
		}
		while (!(lcdDrawUpdate));

		current_position[Z_AXIS] -= 8;
		manual_move_to_current(Z_AXIS);

    // Tries forcing redraw
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    //idle(true);

		lcd_goto_screen(_lcd_calibrate_z_offset);

	}

  void _lcd_z_offset_finish_homing() {

    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    encoderPosition = 10;

    if (lcdDrawUpdate) {
      START_SCREEN();
      STATIC_ITEM(_UxGT("Nozzle height"), true, true);
      STATIC_ITEM("Movements complete ");
      STATIC_ITEM("press to continue!");
      END_SCREEN();
  	}


    if(lcd_clicked)
    lcd_goto_screen( _lcd_z_offset_prepare_calibration);

  }

  void _lcd_z_offset_check_bed_homing() {
    // Tries forcing redraw
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    //idle(true);

    if (lcdDrawUpdate) {
	     START_SCREEN();
	     STATIC_ITEM(_UxGT("Nozzle height"), true, true);
       STATIC_ITEM("Homing XYZ axis ");
       STATIC_ITEM("Allow movement to");
	     STATIC_ITEM("finish and press!");
       END_SCREEN();
	  }

    if  (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS] && planner.leveling_active)
		lcd_goto_screen( _lcd_z_offset_finish_homing);

  }

	void _lcd_z_offset_start_bed_homing()
	{
		defer_return_to_status = true;
    axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
    enqueue_and_echo_commands_P(PSTR("G28\nG29\nG0 X150 Y100"));
		lcd_goto_screen(_lcd_z_offset_check_bed_homing);

	}

  void _lcd_menu_z_offset()
    {
		START_MENU();
		if (LCD_HEIGHT >= 4)
		{
			STATIC_ITEM(_UxGT("Nozzle height"), true, true);
		}

		MENU_BACK(_UxGT("Main"));
		MENU_ITEM(submenu, _UxGT("Calibrate"), _lcd_z_offset_start_bed_homing);
		MENU_ITEM(submenu, _UxGT("Reset"), _lcd_reset_z_offset);

		END_MENU();

    }

	#endif


    /**
   *
   * "Calibrate Extruder" submenu
   *
   *

	void _lcd_screen_calibrate_extruder() { lcd_goto_screen(_lcd_calibrate_extruder);}

	void _lcd_screen_reset_calibrate_extruder() { lcd_goto_screen(_lcd_reset_calibrate_extruder);}

    void _lcd_calibrate_extruder_completed()
	{
		lcd_implementation_clear();

		START_SCREEN();
	  STATIC_ITEM(_UxGT("Extruder Calibration"), true, true);
	  lcd_implementation_drawmenu_static(2, PSTR("Calibration Saved"));
	  lcd_implementation_drawmenu_static(3, PSTR("  Press to exit  "));
      END_SCREEN();
	  if (lcd_clicked)
		{
			defer_return_to_status = false;
			lcd_implementation_clear();
			lcd_main_menu();
		}
	}

	void _lcd_reset_calibrate_extruder()
    {

		if (lcd_clicked)
		{
			planner.axis_steps_per_mm[E_AXIS + active_extruder] = DEFAULT_AXIS_STEPS_PER_UNIT[3+active_extruder];
			lcd_completion_feedback(settings.save());

			return _lcd_calibrate_extruder_completed();
		}

		if (lcdDrawUpdate)
		{
			START_SCREEN();
			STATIC_ITEM(_UxGT("Extruder Calibration"), true, true);
			lcd_implementation_drawmenu_static(2, PSTR("Calibration Saved"));
			lcd_implementation_drawmenu_static(3, PSTR("  Press to exit  "));
			END_SCREEN();
		}
	}

  void _lcd_calibrate_extruder_store(float tempESteps)
  {

    if (lcd_clicked)
	{
		planner.axis_steps_per_mm[E_AXIS + active_extruder] = tempESteps;
		lcd_completion_feedback(settings.save());

		return _lcd_calibrate_extruder_completed();
	}

    ENCODER_DIRECTION_NORMAL();

    if (encoderPosition) {
      refresh_cmd_timeout();

      float min = DEFAULT_AXIS_STEPS_PER_UNIT[3+active_extruder] - 30,
            max = DEFAULT_AXIS_STEPS_PER_UNIT[3+active_extruder] + 30;

      // Get the new position
      tempESteps -= float((int32_t)encoderPosition) * 0.05;


      // Limit only when trying to move towards the limit
      if ((int32_t)encoderPosition < 0) NOLESS(tempESteps, min);
      if ((int32_t)encoderPosition > 0) NOMORE(tempESteps, max);

      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    if (lcdDrawUpdate)
	{
		START_SCREEN();
		STATIC_ITEM(_UxGT("Extruder Calibration"), true, true);
	  lcd_implementation_drawmenu_static(2, PSTR("Calibration Saved"));
		lcd_implementation_drawedit(PSTR(_UxGT("E steps:")), ftostr41sign((tempESteps));
		lcd_implementation_drawmenu_static(3,PSTR("Press to save"));

		END_SCREEN();
	}
  }

	void _lcd_z_offset_prepare_calibration()
	{
		do
		{
			START_SCREEN();
			STATIC_ITEM(_UxGT("Nozzle height"), true, true);
			lcd_implementation_drawedit(PSTR(_UxGT("Z height")), ftostr41sign((current_position[Z_AXIS] + zprobe_zoffset)));
			lcd_implementation_drawmenu_static(3,PSTR("Press to save"));
			END_SCREEN();
		}
		while (!(lcdDrawUpdate));

		current_position[Z_AXIS] -= 8;
		manual_move_to_current(Z_AXIS);

		lcd_goto_screen(_lcd_calibrate_z_offset);

	}

  void _lcd_z_offset_check_bed_homing() {
      if (lcdDrawUpdate && !(axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]))
	  {
	  START_SCREEN();
	  STATIC_ITEM(_UxGT("Nozzle height"), true, true);
      STATIC_ITEM("Homing XYZ axis ", true, true);
      STATIC_ITEM("Allow movement to");
	  STATIC_ITEM("finish and press!");
	  //STATIC_ITEM("finish and press ");
	  //STATIC_ITEM("   to continue   ");
      END_SCREEN();
	  }


      if  (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
	  {
		lcd_goto_screen( _lcd_z_offset_prepare_calibration);
	  }
    }

	void _lcd_z_offset_start_bed_homing()
	{
		defer_return_to_status = true;
        axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
        enqueue_and_echo_commands_P(PSTR("G28"));
		lcd_goto_screen(_lcd_z_offset_check_bed_homing);

	}

  void _lcd_menu_z_offset()
    {
		START_MENU();
		if (LCD_HEIGHT >= 4)
		{
			STATIC_ITEM(_UxGT("Nozzle height"), true, true);
		}

		MENU_BACK(_UxGT("Main"));
		MENU_ITEM(submenu, _UxGT("Calibrate"), _lcd_z_offset_start_bed_homing);
		MENU_ITEM(submenu, _UxGT("Reset"), _lcd_reset_z_offset);

		END_MENU();

    }

	#endif

	*/



  /**
   *
   * "Prepare" > "Move Axis" submenu
   *
   */

  void _lcd_move_xyz(const char* name, AxisEnum axis) {
    if (lcd_clicked) { return lcd_goto_previous_menu(); }
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition && !processing_manual_move) {
      refresh_cmd_timeout();

      // Start with no limits to movement
      float min = current_position[axis] - 1000,
            max = current_position[axis] + 1000;

      // Limit to software endstops, if enabled
      #if ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS)
        if (soft_endstops_enabled) switch (axis) {
          case X_AXIS:
            #if ENABLED(MIN_SOFTWARE_ENDSTOP_X)
              min = soft_endstop_min[X_AXIS];
            #endif
            #if ENABLED(MAX_SOFTWARE_ENDSTOP_X)
              max = soft_endstop_max[X_AXIS];
            #endif
            break;
          case Y_AXIS:
            #if ENABLED(MIN_SOFTWARE_ENDSTOP_Y)
              min = soft_endstop_min[Y_AXIS];
            #endif
            #if ENABLED(MAX_SOFTWARE_ENDSTOP_Y)
              max = soft_endstop_max[Y_AXIS];
            #endif
            break;
          case Z_AXIS:
            #if ENABLED(MIN_SOFTWARE_ENDSTOP_Z)
              min = soft_endstop_min[Z_AXIS];
            #endif
            #if ENABLED(MAX_SOFTWARE_ENDSTOP_Z)
              max = soft_endstop_max[Z_AXIS];
            #endif
          default: break;
        }
      #endif // MIN_SOFTWARE_ENDSTOPS || MAX_SOFTWARE_ENDSTOPS

      // Delta limits XY based on the current offset from center
      // This assumes the center is 0,0
      #if ENABLED(DELTA)
        if (axis != Z_AXIS) {
          max = SQRT(sq((float)(DELTA_PRINTABLE_RADIUS)) - sq(current_position[Y_AXIS - axis])); // (Y_AXIS - axis) == the other axis
          min = -max;
        }
      #endif

      // Apply a multiplyer if the move is long
      encoderPosition = encoderPosition *(abs((int32_t)encoderPosition));

      // Get the new position
      const float diff = float((int32_t)encoderPosition) * move_menu_scale;
      #if IS_KINEMATIC
        manual_move_offset += diff;
        // Limit only when trying to move towards the limit
        if ((int32_t)encoderPosition < 0) NOLESS(manual_move_offset, min - current_position[axis]);
        if ((int32_t)encoderPosition > 0) NOMORE(manual_move_offset, max - current_position[axis]);
      #else
        current_position[axis] += diff;
        // Limit only when trying to move towards the limit
        if ((int32_t)encoderPosition < 0) NOLESS(current_position[axis], min);
        if ((int32_t)encoderPosition > 0) NOMORE(current_position[axis], max);
      #endif

      encoderPosition = 0;

      manual_move_to_current(axis);

      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }
    if (lcdDrawUpdate || millis() > next_update) {
      const float pos = current_position[axis]
        #if IS_KINEMATIC
          + manual_move_offset
        #endif
      ;
      lcd_implementation_drawedit(name, move_menu_scale >= 0.1 ? ftostr41sign(pos) : ftostr43sign(pos));

      // makes sure there is at least one update per second
      next_update = millis() + 1000;
    }
  }
  void lcd_move_x() { _lcd_move_xyz(PSTR(MSG_MOVE_X), X_AXIS); }
  void lcd_move_y() { _lcd_move_xyz(PSTR(MSG_MOVE_Y), Y_AXIS); }
  void lcd_move_z() { _lcd_move_xyz(PSTR(MSG_MOVE_Z), Z_AXIS); }
  void _lcd_move_e(
    #if E_MANUAL > 1
      int8_t eindex=-1
    #endif
  ) {
    if (lcd_clicked) { return lcd_goto_previous_menu(); }
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      if (!processing_manual_move) {
        const float diff = float((int32_t)encoderPosition) * move_menu_scale;
        #if IS_KINEMATIC
          manual_move_offset += diff;
        #else
          current_position[E_AXIS] += diff;
        #endif
        manual_move_to_current(E_AXIS
          #if E_MANUAL > 1
            , eindex
          #endif
        );
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
      encoderPosition = 0;
    }
    if (lcdDrawUpdate && !processing_manual_move) {
      PGM_P pos_label;
      #if E_MANUAL == 1
        pos_label = PSTR(MSG_MOVE_E);
      #else
        switch (eindex) {
          default: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E1); break;
          case 1: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E2); break;
          #if E_MANUAL > 2
            case 2: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E3); break;
            #if E_MANUAL > 3
              case 3: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E4); break;
              #if E_MANUAL > 4
                case 4: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E5); break;
              #endif // E_MANUAL > 4
            #endif // E_MANUAL > 3
          #endif // E_MANUAL > 2
        }
      #endif // E_MANUAL > 1
      lcd_implementation_drawedit(pos_label, ftostr41sign(current_position[E_AXIS]
        #if IS_KINEMATIC
          + manual_move_offset
        #endif
      ));
    }
  }

  void lcd_move_e() { _lcd_move_e(); }
  #if E_MANUAL > 1
    void lcd_move_e0() { _lcd_move_e(0); }
    void lcd_move_e1() { _lcd_move_e(1); }
    #if E_MANUAL > 2
      void lcd_move_e2() { _lcd_move_e(2); }
      #if E_MANUAL > 3
        void lcd_move_e3() { _lcd_move_e(3); }
        #if E_MANUAL > 4
          void lcd_move_e4() { _lcd_move_e(4); }
        #endif // E_MANUAL > 4
      #endif // E_MANUAL > 3
    #endif // E_MANUAL > 2
  #endif // E_MANUAL > 1

  /**
   *
   * "Prepare" > "Move Xmm" > "Move XYZ" submenu
   *
   */

  screenFunc_t _manual_move_func_ptr;

  void _goto_manual_move(const float scale) {
    defer_return_to_status = true;
    move_menu_scale = scale;
    lcd_goto_screen(_manual_move_func_ptr);
  }
  void _goto_manual_move_small(const screenFunc_t func) {
    defer_return_to_status = true;
    move_menu_scale = 0.1;
    lcd_goto_screen(func);
  }

  void lcd_move_menu_10mm() { _goto_manual_move(10.0); }
  void lcd_move_menu_1mm()  { _goto_manual_move( 1.0); }
  void lcd_move_menu_01mm() { _goto_manual_move( 0.1); }

  void _lcd_move_distance_menu(const AxisEnum axis, const screenFunc_t func) {
    _manual_move_func_ptr = func;
    START_MENU();
    if (LCD_HEIGHT >= 4) {
      switch(axis) {
        case X_AXIS:
          STATIC_ITEM(MSG_MOVE_X, true, true); break;
        case Y_AXIS:
          STATIC_ITEM(MSG_MOVE_Y, true, true); break;
        case Z_AXIS:
          STATIC_ITEM(MSG_MOVE_Z, true, true); break;
        default:
          STATIC_ITEM(MSG_MOVE_E, true, true); break;
      }
    }
    MENU_BACK(MSG_MOVE_AXIS);
    MENU_ITEM(submenu, MSG_MOVE_10MM, lcd_move_menu_10mm);
    MENU_ITEM(submenu, MSG_MOVE_1MM, lcd_move_menu_1mm);
    MENU_ITEM(submenu, MSG_MOVE_01MM, lcd_move_menu_01mm);
    END_MENU();
  }
  void lcd_move_get_x_amount()        { _goto_manual_move_small(lcd_move_x); }
  void lcd_move_get_y_amount()        { _goto_manual_move_small(lcd_move_y); }
  void lcd_move_get_z_amount()        { _goto_manual_move_small(lcd_move_z); }
  void lcd_move_get_e_amount()        { _lcd_move_distance_menu(E_AXIS, lcd_move_e); }
  #if E_MANUAL > 1
    void lcd_move_get_e0_amount()     { _lcd_move_distance_menu(E_AXIS, lcd_move_e0); }
    void lcd_move_get_e1_amount()     { _lcd_move_distance_menu(E_AXIS, lcd_move_e1); }
    #if E_MANUAL > 2
      void lcd_move_get_e2_amount()   { _lcd_move_distance_menu(E_AXIS, lcd_move_e2); }
      #if E_MANUAL > 3
        void lcd_move_get_e3_amount() { _lcd_move_distance_menu(E_AXIS, lcd_move_e3); }
        #if E_MANUAL > 4
          void lcd_move_get_e4_amount() { _lcd_move_distance_menu(E_AXIS, lcd_move_e4); }
        #endif // E_MANUAL > 4
      #endif // E_MANUAL > 3
    #endif // E_MANUAL > 2
  #endif // E_MANUAL > 1

  /**
   *
   * "Prepare" > "Move Axis" submenu
   *
   */

  #if IS_KINEMATIC || ENABLED(NO_MOTION_BEFORE_HOMING)
    #define _MOVE_XYZ_ALLOWED (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
  #else
    #define _MOVE_XYZ_ALLOWED true
  #endif

  #if ENABLED(DELTA)
    #define _MOVE_XY_ALLOWED (current_position[Z_AXIS] <= delta_clip_start_height)
    void lcd_lower_z_to_clip_height() {
      line_to_z(delta_clip_start_height);
      lcd_synchronize();
    }
  #else
    #define _MOVE_XY_ALLOWED true
  #endif

  void lcd_move_menu() {
    START_MENU();
    MENU_BACK(MSG_PREPARE);

    if (_MOVE_XYZ_ALLOWED) {
      if (_MOVE_XY_ALLOWED) {
        MENU_ITEM(submenu, MSG_MOVE_X, lcd_move_get_x_amount);
        MENU_ITEM(submenu, MSG_MOVE_Y, lcd_move_get_y_amount);
      }
      #if ENABLED(DELTA)
        else
          MENU_ITEM(function, MSG_FREE_XY, lcd_lower_z_to_clip_height);
      #endif

      MENU_ITEM(submenu, MSG_MOVE_Z, lcd_move_get_z_amount);
    }
    else
      MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));

    #if ENABLED(SWITCHING_EXTRUDER) || ENABLED(DUAL_X_CARRIAGE)
      if (active_extruder)
        MENU_ITEM(gcode, MSG_SELECT " " MSG_E1, PSTR("T0"));
      else
        MENU_ITEM(gcode, MSG_SELECT " " MSG_E2, PSTR("T1"));
    #endif

	// // DR - added if to avoid unnecessary Extruder showing up
	// #if E_MANUAL == 1
  //   MENU_ITEM(submenu, MSG_MOVE_E, lcd_move_get_e_amount);
	// #endif
  //
  //   #if E_MANUAL > 1
  //     MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E1, lcd_move_get_e0_amount);
  //     MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E2, lcd_move_get_e1_amount);
  //     #if E_MANUAL > 2
  //       MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E3, lcd_move_get_e2_amount);
  //       #if E_MANUAL > 3
  //         MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E4, lcd_move_get_e3_amount);
  //         #if E_MANUAL > 4
  //           MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E5, lcd_move_get_e4_amount);
  //         #endif // E_MANUAL > 4
  //       #endif // E_MANUAL > 3
  //     #endif // E_MANUAL > 2
  //   #endif // E_MANUAL > 1

    END_MENU();
  }

  /**
   *
   * "Control" submenu
   *
   */

  #if HAS_LCD_CONTRAST
    void lcd_callback_set_contrast() { set_lcd_contrast(lcd_contrast); }
  #endif

  static void lcd_factory_settings() {
    settings.reset();
    lcd_completion_feedback();
  }

  #if ENABLED(EEPROM_SETTINGS)

    static void lcd_init_eeprom() {
      lcd_factory_settings();
      settings.save();
      lcd_goto_previous_menu();
    }

    static void lcd_init_eeprom_confirm() {
      START_MENU();
      MENU_BACK(MSG_CONTROL);
      MENU_ITEM(function, MSG_INIT_EEPROM, lcd_init_eeprom);
      END_MENU();
    }

  #endif

  ////////////    Trinamic stealth mode    //////////////
  #ifdef HAVE_TMC2130
  	void disable_silent_mode (void)
    {
      cli();

      uint16_t temp_current = 0 ;

      #ifdef X_IS_TMC2130
        temp_current = stepperX.getCurrent();
        stepperX.stealthChop(0);
        stepperX.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      #ifdef Y_IS_TMC2130
        temp_current = stepperY.getCurrent();
        stepperY.stealthChop(0);
        stepperY.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      // #ifdef E0_IS_TMC2130
      //   stepperE0.stealthChop(0);
      // #endif
      // #ifdef E1_IS_TMC2130
      //   stepperE1.stealthChop(0);
      // #endif

      // Stores the global flag
      silent_mode = 0;

      // Resets default acceleration and velocity values
      // Temp values to load the default from define values
      uint32_t temp_acc[] = DEFAULT_MAX_ACCELERATION;
      float temp_feed[] = DEFAULT_MAX_FEEDRATE;
      planner.max_acceleration_mm_per_s2[X_AXIS] = temp_acc[0];
      planner.max_acceleration_mm_per_s2[Y_AXIS] = temp_acc[1];
      planner.max_acceleration_mm_per_s2[E_AXIS] = temp_acc[3];
      planner.max_acceleration_mm_per_s2[E_AXIS+1] = temp_acc[4];
      planner.max_jerk[X_AXIS] = DEFAULT_XJERK;
      planner.max_jerk[Y_AXIS] = DEFAULT_YJERK;
      planner.max_jerk[Z_AXIS] = DEFAULT_ZJERK;
      planner.max_jerk[E_AXIS] = DEFAULT_EJERK;
      planner.max_feedrate_mm_s[X_AXIS] = temp_feed[0];
      planner.max_feedrate_mm_s[Y_AXIS] = temp_feed[1];
      planner.max_feedrate_mm_s[Z_AXIS] = temp_feed[2];
      planner.max_feedrate_mm_s[E_AXIS] = temp_feed[3];
      planner.max_feedrate_mm_s[E_AXIS+1] = temp_feed[3];

      // Ensures enough time for the stepper drives to stabilize
      sei();

      // Returns to machine menu
      lcd_goto_screen(beevc_machine_menu);
    }

    void enable_silent_mode (void)
    {
      cli();

      uint16_t temp_current = 0 ;

      #ifdef X_IS_TMC2130
        temp_current = stepperX.getCurrent();
        stepperX.stealthChop(1);
        stepperX.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      #ifdef Y_IS_TMC2130
        temp_current = stepperY.getCurrent();
        stepperY.stealthChop(1);
        stepperY.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      // #ifdef E0_IS_TMC2130
      //   stepperE0.stealthChop(1);
      // #endif
      // #ifdef E1_IS_TMC2130
      //   stepperE1.stealthChop(1);
      // #endif

      // Stores the global flag
      silent_mode = 1;

      // Ensures enough time for the stepper drives to stabilize
      sei();

      // Returns to machine menu
      lcd_goto_screen(beevc_machine_menu);
    }

    void enable_stealth_mode (void)
    {
      cli();

      uint16_t temp_current = 0 ;

      #ifdef X_IS_TMC2130
        temp_current = stepperX.getCurrent();
        stepperX.stealthChop(1);
        stepperX.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      #ifdef Y_IS_TMC2130
        temp_current = stepperY.getCurrent();
        stepperY.stealthChop(1);
        stepperY.setCurrent(temp_current, R_SENSE, HOLD_MULTIPLIER);
      #endif
      // #ifdef E0_IS_TMC2130
      //   stepperE0.stealthChop(1);
      // #endif
      // #ifdef E1_IS_TMC2130
      //   stepperE1.stealthChop(1);
      // #endif

      // Stores the global flag
      silent_mode = 2;

      // Lowers the acceleration and velocity values to reduce noise levels
      planner.max_acceleration_mm_per_s2[X_AXIS] = 400;
      planner.max_acceleration_mm_per_s2[Y_AXIS] = 150;
      planner.max_acceleration_mm_per_s2[E_AXIS] = 1000;
      planner.max_acceleration_mm_per_s2[E_AXIS+1] = 1000;
      planner.max_jerk[X_AXIS] = DEFAULT_XJERK/2;
      planner.max_jerk[Y_AXIS] = DEFAULT_YJERK/2;
      planner.max_jerk[Z_AXIS] = DEFAULT_ZJERK/2;
      planner.max_jerk[E_AXIS] = DEFAULT_EJERK/2;
      planner.max_feedrate_mm_s[X_AXIS] = 80;
      planner.max_feedrate_mm_s[Y_AXIS] = 80;
      planner.max_feedrate_mm_s[Z_AXIS] = 2;
      planner.max_feedrate_mm_s[E_AXIS] = 40;
      planner.max_feedrate_mm_s[E_AXIS+1] = 40;

      // Ensures enough time for the stepper drives to stabilize
      sei();

      // Returns to machine menu
      lcd_goto_screen(beevc_machine_menu);
    }
  #endif
  ///////////////////////////////////////////////////////

  #define MACHINE_SETUP_TITLE \
    START_SCREEN();\
    STATIC_ITEM(_UxGT("Self-test wizard"), true, true)

  #define MACHINE_SETUP_TITLE_WAIT \
    START_SCREEN();\
    STATIC_ITEM(_UxGT("Self-test wizard"), true, true);\
    STATIC_ITEM(_UxGT("Please wait, while"))

  #define MACHINE_SETUP_END END_SCREEN()

  #define MACHINE_SETUP_TITLE_CHOICE \
    START_MENU();\
    STATIC_ITEM(_UxGT("Self-test wizard"), true, true)

  #define MACHINE_SETUP_END_CHOICE END_MENU()

  #define MACHINE_SETUP_WAIT_FOR(N) \
    STATIC_ITEM(_UxGT(N));\
    STATIC_ITEM(_UxGT("Please wait for"));\
    STATIC_ITEM(_UxGT("the process"));\
    STATIC_ITEM(_UxGT("to finish"))

  #define MACHINE_SETUP_CHECK_MANUAL \
    STATIC_ITEM(_UxGT(" "));\
    STATIC_ITEM(_UxGT("(scroll to read more)"));\
    STATIC_ITEM(_UxGT("A error occured"));\
    STATIC_ITEM(_UxGT("during the self-test"));\
    STATIC_ITEM(_UxGT("Please, check the"));\
    STATIC_ITEM(_UxGT("\"Error Codes\" chapter"));\
    STATIC_ITEM(_UxGT("on the User Manual."));\
    STATIC_ITEM(_UxGT(" "))

  #define MACHINE_SETUP_SHUTDOWN \
    STATIC_ITEM(_UxGT(" "));\
    STATIC_ITEM(_UxGT("Please shutdown the"));\
    STATIC_ITEM(_UxGT("printer, fix the"));\
    STATIC_ITEM(_UxGT("issue and try again."))

  void beevc_machine_setup_EEPROM_updated() {
    MACHINE_SETUP_TITLE;
    STATIC_ITEM(_UxGT("EEPROM structure was"));
    STATIC_ITEM(_UxGT("updated, settings"));
    STATIC_ITEM(_UxGT("were reset."));
    STATIC_ITEM(_UxGT("Press to continue."));
    MACHINE_SETUP_END;
  }

  void beevc_machine_setup_screen_start() {
    MACHINE_SETUP_TITLE;
    STATIC_ITEM(_UxGT("Welcome to the B2X300"));
    STATIC_ITEM(_UxGT("self-test wizard!"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("(scroll to read more)"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("I will guide you"));
    STATIC_ITEM(_UxGT("through the initial"));
    STATIC_ITEM(_UxGT("printer setup."));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Click to continue."));
    MACHINE_SETUP_END;
  }

  void beevc_machine_setup_screen_hotend_start() {
    MACHINE_SETUP_TITLE;
    STATIC_ITEM(_UxGT("The machine will now"));
    STATIC_ITEM(_UxGT("test the hotends."));
    MACHINE_SETUP_END;
  }

  void beevc_machine_setup_screen_hotend() {
    MACHINE_SETUP_TITLE;

    LCD_PRINT_EXT_TEMP();

    STATIC_ITEM(_UxGT("Status:  heating test"));
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Please wait."));

    MACHINE_SETUP_END;
  }

void beevc_machine_setup_screen_hotend_ok() {
    MACHINE_SETUP_TITLE;

    LCD_PRINT_EXT_TEMP_STABLE();

    STATIC_ITEM(_UxGT("Status: OK!"), false,false );
    STATIC_ITEM(_UxGT(" "));
    STATIC_ITEM(_UxGT("Click to continue."));

    MACHINE_SETUP_END;
  }

void beevc_machine_setup_screen_hotend_cooling() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT(" "));

  u8g.setPrintPos(0, 26);
  u8g.print("Extruder ");
  u8g.print(active_extruder+1);
  u8g.print(": ");
  if(round(thermalManager.degHotend(active_extruder)) <100)
    u8g.print(" ");
  u8g.print(round(thermalManager.degHotend(active_extruder)));
  u8g.print("/");
  u8g.print(round(40));;
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
  u8g.print("C");

  STATIC_ITEM(_UxGT("Status:  cooling down"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please wait."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_hotbed_ok() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT(" "));

  u8g.setPrintPos(0, 26);
  u8g.print("Heated bed:   ");
  u8g.print(50);
  u8g.print("/");
  u8g.print(50);
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
  u8g.print("C");

  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_hotbed() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT(" "));

  u8g.setPrintPos(0, 26);
  u8g.print("Heated bed:   ");
  u8g.print(round(thermalManager.degBed()));
  u8g.print("/");
  u8g.print(round(thermalManager.degTargetBed()));
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
  u8g.print("C");

  STATIC_ITEM(_UxGT("Status:  heating test"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please wait."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotend_hot() {
MACHINE_SETUP_TITLE;

STATIC_ITEM(_UxGT("Extruder already reading as hot"));
STATIC_ITEM(_UxGT("Please verify the"));
STATIC_ITEM(_UxGT("thermistor and"));
STATIC_ITEM(_UxGT("extruder output by"));
MACHINE_SETUP_CHECK_MANUAL;
STATIC_ITEM(_UxGT("XX"));
MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotend_timeout() {
  MACHINE_SETUP_TITLE;

  char string_to_print[25];
  strcpy(string_to_print, "Extruder ");
  strcat(string_to_print, i8tostr1(active_extruder+1));
  strcat(string_to_print, ":  xx/100\x09\u0043");
  STATIC_ITEM("", false,false, string_to_print);

  if(active_extruder){
    STATIC_ITEM(_UxGT("Status: EC23 - ERROR!"));
  }
  else{
    STATIC_ITEM(_UxGT("Status: EC13 - ERROR!"));
  }

  MACHINE_SETUP_CHECK_MANUAL;

  if(active_extruder){
    STATIC_ITEM(_UxGT("Error code: EC23"));
  }
  else{
    STATIC_ITEM(_UxGT("Error code: EC13"));
  }

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotend_sensor() {
  MACHINE_SETUP_TITLE;

  char string_to_print[25];
  strcpy(string_to_print, "Extruder ");
  strcat(string_to_print, i8tostr1(active_extruder+1));
  strcat(string_to_print, ":  xx/100\x09\u0043");
  STATIC_ITEM("", false,false, string_to_print);

  if(active_extruder){
    STATIC_ITEM(_UxGT("Status: EC21 - ERROR!"));
  }
  else{
    STATIC_ITEM(_UxGT("Status: EC11 - ERROR!"));
  }

  MACHINE_SETUP_CHECK_MANUAL;

  if(active_extruder){
    STATIC_ITEM(_UxGT("Error code: EC21"));
  }
  else{
    STATIC_ITEM(_UxGT("Error code: EC11"));
  }

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotend_sensor_swaped() {
  MACHINE_SETUP_TITLE;

  char string_to_print[25];
  strcpy(string_to_print, "Extruder ");
  strcat(string_to_print, i8tostr1(active_extruder+1));
  strcat(string_to_print, ":  xx/100\x09\u0043");
  STATIC_ITEM("", false,false, string_to_print);

  if(active_extruder){
    STATIC_ITEM(_UxGT("Status: EC22 - ERROR!"));
  }
  else{
    STATIC_ITEM(_UxGT("Status: EC12 - ERROR!"));
  }

  MACHINE_SETUP_CHECK_MANUAL;

  if(active_extruder){
    STATIC_ITEM(_UxGT("Error code: EC22"));
  }
  else{
    STATIC_ITEM(_UxGT("Error code: EC12"));
  }

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotbed_hot() {
MACHINE_SETUP_TITLE;
STATIC_ITEM(_UxGT("Hotbed already reading as hot"));
STATIC_ITEM(_UxGT("Please verify the"));
STATIC_ITEM(_UxGT("thermistor and"));
STATIC_ITEM(_UxGT("hotbed output by"));
MACHINE_SETUP_CHECK_MANUAL;
STATIC_ITEM(_UxGT("XX"));
MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotbed_timeout() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM("Heated bed:    xx/50\x09\u0043");

  STATIC_ITEM(_UxGT("Status: EC33 - ERROR!"));

  MACHINE_SETUP_CHECK_MANUAL;

  STATIC_ITEM(_UxGT("Error code: EC33"));

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_hotbed_sensor() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM("Heated bed:    xx/50\x09\u0043");

  STATIC_ITEM(_UxGT("Status: EC31 - ERROR!"));

  MACHINE_SETUP_CHECK_MANUAL;

  STATIC_ITEM(_UxGT("Error code: EC31"));

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_blower_start() {
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("The machine will now"));
  STATIC_ITEM(_UxGT("test the blower fan."));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_blower_test_ok() {
  // Sets test flag as complete
  beevc_continue = true ;

  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Blower fan:        OK"));
  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_error_blower() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Blower fan:      ---%"));
  STATIC_ITEM(_UxGT("Status: EC41 - ERROR!"));

  MACHINE_SETUP_CHECK_MANUAL;

  STATIC_ITEM(_UxGT("Error code: EC41"));

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_blower_test() {
  MACHINE_SETUP_TITLE_CHOICE;
  STATIC_ITEM(_UxGT("Is the blower active?"));
  MENU_ITEM_MIX(submenu, _UxGT(" - Yes"), beevc_machine_setup_screen_blower_test_ok);
  MENU_ITEM_MIX(submenu, _UxGT(" - No"), beevc_machine_setup_screen_error_blower);
  END_SCREEN();
}

void beevc_machine_setup_screen_trinamic_start() {
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("The machine will now"));
  STATIC_ITEM(_UxGT("test the trinamic"));
  STATIC_ITEM(_UxGT("stepper drivers. "));
  STATIC_ITEM(_UxGT("Click to continue."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_trinamic_ok() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Trinamic drivers: OK!"));
  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_trinamic_error() {
  MACHINE_SETUP_TITLE;

  // X axis
  if(!(trinamic_ok & 0x01)){
    STATIC_ITEM(_UxGT("X Stepper driver: NOK"));
  }

  // Y axis
  if(!(trinamic_ok & 0x02)){
    STATIC_ITEM(_UxGT("Y Stepper driver: NOK"));
  }

  // Z axis
  if(!(trinamic_ok & 0x04)){
    STATIC_ITEM(_UxGT("Z Stepper driver: NOK"));
  }

  // E1 axis
  if(!(trinamic_ok & 0x08)){
    STATIC_ITEM(_UxGT("E1 Stepper driver:NOK"));
  }

  // E2 axis
  if(!(trinamic_ok & 0x10)){
    STATIC_ITEM(_UxGT("E2 Stepper driver:NOK"));
  }


  STATIC_ITEM(_UxGT("Status: EC51 - ERROR!"));

  MACHINE_SETUP_CHECK_MANUAL;

  STATIC_ITEM(_UxGT("Error code: EC51"));

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_powerloss_start() {
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("The machine will now"));
  STATIC_ITEM(_UxGT("test the powerloss"));
  STATIC_ITEM(_UxGT("detection board. "));
  STATIC_ITEM(_UxGT("Click to continue."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_powerloss_ok() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Powerloss:         OK"));
  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_powerloss_error() {
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Powerloss:        NOK"));

  STATIC_ITEM(_UxGT("Status: EC61 - ERROR!"));

  MACHINE_SETUP_CHECK_MANUAL;

  STATIC_ITEM(_UxGT("Error code: EC61"));

  MACHINE_SETUP_SHUTDOWN;
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_sensorless_homing(){
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Sensorless-homing: XY"));
  STATIC_ITEM(_UxGT("Status: Homing XY"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please wait."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_sensorless_homing_complete(){
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Sensorless-homing: XY"));
  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));

  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_home_complete(){
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("Leveling done!"));
  STATIC_ITEM(_UxGT("Moving to calibration"));
  STATIC_ITEM(_UxGT("position"));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_explain(){
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("Please, adjust the Z"));
  STATIC_ITEM(_UxGT("height by turning the"));
  STATIC_ITEM(_UxGT("LCD knob."));
  STATIC_ITEM(_UxGT("(scroll to read more)"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Place a paper sheet "));
  STATIC_ITEM(_UxGT("between the nozzle "));
  STATIC_ITEM(_UxGT("and the print surface"));
  STATIC_ITEM(_UxGT(". Adjust until the "));
  STATIC_ITEM(_UxGT("paper is neither free"));
  STATIC_ITEM(_UxGT("nor completely stuck"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please, check the"));
  STATIC_ITEM(_UxGT("\"Other Codes\" chapter"));
  STATIC_ITEM(_UxGT("on the User Manual"));
  STATIC_ITEM(_UxGT("for more information."));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Proc. code: OC01"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_homing(){
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("Nozzle height: -----"));
  STATIC_ITEM(_UxGT("Status: finding mesh"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please wait."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_moving(){
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT("Nozzle height: -----"));
  STATIC_ITEM(_UxGT("Status: moving"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Please wait."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_complete(){
  MACHINE_SETUP_TITLE;
  STATIC_ITEM(_UxGT(" "));
  lcd_implementation_drawmenu_setting_edit_generic(false, 1,PSTR("Nozzle height"),ftostr42sign(zprobe_zoffset));
  STATIC_ITEM(_UxGT("Status: OK!"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to continue."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_complete(){
  MACHINE_SETUP_TITLE;

  STATIC_ITEM(_UxGT("Self-test completed"));
  STATIC_ITEM(_UxGT("successfully."));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("(scroll to read more)"));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Extruder 1: OK"));
  STATIC_ITEM(_UxGT("Extruder 2: OK"));
  STATIC_ITEM(_UxGT("Heated bed: OK"));
  STATIC_ITEM(_UxGT("Blower Fan: OK"));
  STATIC_ITEM(_UxGT("Homing X:"), false, false, i8tostr3(thermalManager.sg2_homing_x_calibration));
  STATIC_ITEM(_UxGT("Homing Y:"), false, false, i8tostr3(thermalManager.sg2_homing_y_calibration));
  STATIC_ITEM(_UxGT("Homing Z: OK"));
  STATIC_ITEM(_UxGT("Nozzle height: "), false, false, ftostr42sign(zprobe_zoffset));
  STATIC_ITEM(_UxGT(" "));
  STATIC_ITEM(_UxGT("Click to exit."));
  MACHINE_SETUP_END;
}

void beevc_machine_setup_screen_set_offset_calibrate(){
  if (lcd_clicked)
    {
      zprobe_zoffset = (current_position[Z_AXIS] + zprobe_zoffset);
      lcd_completion_feedback(settings.save());
      z_offset_finished = true;
      lcd_goto_screen(beevc_machine_setup_screen_set_offset_complete);
    }

  ENCODER_DIRECTION_NORMAL();

  if (encoderPosition) {
    refresh_cmd_timeout();

    float min = current_position[Z_AXIS] - 1000,
          max = current_position[Z_AXIS] + 1000;

    // Get the new position
    current_position[Z_AXIS] -= float((int32_t)encoderPosition) * 0.05;


    // Limit only when trying to move towards the limit
    if ((int32_t)encoderPosition < 0) NOLESS(current_position[Z_AXIS], min);
    if ((int32_t)encoderPosition > 0) NOMORE(current_position[Z_AXIS], max);

    manual_move_to_current(Z_AXIS);

    encoderPosition = 0;
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
  }
  if (lcdDrawUpdate) {
    MACHINE_SETUP_TITLE;

    lcd_implementation_drawmenu_setting_edit_generic(false, 1,PSTR("Nozzle height"),ftostr42sign((current_position[Z_AXIS] + zprobe_zoffset)));
    lcd_implementation_drawmenu_static(2,PSTR("Status: please adjust"));
    lcd_implementation_drawmenu_static(4,PSTR("Click to save.       "));

    MACHINE_SETUP_END;
  }
}

void beevc_machine_setup_sensorless_homing (){
  beevc_buzz();

  // Show info screen
  lcd_goto_screen(beevc_machine_setup_screen_sensorless_homing);

  // Waits for click or timeout
  beevc_wait(1000);

  // Starts variables to force calibration
  calibrating_sensorless_homing_x = true;
  calibrating_sensorless_homing_y = true;

  // Start sensorless homing calibration procedure
  gcode_M918();

  //Display sensorless homing ok screen
  lcd_goto_screen(beevc_machine_setup_screen_sensorless_homing_complete);

  //Beep
  beevc_buzz();

  //Wait for 5sec or click
  beevc_wait(5000);
}

void beevc_machine_setup_measure_xy(){
}

void beevc_machine_setup_set_offset(){
  // Initializes required variable
  z_offset_finished = false;

  // Show finding mesh screen
  lcd_goto_screen(beevc_machine_setup_screen_set_offset_homing);

  // Waits for click or timeout
  beevc_wait(1000);

  // Homes and autoleves axes
  gcode_G28(1);
  gcode_G29();

  // Show moving screen
  lcd_goto_screen(beevc_machine_setup_screen_set_offset_moving);

  // Moves the carriage and bed to the offset adjust position
  do_blocking_move_to_xy(((X_BED_SIZE) / 2),((Y_BED_SIZE) / 2), 120);

  // Lowers Z axis
  current_position[Z_AXIS] = (float)(round((current_position[Z_AXIS]-8)*10))/10;
  manual_move_to_current(Z_AXIS,0);

  // Waits a few seconds to allow movement to finish
  uint32_t temp_time = millis() + 3200;
  while(millis() < temp_time){
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    idle(true);
  }

  // Shows the help screen
  lcd_goto_screen(beevc_machine_setup_screen_set_offset_explain);
  beevc_wait(20000);

  // Shows the leveling screen
  lcd_goto_screen(beevc_machine_setup_screen_set_offset_calibrate);

  // Waits for conclusion of Adjustment
  while(!z_offset_finished){
    idle(true);
  }

  //Beep
  beevc_buzz();

  //Wait for 5sec or click
  beevc_wait(5000);
}

void beevc_machine_setup_test_hotend (uint8_t extruder){
  // Time variable
  uint32_t duration = 0, timeout =0;

  active_extruder = extruder-1;

  // Check for disconnected thermistor
  if(thermalManager.degHotend(active_extruder) <= -14){
    lcd_goto_screen(beevc_machine_setup_screen_error_hotend_sensor);
    while(1){
      idle(true);
    }
  }

  // Waits for the hotend to cool down enough to read the heating time
  thermalManager.setTargetHotend(0, active_extruder);
  duration = millis();
  // Blower on to help cooling
  fanSpeeds[0] = 255;
  lcd_goto_screen(beevc_machine_setup_screen_hotend_cooling);
  while(thermalManager.degHotend(active_extruder) > 40){
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    idle(true);
    if(1){
      idle(true);
    }
  }
  // Blower off
  fanSpeeds[0] = 0;

  //Set E destination temperature
  thermalManager.setTargetHotend(100, active_extruder);
  //Start counting time
  duration = millis();
  //Sets a timeout of 60sec
  timeout = duration + 60000;

  lcd_goto_screen(beevc_machine_setup_screen_hotend);

  // Stores idle temperature for the other extruder
  uint16_t idle_temp = thermalManager.degHotend(extruder == 0 ? 1 : 0)+10;

  while(thermalManager.degHotend(active_extruder) < thermalManager.degTargetHotend(active_extruder)){

    // Checks for swapped hotend sensor/resistors
      if (thermalManager.degHotend(extruder == 1 ? 1 : 0) > idle_temp){
        lcd_goto_screen(beevc_machine_setup_screen_error_hotend_sensor_swaped);

        // Disable heating
        thermalManager.setTargetHotend(0, active_extruder);

        // Loops to show error message and require a reset
        while(1){
          idle(true);
        }

        break;
      }


    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    idle(true);

    if (millis() > timeout){
      lcd_goto_screen(beevc_machine_setup_screen_error_hotend_timeout);
      while(1){
        idle(true);
      }
    }
  }


  //Beep
  beevc_buzz();

  //Display Hotbed ok screen
  lcd_goto_screen(beevc_machine_setup_screen_hotend_ok);

  //Wait for 5sec or click
  beevc_wait(5000);

  //Disable E heating
  thermalManager.setTargetHotend(0, active_extruder);

}

void beevc_machine_setup_test_hotbed (){
  // Time variable
  uint32_t duration = 0, timeout =0;

  // Check for disconnected thermistor
  if(thermalManager.degBed() <= -14){
    lcd_goto_screen(beevc_machine_setup_screen_error_hotbed_sensor);
    while(1){
      idle(true);
    }
  }
  //Set bed destination temperature
  thermalManager.setTargetBed(50);
  //Start counting time
  duration = millis();
  //Sets a timeout of 180sec
  timeout = duration + 180000;

    lcd_goto_screen(beevc_machine_setup_screen_hotbed);

  while(thermalManager.degBed() < (thermalManager.degTargetBed()-2)){
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    idle(true);

    if (millis() > timeout){
      lcd_goto_screen(beevc_machine_setup_screen_error_hotbed_timeout);
      while(1){
        idle(true);
      }
    }
  }

  //Disable Hotbed
  thermalManager.setTargetBed(0);

  //Beep
  beevc_buzz();

  //Display Hotbed ok screen
  lcd_goto_screen(beevc_machine_setup_screen_hotbed_ok);

  //Wait for 5sec or click
  beevc_wait(5000);
}

void beevc_machine_setup_test_blower (){
    lcd_goto_screen(beevc_machine_setup_screen_blower_start);

    beevc_wait(5000);
    fanSpeeds[0] = 255;
    beevc_continue = 0;

    lcd_goto_screen(beevc_machine_setup_screen_blower_test);

    while(!beevc_continue){
      idle(true);
    }

    fanSpeeds[0] = 0;

    //Beep
    beevc_buzz();

    //Wait for 5sec or click
    beevc_wait(5000);
}

void beevc_machine_setup_test_trinamic (){
    // Displays Trinamic test start screen
    lcd_goto_screen(beevc_machine_setup_screen_trinamic_start);

    // Waits for 5s or click
    beevc_wait(5000);

    // Re-initializes variable
    trinamic_ok = 0;

    // Temporary variable to store default data
    uint32_t trinamic_data = 0;

    // Tests each axis one at a time storing the result if positive
    // X
      trinamic_data = stepperX.GCONF(); 
      stepperX.GCONF(0b101010101010101010);
      if(stepperX.GCONF() == 0b101010101010101010){
        stepperX.GCONF(trinamic_data);
        if(stepperX.GCONF() == trinamic_data)
          trinamic_ok |= 0x01;
      }

    // Y
      trinamic_data = stepperY.GCONF(); 
      stepperY.GCONF(0b101010101010101010);
      if(stepperY.GCONF() == 0b101010101010101010){
        stepperY.GCONF(trinamic_data);
        if(stepperY.GCONF() == trinamic_data)
          trinamic_ok |= 0x02;
      }

    // Z
      trinamic_data = stepperZ.GCONF(); 
      stepperZ.GCONF(0b101010101010101010);
      if(stepperZ.GCONF() == 0b101010101010101010){
        stepperZ.GCONF(trinamic_data);
        if(stepperZ.GCONF() == trinamic_data)
          trinamic_ok |= 0x04;
      }

    // E0
      trinamic_data = stepperE0.GCONF(); 
      stepperE0.GCONF(0b101010101010101010);
      if(stepperE0.GCONF() == 0b101010101010101010){
        stepperE0.GCONF(trinamic_data);
        if(stepperE0.GCONF() == trinamic_data)
          trinamic_ok |= 0x08;
      }

    // E1
      trinamic_data = stepperE1.GCONF(); 
      stepperE1.GCONF(0b101010101010101010);
      if(stepperE1.GCONF() == 0b101010101010101010){
        stepperE1.GCONF(trinamic_data);
        if(stepperE1.GCONF() == trinamic_data)
          trinamic_ok |= 0x10;
      }

    #ifdef SERIAL_DEBUG
      SERIAL_PROTOCOLLNPAIR("X = ", stepperX.test_connection());
      SERIAL_PROTOCOLLNPAIR("Y = ", stepperY.test_connection());
      SERIAL_PROTOCOLLNPAIR("Z = ", stepperZ.test_connection());
      SERIAL_PROTOCOLLNPAIR("E1 = ", stepperE0.test_connection());
      SERIAL_PROTOCOLLNPAIR("E2 = ", stepperE1.test_connection());

      SERIAL_PROTOCOLLNPAIR("X = ", (trinamic_ok & 0x01));
      SERIAL_PROTOCOLLNPAIR("Y = ", (trinamic_ok & 0x02));
      SERIAL_PROTOCOLLNPAIR("Z = ", (trinamic_ok & 0x04));
      SERIAL_PROTOCOLLNPAIR("E1 = ", (trinamic_ok & 0x08));
      SERIAL_PROTOCOLLNPAIR("E2 = ", (trinamic_ok & 0x10));

      SERIAL_PROTOCOLLNPAIR("Value = ", trinamic_ok);
    #endif

    // If any axis failed display error screen
    if (trinamic_ok != 0x1F){
      lcd_goto_screen(beevc_machine_setup_screen_trinamic_error);
      while(1){
        idle(true);
      }
    }

    //Beep
    beevc_buzz();

    // Display ok screen
    lcd_goto_screen(beevc_machine_setup_screen_trinamic_ok);

    //Wait for 5sec or click
    beevc_wait(5000);
}

void beevc_machine_setup_test_powerloss (){
    // Displays Trinamic test start screen
    lcd_goto_screen(beevc_machine_setup_screen_powerloss_start);

    // Waits for 5s or click
    beevc_wait(5000);

    // Tests if the powerloss pin is ofline as it should be
    if (digitalRead(11) != 0){
      lcd_goto_screen(beevc_machine_setup_screen_powerloss_error);
      while(1){
        idle(true);
      }
    }

    //Beep
    beevc_buzz();

    // Display ok screen
    lcd_goto_screen(beevc_machine_setup_screen_powerloss_ok);

    //Wait for 5sec or click
    beevc_wait(5000);
}

  /**
   *  BEEVC
   * "Machine settings" > "Self-test Wizard"
   */
  void beevc_machine_setup() {

    // Resets the printer to the default configuration
    lcd_init_eeprom();

    // Clear LCD
    lcd_implementation_clear();

    // Shows EEPROM updated screen
    if(toCalibrate == 1) {
      lcd_goto_screen(beevc_machine_setup_EEPROM_updated);
      beevc_wait_click();
    }

    // Start screen

    lcd_goto_screen(beevc_machine_setup_screen_start);

    // Wait for click
    beevc_wait_click();

    // Verify Hotend connections
    lcd_goto_screen(beevc_machine_setup_screen_hotend_start);

    // Wait for while to allow the user to read the message
    beevc_wait(5000);

    // Verify E1 connection
      // Test E1
      beevc_machine_setup_test_hotend(1);

    // Verify E2 connection
      // Test E2
      beevc_machine_setup_test_hotend(2);

    // Verify hotbed connections
    beevc_machine_setup_test_hotbed();

    // Verify blower connection
    beevc_machine_setup_test_blower();

    // Test Trinamic stepper drivers
    beevc_machine_setup_test_trinamic();

    // Test powerloss detection pin
    beevc_machine_setup_test_powerloss();

    // Calibrate sensorless homing
    beevc_machine_setup_sensorless_homing();

    //Measure XY axes size
    //beevc_machine_setup_measure_xy();

    // Set Z offset
    beevc_machine_setup_set_offset();

    // Fine tune offset w/test print

    // PID Calibration

    // Clears Self-test Wizard flag
    gcode_M721();

    // Stores EEPROM data
    lcd_store_settings();

    // Enqueue a XY home
    enqueue_and_echo_commands_P(PSTR("G28 X Y"));

    // Shows complete screen
    lcd_goto_screen(beevc_machine_setup_screen_complete);

    // Waits for click
    beevc_wait_click();

    // Return to status screen
    lcd_return_to_status();

  }

  ///////////////////////////////////////////////////////

  /**
   *  BEEVC
   * "Machine settings" > "Save settings" submenu
   */
  void beevc_machine_save_confirm_go_to_status() {
      lcd_store_settings();
      lcd_return_to_status();
  }

  void beevc_machine_save_confirm() {
    START_MENU();
    STATIC_ITEM("Save settings", true, true);
    MENU_BACK(MSG_MAIN);

    MENU_ITEM(function, _UxGT("Confirm"), beevc_machine_save_confirm_go_to_status);

    END_MENU();
  }

  /**
   *  BEEVC
   * "Machine settings" > "Save settings" submenu
   */
  void beevc_machine_reset_confirm_go_to_status() {
       lcd_init_eeprom();
       lcd_return_to_status();
  }

  void beevc_machine_reset_confirm() {
    START_MENU();
    STATIC_ITEM("Reset settings", true, true);
    MENU_BACK(MSG_MAIN);

    MENU_ITEM(function, _UxGT("Confirm"), beevc_machine_reset_confirm_go_to_status);

    END_MENU();
  }

  /**
   *  BEEVC
   * "Machine settings" > "Mode" submenu
   */
  void beevc_machine_mode() {
    START_MENU();
    // MENU_BACK(MSG_MAIN);
    MENU_ITEM(function, _UxGT("Normal mode"), disable_silent_mode);
    MENU_ITEM(function, _UxGT("Silent mode"), enable_silent_mode);
    MENU_ITEM(function, _UxGT("Stealth mode"), enable_stealth_mode);
    END_MENU();
  }


  /**
   *  BEEVC
   * "Machine settings" submenu
   */
  void beevc_machine_menu() {
    START_MENU();
    MENU_BACK(MSG_MAIN);

    #ifdef HAVE_TMC2130
      if (silent_mode == 1)
        MENU_ITEM(submenu, _UxGT("Mode:       Silent "), beevc_machine_mode);
      else if (silent_mode == 2)
        MENU_ITEM(submenu, _UxGT("Mode:       Stealth"), beevc_machine_mode);
      else
        MENU_ITEM(submenu, _UxGT("Mode:       Normal "), beevc_machine_mode);
    #endif

    MENU_ITEM(submenu, MSG_TEMPERATURE, beevc_machine_temperature_menu);

    MENU_ITEM(submenu, MSG_MOTION, beevc_machine_motion_menu);

    MENU_ITEM(function, _UxGT("Self-test wizard"), beevc_machine_setup);

    MENU_ITEM(submenu, MSG_STORE_EEPROM, beevc_machine_save_confirm);

    MENU_ITEM(submenu, _UxGT("Reset settings"), beevc_machine_reset_confirm);

    END_MENU();
  }

  void lcd_control_menu() {
    START_MENU();
    MENU_BACK(MSG_MAIN);

    #ifdef HAVE_TMC2130
      if (silent_mode == 1)
        MENU_ITEM(function, _UxGT("Enable Silent+ mode"), enable_stealth_mode);
      else if (silent_mode == 2)
        MENU_ITEM(function, _UxGT("Disable Silent mode"), disable_silent_mode);
      else
        MENU_ITEM(function, _UxGT("Enable Silent mode"), enable_silent_mode);

        MENU_ITEM(submenu,  _UxGT("Trinamic settings"), beevc_trinamic_settings_menu);
    #endif


    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
    MENU_ITEM(submenu, MSG_FILAMENT, lcd_control_filament_menu);

    #if HAS_LCD_CONTRAST
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_CONTRAST, &lcd_contrast, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX, lcd_callback_set_contrast, true);
    #endif
    #if ENABLED(FWRETRACT)
      MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
    #endif
    #if ENABLED(DAC_STEPPER_CURRENT)
      MENU_ITEM(submenu, MSG_DRIVE_STRENGTH, lcd_dac_menu);
    #endif
    #if HAS_MOTOR_CURRENT_PWM
      MENU_ITEM(submenu, MSG_DRIVE_STRENGTH, lcd_pwm_menu);
    #endif

    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
      MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
    #endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, lcd_factory_settings);
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(submenu, MSG_INIT_EEPROM, lcd_init_eeprom_confirm);
    #endif

    END_MENU();
  }

  /**
   *
   * "Temperature" submenu
   *
   */

  #if ENABLED(PID_AUTOTUNE_MENU)

    #if ENABLED(PIDTEMP)
      int16_t autotune_temp[HOTENDS] = ARRAY_BY_HOTENDS1(175);
    #endif

    #if ENABLED(PIDTEMPBED)
      int16_t autotune_temp_bed = 70;
    #endif

    void _lcd_autotune(int16_t e) {
      char cmd[30];
      sprintf_P(cmd, PSTR("M303 U1 E%i S%i"), e,
        #if HAS_PID_FOR_BOTH
          e < 0 ? autotune_temp_bed : autotune_temp[e]
        #elif ENABLED(PIDTEMPBED)
          autotune_temp_bed
        #else
          autotune_temp[e]
        #endif
      );
      enqueue_and_echo_command(cmd);
    }

  #endif //PID_AUTOTUNE_MENU

  #if ENABLED(PIDTEMP)

    // Helpers for editing PID Ki & Kd values
    // grab the PID value out of the temp variable; scale it; then update the PID driver
    void copy_and_scalePID_i(int16_t e) {
      #if DISABLED(PID_PARAMS_PER_HOTEND) || HOTENDS == 1
        UNUSED(e);
      #endif
      PID_PARAM(Ki, e) = scalePID_i(raw_Ki);
      thermalManager.updatePID();
    }
    void copy_and_scalePID_d(int16_t e) {
      #if DISABLED(PID_PARAMS_PER_HOTEND) || HOTENDS == 1
        UNUSED(e);
      #endif
      PID_PARAM(Kd, e) = scalePID_d(raw_Kd);
      thermalManager.updatePID();
    }
    #define _DEFINE_PIDTEMP_BASE_FUNCS(N) \
      void copy_and_scalePID_i_E ## N() { copy_and_scalePID_i(N); } \
      void copy_and_scalePID_d_E ## N() { copy_and_scalePID_d(N); }

    #if ENABLED(PID_AUTOTUNE_MENU)
      #define DEFINE_PIDTEMP_FUNCS(N) \
        _DEFINE_PIDTEMP_BASE_FUNCS(N); \
        void lcd_autotune_callback_E ## N() { _lcd_autotune(N); } typedef void _pid_##N##_void
    #else
      #define DEFINE_PIDTEMP_FUNCS(N) _DEFINE_PIDTEMP_BASE_FUNCS(N) typedef void _pid_##N##_void
    #endif

    DEFINE_PIDTEMP_FUNCS(0);
    #if ENABLED(PID_PARAMS_PER_HOTEND)
      #if HOTENDS > 1
        DEFINE_PIDTEMP_FUNCS(1);
        #if HOTENDS > 2
          DEFINE_PIDTEMP_FUNCS(2);
          #if HOTENDS > 3
            DEFINE_PIDTEMP_FUNCS(3);
            #if HOTENDS > 4
              DEFINE_PIDTEMP_FUNCS(4);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
    #endif // PID_PARAMS_PER_HOTEND

  #endif // PIDTEMP

  /**
   *  BEEVC
   *
   * "Machine settings" > "Filament" > "Advanced Settings" submenu
   *
   */
  void beevc_machine_temperature_advanced_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    MENU_ITEM(submenu, MSG_PREHEAT_1, lcd_preheat_m1_menu);
    MENU_ITEM(submenu, MSG_PREHEAT_2, lcd_preheat_m2_menu);
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE " " MSG_E1, &autotune_temp[0], 175, heater_maxtemp[0] - 15, lcd_autotune_callback_E0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE " " MSG_E2, &autotune_temp[1], 175, heater_maxtemp[1] - 15, lcd_autotune_callback_E1);

    _PID_BASE_MENU_ITEMS(" " MSG_E1, 0);
    _PID_BASE_MENU_ITEMS(" " MSG_E2, 1);

  END_MENU();
  }

  /**
   *  BEEVC
   *
   * "Machine settings" > "Filament" submenu
   *
   */
  void beevc_machine_temperature_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    // Nozzle:
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_TEMPERATURE MSG_SE2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);

    // Bed:
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3,MSG_TEMPERATURE MSG_SBED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);

    // Fan Speed:
    MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED FAN_SPEED_1_SUFFIX, &fanSpeeds[0], 0, 255);

    // Makes sure the fan isn't set for a speed at which it can't spin
    if (fanSpeeds[0] > 0 && fanSpeeds[0] < B2X300_MIN_FAN)
      fanSpeeds[0]=B2X300_MIN_FAN;

    // Advanced options
    MENU_ITEM(submenu,  _UxGT("Advanced settings"), beevc_machine_temperature_advanced_menu);


    END_MENU();
  }


  /**
   *
   * "Control" > "Temperature" submenu
   *
   */
  void lcd_control_temperature_menu() {
    START_MENU();

    //
    // ^ Control
    //
    MENU_BACK(MSG_CONTROL);

    //
    // Nozzle:
    // Nozzle [1-5]:
    //
    #if HOTENDS == 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #else // HOTENDS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
      #if HOTENDS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &thermalManager.target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
        #if HOTENDS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &thermalManager.target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
          #if HOTENDS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N5, &thermalManager.target_temperature[4], 0, HEATER_4_MAXTEMP - 15, watch_temp_callback_E4);
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if HAS_TEMP_BED
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);
    #endif

    //
    // Fan Speed:
    //
    #if FAN_COUNT > 0
      #if HAS_FAN0
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED FAN_SPEED_1_SUFFIX, &fanSpeeds[0], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED FAN_SPEED_1_SUFFIX, &new_fanSpeeds[0], 3, 255);
        #endif
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED " 2", &new_fanSpeeds[1], 3, 255);
        #endif
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
        #if ENABLED(EXTRA_FAN_SPEED)
          MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_EXTRA_FAN_SPEED " 3", &new_fanSpeeds[2], 3, 255);
        #endif
      #endif
    #endif // FAN_COUNT > 0

    //
    // Autotemp, Min, Max, Fact
    //
    #if ENABLED(AUTOTEMP) && (TEMP_SENSOR_0 != 0)
      MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &planner.autotemp_enabled);
      MENU_ITEM_EDIT(float3, MSG_MIN, &planner.autotemp_min, 0, HEATER_0_MAXTEMP - 15);
      MENU_ITEM_EDIT(float3, MSG_MAX, &planner.autotemp_max, 0, HEATER_0_MAXTEMP - 15);
      MENU_ITEM_EDIT(float32, MSG_FACTOR, &planner.autotemp_factor, 0.0, 1.0);
    #endif

    //
    // PID-P, PID-I, PID-D, PID-C, PID Autotune
    // PID-P E1, PID-I E1, PID-D E1, PID-C E1, PID Autotune E1
    // PID-P E2, PID-I E2, PID-D E2, PID-C E2, PID Autotune E2
    // PID-P E3, PID-I E3, PID-D E3, PID-C E3, PID Autotune E3
    // PID-P E4, PID-I E4, PID-D E4, PID-C E4, PID Autotune E4
    // PID-P E5, PID-I E5, PID-D E5, PID-C E5, PID Autotune E5
    //
    #if ENABLED(PIDTEMP)

      #define _PID_BASE_MENU_ITEMS(ELABEL, eindex) \
        raw_Ki = unscalePID_i(PID_PARAM(Ki, eindex)); \
        raw_Kd = unscalePID_d(PID_PARAM(Kd, eindex)); \
        MENU_ITEM_EDIT(float52, MSG_PID_P ELABEL, &PID_PARAM(Kp, eindex), 1, 9990); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I ELABEL, &raw_Ki, 0.01, 9990, copy_and_scalePID_i_E ## eindex); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D ELABEL, &raw_Kd, 1, 9990, copy_and_scalePID_d_E ## eindex)

      #if ENABLED(PID_EXTRUSION_SCALING)
        #define _PID_MENU_ITEMS(ELABEL, eindex) \
          _PID_BASE_MENU_ITEMS(ELABEL, eindex); \
          MENU_ITEM_EDIT(float3, MSG_PID_C ELABEL, &PID_PARAM(Kc, eindex), 1, 9990)
      #else
        #define _PID_MENU_ITEMS(ELABEL, eindex) _PID_BASE_MENU_ITEMS(ELABEL, eindex)
      #endif

      #if ENABLED(PID_AUTOTUNE_MENU)
        #define PID_MENU_ITEMS(ELABEL, eindex) \
          _PID_MENU_ITEMS(ELABEL, eindex); \
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE ELABEL, &autotune_temp[eindex], 150, heater_maxtemp[eindex] - 15, lcd_autotune_callback_E ## eindex)
      #else
        #define PID_MENU_ITEMS(ELABEL, eindex) _PID_MENU_ITEMS(ELABEL, eindex)
      #endif

      #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
        PID_MENU_ITEMS(" " MSG_E1, 0);
        PID_MENU_ITEMS(" " MSG_E2, 1);
        #if HOTENDS > 2
          PID_MENU_ITEMS(" " MSG_E3, 2);
          #if HOTENDS > 3
            PID_MENU_ITEMS(" " MSG_E4, 3);
            #if HOTENDS > 4
              PID_MENU_ITEMS(" " MSG_E5, 4);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #else // !PID_PARAMS_PER_HOTEND || HOTENDS == 1
        PID_MENU_ITEMS("", 0);
      #endif // !PID_PARAMS_PER_HOTEND || HOTENDS == 1

    #endif //PIDTEMP

    //
    // Preheat Material 1 conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_1_SETTINGS, lcd_control_temperature_preheat_material1_settings_menu);

    //
    // Preheat Material 2 conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_2_SETTINGS, lcd_control_temperature_preheat_material2_settings_menu);
    END_MENU();
  }

  void _lcd_control_temperature_preheat_settings_menu(uint8_t material) {
    #if HOTENDS > 4
      #define MINTEMP_ALL MIN5(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP, HEATER_4_MINTEMP)
      #define MAXTEMP_ALL MAX5(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP, HEATER_4_MAXTEMP)
    #elif HOTENDS > 3
      #define MINTEMP_ALL MIN4(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP)
      #define MAXTEMP_ALL MAX4(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP)
    #elif HOTENDS > 2
      #define MINTEMP_ALL MIN3(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP)
      #define MAXTEMP_ALL MAX3(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP)
    #elif HOTENDS > 1
      #define MINTEMP_ALL min(HEATER_0_MINTEMP, HEATER_1_MINTEMP)
      #define MAXTEMP_ALL max(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP)
    #else
      #define MINTEMP_ALL HEATER_0_MINTEMP
      #define MAXTEMP_ALL HEATER_0_MAXTEMP
    #endif
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &lcd_preheat_fan_speed[material], 0, 255);
    #if TEMP_SENSOR_0 != 0
      MENU_ITEM_EDIT(int3, MSG_NOZZLE, &lcd_preheat_hotend_temp[material], MINTEMP_ALL, MAXTEMP_ALL - 15);
    #endif
    #if TEMP_SENSOR_BED != 0
      MENU_ITEM_EDIT(int3, MSG_BED, &lcd_preheat_bed_temp[material], BED_MINTEMP, BED_MAXTEMP - 15);
    #endif
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
    #endif
    END_MENU();
  }

  /**
   *
   * "Temperature" > "Preheat Material 1 conf" submenu
   *
   */
  void lcd_control_temperature_preheat_material1_settings_menu() { _lcd_control_temperature_preheat_settings_menu(0); }

  /**
   *
   * "Temperature" > "Preheat Material 2 conf" submenu
   *
   */
  void lcd_control_temperature_preheat_material2_settings_menu() { _lcd_control_temperature_preheat_settings_menu(1); }


  /**
   *
   * "Control" > "Motion" submenu
   *
   */

  void _reset_acceleration_rates() { planner.reset_acceleration_rates(); }
  #if ENABLED(DISTINCT_E_FACTORS)
    void _reset_e_acceleration_rate(const uint8_t e) { if (e == active_extruder) _reset_acceleration_rates(); }
    void _reset_e0_acceleration_rate() { _reset_e_acceleration_rate(0); }
    void _reset_e1_acceleration_rate() { _reset_e_acceleration_rate(1); }
    #if E_STEPPERS > 2
      void _reset_e2_acceleration_rate() { _reset_e_acceleration_rate(2); }
      #if E_STEPPERS > 3
        void _reset_e3_acceleration_rate() { _reset_e_acceleration_rate(3); }
        #if E_STEPPERS > 4
          void _reset_e4_acceleration_rate() { _reset_e_acceleration_rate(4); }
        #endif // E_STEPPERS > 4
      #endif // E_STEPPERS > 3
    #endif // E_STEPPERS > 2
  #endif

  void _planner_refresh_positioning() { planner.refresh_positioning(); }
  #if ENABLED(DISTINCT_E_FACTORS)
    void _planner_refresh_e_positioning(const uint8_t e) {
      if (e == active_extruder)
        _planner_refresh_positioning();
      else
        planner.steps_to_mm[E_AXIS + e] = 1.0 / planner.axis_steps_per_mm[E_AXIS + e];
    }
    void _planner_refresh_e0_positioning() { _planner_refresh_e_positioning(0); }
    void _planner_refresh_e1_positioning() { _planner_refresh_e_positioning(1); }
    #if E_STEPPERS > 2
      void _planner_refresh_e2_positioning() { _planner_refresh_e_positioning(2); }
      #if E_STEPPERS > 3
        void _planner_refresh_e3_positioning() { _planner_refresh_e_positioning(3); }
        #if E_STEPPERS > 4
          void _planner_refresh_e4_positioning() { _planner_refresh_e_positioning(4); }
        #endif // E_STEPPERS > 4
      #endif // E_STEPPERS > 3
    #endif // E_STEPPERS > 2
  #endif

  // M203 / M205 Velocity options
  void lcd_control_motion_velocity_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    // M203 Max Feedrate
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &planner.max_feedrate_mm_s[X_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &planner.max_feedrate_mm_s[Y_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &planner.max_feedrate_mm_s[Z_AXIS], 1, 999);

    #if ENABLED(DISTINCT_E_FACTORS)
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &planner.max_feedrate_mm_s[E_AXIS + active_extruder], 1, 999);
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E1, &planner.max_feedrate_mm_s[E_AXIS], 1, 999);
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E2, &planner.max_feedrate_mm_s[E_AXIS + 1], 1, 999);
      #if E_STEPPERS > 2
        MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E3, &planner.max_feedrate_mm_s[E_AXIS + 2], 1, 999);
        #if E_STEPPERS > 3
          MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E4, &planner.max_feedrate_mm_s[E_AXIS + 3], 1, 999);
          #if E_STEPPERS > 4
            MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E5, &planner.max_feedrate_mm_s[E_AXIS + 4], 1, 999);
          #endif // E_STEPPERS > 4
        #endif // E_STEPPERS > 3
      #endif // E_STEPPERS > 2
    #else
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &planner.max_feedrate_mm_s[E_AXIS], 1, 999);
    #endif

    // M205 S Min Feedrate
    MENU_ITEM_EDIT(float3, MSG_VMIN, &planner.min_feedrate_mm_s, 0, 999);

    // M205 T Min Travel Feedrate
    MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &planner.min_travel_feedrate_mm_s, 0, 999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void lcd_control_motion_acceleration_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    // M204 P Acceleration
    MENU_ITEM_EDIT(float5, MSG_ACC, &planner.acceleration, 10, 99000);

    // M204 R Retract Acceleration
    MENU_ITEM_EDIT(float5, MSG_A_RETRACT, &planner.retract_acceleration, 100, 99000);

    // M204 T Travel Acceleration
    MENU_ITEM_EDIT(float5, MSG_A_TRAVEL, &planner.travel_acceleration, 100, 99000);

    // M201 settings
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &planner.max_acceleration_mm_per_s2[X_AXIS], 100, 99000, _reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &planner.max_acceleration_mm_per_s2[Y_AXIS], 100, 99000, _reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &planner.max_acceleration_mm_per_s2[Z_AXIS], 10, 500, _reset_acceleration_rates);

    #if ENABLED(DISTINCT_E_FACTORS)
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &planner.max_acceleration_mm_per_s2[E_AXIS + active_extruder], 100, 99000, _reset_acceleration_rates);
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E1, &planner.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_e0_acceleration_rate);
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E2, &planner.max_acceleration_mm_per_s2[E_AXIS + 1], 100, 99000, _reset_e1_acceleration_rate);
      #if E_STEPPERS > 2
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E3, &planner.max_acceleration_mm_per_s2[E_AXIS + 2], 100, 99000, _reset_e2_acceleration_rate);
        #if E_STEPPERS > 3
          MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E4, &planner.max_acceleration_mm_per_s2[E_AXIS + 3], 100, 99000, _reset_e3_acceleration_rate);
          #if E_STEPPERS > 4
            MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E5, &planner.max_acceleration_mm_per_s2[E_AXIS + 4], 100, 99000, _reset_e4_acceleration_rate);
          #endif // E_STEPPERS > 4
        #endif // E_STEPPERS > 3
      #endif // E_STEPPERS > 2
    #else
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &planner.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_acceleration_rates);
    #endif

    END_MENU();
  }

  // M205 Jerk
  void lcd_control_motion_jerk_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    MENU_ITEM_EDIT(float3, MSG_VX_JERK, &planner.max_jerk[X_AXIS], 1, 990);
    MENU_ITEM_EDIT(float3, MSG_VY_JERK, &planner.max_jerk[Y_AXIS], 1, 990);
    #if ENABLED(DELTA)
      MENU_ITEM_EDIT(float3, MSG_VZ_JERK, &planner.max_jerk[Z_AXIS], 1, 990);
    #else
      MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &planner.max_jerk[Z_AXIS], 0.1, 990);
    #endif
    MENU_ITEM_EDIT(float3, MSG_VE_JERK, &planner.max_jerk[E_AXIS], 1, 990);

    END_MENU();
  }

  // M92 Steps-per-mm
  void lcd_control_motion_steps_per_mm_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_XSTEPS, &planner.axis_steps_per_mm[X_AXIS], 5, 9999, _planner_refresh_positioning);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_YSTEPS, &planner.axis_steps_per_mm[Y_AXIS], 5, 9999, _planner_refresh_positioning);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ZSTEPS, &planner.axis_steps_per_mm[Z_AXIS], 5, 9999, _planner_refresh_positioning);

    #if ENABLED(DISTINCT_E_FACTORS)
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &planner.axis_steps_per_mm[E_AXIS + active_extruder], 5, 9999, _planner_refresh_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E1STEPS, &planner.axis_steps_per_mm[E_AXIS], 5, 9999, _planner_refresh_e0_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E2STEPS, &planner.axis_steps_per_mm[E_AXIS + 1], 5, 9999, _planner_refresh_e1_positioning);
      #if E_STEPPERS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E3STEPS, &planner.axis_steps_per_mm[E_AXIS + 2], 5, 9999, _planner_refresh_e2_positioning);
        #if E_STEPPERS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E4STEPS, &planner.axis_steps_per_mm[E_AXIS + 3], 5, 9999, _planner_refresh_e3_positioning);
          #if E_STEPPERS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E5STEPS, &planner.axis_steps_per_mm[E_AXIS + 4], 5, 9999, _planner_refresh_e4_positioning);
          #endif // E_STEPPERS > 4
        #endif // E_STEPPERS > 3
      #endif // E_STEPPERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &planner.axis_steps_per_mm[E_AXIS], 5, 9999, _planner_refresh_positioning);
    #endif

    END_MENU();
  }

  void lcd_control_motion_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
    #elif HAS_BED_PROBE
      MENU_ITEM_EDIT(float32, MSG_ZPROBE_ZOFFSET, &zprobe_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
    #endif

    // M203 / M205 - Feedrate items
    MENU_ITEM(submenu, MSG_VELOCITY, lcd_control_motion_velocity_menu);

    // M201 - Acceleration items
    MENU_ITEM(submenu, MSG_ACCELERATION, lcd_control_motion_acceleration_menu);

    // M205 - Max Jerk
    MENU_ITEM(submenu, MSG_JERK, lcd_control_motion_jerk_menu);

    // M92 - Steps Per mm
    MENU_ITEM(submenu, MSG_STEPS_PER_MM, lcd_control_motion_steps_per_mm_menu);

    // M540 S - Abort on endstop hit when SD printing
    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
      MENU_ITEM_EDIT(bool, MSG_ENDSTOP_ABORT, &stepper.abort_on_endstop_hit);
    #endif

    END_MENU();
  }

  /**
   *  BEEVC
   *
   * "Machine settings > Motion > Offset" options
   *
   */

  void beevc_machine_motion_offset_adjust() {
    if (lcd_clicked)
      {
        // Save to EEPRom
        lcd_completion_feedback(settings.save());

        // Changes the displayed screen to the previous menu
        lcd_goto_screen(beevc_machine_motion_offset_menu);

        // Necessary as this function is called as a submenu
        lcd_goto_previous_menu();
      }

    ENCODER_DIRECTION_NORMAL();

    if (encoderPosition) {
      refresh_cmd_timeout();

      // Get the new position
      hotend_offset[isX?X_AXIS:Y_AXIS][1] += float((int32_t)encoderPosition) * 0.05;

      // Limit only when trying to move towards the limit
      if ((int32_t)encoderPosition < 0) NOLESS(hotend_offset[isX?X_AXIS:Y_AXIS][1], (isX?12:-1));
      if ((int32_t)encoderPosition > 0) NOMORE(hotend_offset[isX?X_AXIS:Y_AXIS][1], (isX?14:1));
      
      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    if (lcdDrawUpdate) {
      START_SCREEN();
      STATIC_ITEM(_UxGT("Offset XY"), true, true);

      lcd_implementation_drawmenu_setting_edit_generic(false, 1,(isX?PSTR("Offset X"):PSTR("Offset Y")),itostr3((int)((hotend_offset[ isX?X_AXIS:Y_AXIS ][1]-old_hotend_offset) /0.05) ));
      //lcd_implementation_drawmenu_setting_edit_generic(false, 2,PSTR("Absolute value"),ftostr42sign(isX?(hotend_offset[X_AXIS][1] - 13):hotend_offset[Y_AXIS][1]));
      lcd_implementation_drawmenu_static(2,PSTR("Status: please adjust"));
      lcd_implementation_drawmenu_static(4,PSTR("Click to save.       "));

      END_SCREEN();
    }
  }

  void beevc_machine_motion_offset_x() {
    isX = true;
    old_hotend_offset = hotend_offset[X_AXIS][1];

    lcd_goto_screen (beevc_machine_motion_offset_adjust);
  }

  void beevc_machine_motion_offset_y() {
    isX = false;
    old_hotend_offset = hotend_offset[Y_AXIS][1];

    lcd_goto_screen (beevc_machine_motion_offset_adjust);
  }

  /**
   *  BEEVC
   *
   * "Machine settings > Motion > Offset" submenu
   *
   */
  void beevc_machine_motion_offset_menu() {
    START_MENU();
    MENU_BACK();

    // Offset X
    MENU_ITEM(submenu, _UxGT("Offset X"), beevc_machine_motion_offset_x);

    // Offset Y
    MENU_ITEM(submenu, _UxGT("Offset Y"), beevc_machine_motion_offset_y);

    END_MENU();
  }

  /**
   *  BEEVC
   *
   * "Machine settings > Motion" submenu
   *
   */
  void beevc_machine_motion_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    #ifdef HAVE_TMC2130
        MENU_ITEM(submenu,  _UxGT("Trinamic settings"), beevc_trinamic_settings_menu);
    #endif

    // M203 / M205 - Feedrate items
    MENU_ITEM(submenu, MSG_SPEED, lcd_control_motion_velocity_menu);

    // M201 - Acceleration items
    MENU_ITEM(submenu, MSG_ACCELERATION, lcd_control_motion_acceleration_menu);

    // M205 - Max Jerk
    MENU_ITEM(submenu, MSG_JERK, lcd_control_motion_jerk_menu);

    // M92 - Steps Per mm
    MENU_ITEM(submenu, MSG_STEPS_PER_MM, lcd_control_motion_steps_per_mm_menu);

    // M218 T1 - Second extruder offset
    MENU_ITEM(submenu, _UxGT("Offset"), beevc_machine_motion_offset_menu);

    END_MENU();
  }

  #if HAS_TRINAMIC
  // Auxiliary funcitons
  #if ENABLED(X_IS_TMC2130)
    inline void _trinamic_setX_current() {stepperX.setCurrent((uint16_t)trinamic_tempXcurrent, R_SENSE, HOLD_MULTIPLIER);}
    inline void _trinamic_setX_sgt() {stepperX.sgt(trinamic_tempXsgt);}
  #endif
  #if ENABLED(Y_IS_TMC2130)
    inline void _trinamic_setY_current() {stepperY.setCurrent((uint16_t)trinamic_tempYcurrent, R_SENSE, HOLD_MULTIPLIER);}
    inline void _trinamic_setY_sgt() {stepperY.sgt(trinamic_tempYsgt);}
  #endif
  #if ENABLED(Z_IS_TMC2130)
    inline void _trinamic_setZ_current() {stepperZ.setCurrent((uint16_t)trinamic_tempZcurrent, R_SENSE, HOLD_MULTIPLIER);}
  #endif
  #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
    inline void _trinamic_setE_current() {stepperE0.setCurrent((uint16_t)trinamic_tempEcurrent, R_SENSE, HOLD_MULTIPLIER);stepperE1.setCurrent((uint16_t)trinamic_tempEcurrent, R_SENSE, HOLD_MULTIPLIER);}
    inline void _trinamic_setE_sgt() {stepperE0.sgt((int8_t)trinamic_tempEsgt);stepperE1.sgt((int8_t)trinamic_tempEsgt);}
  #endif

  inline void _trinamic_update_current_values() {
    #if ENABLED(X_IS_TMC2130)
      trinamic_tempXcurrent = stepperX.getCurrent();
    #endif
    #if ENABLED(Y_IS_TMC2130)
      trinamic_tempYcurrent = stepperY.getCurrent();
    #endif
    #if ENABLED(Z_IS_TMC2130)
      trinamic_tempZcurrent = stepperZ.getCurrent();
    #endif
    #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
      trinamic_tempEcurrent = stepperE0.getCurrent();
    #endif
  }

  inline void _trinamic_update_sgt_values() {
    #if ENABLED(X_IS_TMC2130)
      trinamic_tempXsgt = (uint8_t)stepperX.sgt();
    #endif
    #if ENABLED(Y_IS_TMC2130)
      trinamic_tempYsgt = (uint8_t)stepperY.sgt();
    #endif
    #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
      trinamic_tempEsgt = (uint8_t)stepperE0.sgt();
    #endif
  }

  inline void _void_() {}

  inline void lcd_trinamic_reset() {
    #if ENABLED(X_IS_TMC2130)
      stepperX.setCurrent(X_CURRENT, R_SENSE, HOLD_MULTIPLIER);
      stepperX.sgt(X_HOMING_SENSITIVITY);
      thermalManager.sg2_homing_x_calibration = 0;
    #endif
    #if ENABLED(Y_IS_TMC2130)
      stepperY.setCurrent(Y_CURRENT, R_SENSE, HOLD_MULTIPLIER);
      stepperY.sgt(Y_HOMING_SENSITIVITY);
      thermalManager.sg2_homing_y_calibration = 40;
    #endif
    #if ENABLED(Z_IS_TMC2130)
      stepperZ.setCurrent(Z_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
    stepperE0.setCurrent(E0_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    stepperE1.setCurrent(E1_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    stepperE0.sgt(5);
    stepperE1.sgt(5);
    #endif

    (void)settings.save();
  }

  /**
   *
   * "Control" > "Trinamic Settings" > "Sensorless homing" submenu
   *
   */
    void lcd_trinamic_current() {
      START_MENU();
      MENU_BACK(_UxGT("Trinamic Settings"));
      _trinamic_update_current_values();
      #if ENABLED(X_IS_TMC2130)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_X, &trinamic_tempXcurrent, 500, 1200, _trinamic_setX_current);
      #endif
      #if ENABLED(Y_IS_TMC2130)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_Y, &trinamic_tempYcurrent, 500, 1200, _trinamic_setY_current);
      #endif
      #if ENABLED(Z_IS_TMC2130)
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_Z, &trinamic_tempZcurrent, 500, 1200, _trinamic_setZ_current);
      #endif
      #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_E, &trinamic_tempEcurrent, 500, 1200, _trinamic_setE_current);
      #endif
      END_MENU();
    }

  /**
   *
   * "Control" > "Trinamic Settings" > "Sensorless homing" submenu
   *
   */
   void lcd_trinamic_sensorless_auto_adjust(){
     // Starts variables to force calibration
     calibrating_sensorless_homing_x = true;
     calibrating_sensorless_homing_y = true;

     // Sensorless homing auto calibration
     gcode_M918();

     // Return to status screen
     lcd_return_to_status();
   }

   void lcd_trinamic_sensorless() {
     START_MENU();
     MENU_BACK(_UxGT("Trinamic Settings"));
     #if ENABLED(X_IS_TMC2130)
       MENU_ITEM_EDIT_CALLBACK(int8, _UxGT("X calibration"), &thermalManager.sg2_homing_x_calibration, 0, 100,_void_);
       MENU_ITEM(gcode, _UxGT("Test X homing"), PSTR("G28 X\nG28 X\nG28 X\nG28 X\nG28 X"));
     #endif
     #if ENABLED(Y_IS_TMC2130)
       MENU_ITEM_EDIT_CALLBACK(int8, _UxGT("Y calibration"), &thermalManager.sg2_homing_y_calibration, 40, 200,_void_);
       MENU_ITEM(gcode, _UxGT("Test Y homing"), PSTR("G28 Y\nG28 Y\nG28 Y\nG28 Y\nG28 Y"));
     #endif
     #if (ENABLED(Y_IS_TMC2130) && ENABLED(X_IS_TMC2130))
       MENU_ITEM(function, _UxGT("Auto adjust"), lcd_trinamic_sensorless_auto_adjust);
     #endif

   END_MENU();
   }

   /**
    *
    * "Control" > "Trinamic Settings" > "Stallguard2 settings" submenu
    *
    */
    void lcd_trinamic_stallguard2() {
      START_MENU();
      MENU_BACK(_UxGT("Trinamic Settings"));
      _trinamic_update_sgt_values();
      #if ENABLED(X_IS_TMC2130)
        MENU_ITEM_EDIT_CALLBACK(int8, _UxGT("X threshold"), &trinamic_tempXsgt, 0, 15, _trinamic_setX_sgt);
      #endif
      #if ENABLED(Y_IS_TMC2130)
        MENU_ITEM_EDIT_CALLBACK(int8, _UxGT("Y threshold"), &trinamic_tempYsgt, 0, 15, _trinamic_setY_sgt);
      #endif
      #if (ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
        MENU_ITEM_EDIT_CALLBACK(int8, _UxGT("E threshold"), &trinamic_tempEsgt, 0, 15, _trinamic_setE_sgt);
      #endif
   END_MENU();
    }



  /**
   *
   * "Control" > "Trinamic Settings" submenu
   *
   */
    void beevc_trinamic_settings_menu() {
      START_MENU();
      MENU_BACK(MSG_CONTROL);

      MENU_ITEM(submenu, _UxGT("Stepper current"), lcd_trinamic_current);

      // Only shown if X or Y are TMC2130 otherwise sensorless homing wont be used
      #if (ENABLED(X_IS_TMC2130) || ENABLED(Y_IS_TMC2130))
        MENU_ITEM(submenu, _UxGT("Sensorless homing"), lcd_trinamic_sensorless);
      #endif

      // Only shown if X, Y or E are TMC2130 otherwise stallguard measure aren't useful
      #if (ENABLED(X_IS_TMC2130) || ENABLED(Y_IS_TMC2130) || ENABLED(E0_IS_TMC2130) || ENABLED(E1_IS_TMC2130))
        MENU_ITEM(submenu, _UxGT("Stallguard2 settings"), lcd_trinamic_stallguard2);
      #endif

      MENU_ITEM(function, _UxGT("Reset default"), lcd_trinamic_reset);

      END_MENU();
    }
  #endif // HAS_TRINAMIC

  /**
   *
   * "Control" > "Filament" submenu
   *
   */
  void lcd_control_filament_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    #if ENABLED(LIN_ADVANCE)
      MENU_ITEM_EDIT(float3, MSG_ADVANCE_K, &planner.extruder_advance_k, 0, 999);
    #endif

    MENU_ITEM_EDIT_CALLBACK(bool, MSG_VOLUMETRIC_ENABLED, &parser.volumetric_enabled, planner.calculate_volumetric_multipliers);

    if (parser.volumetric_enabled) {
      #if EXTRUDERS == 1
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &planner.filament_size[0], 1.5, 3.25, planner.calculate_volumetric_multipliers);
      #else // EXTRUDERS > 1
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &planner.filament_size[active_extruder], 1.5, 3.25, planner.calculate_volumetric_multipliers);
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E1, &planner.filament_size[0], 1.5, 3.25, planner.calculate_volumetric_multipliers);
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E2, &planner.filament_size[1], 1.5, 3.25, planner.calculate_volumetric_multipliers);
        #if EXTRUDERS > 2
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E3, &planner.filament_size[2], 1.5, 3.25, planner.calculate_volumetric_multipliers);
          #if EXTRUDERS > 3
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E4, &planner.filament_size[3], 1.5, 3.25, planner.calculate_volumetric_multipliers);
            #if EXTRUDERS > 4
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E5, &planner.filament_size[4], 1.5, 3.25, planner.calculate_volumetric_multipliers);
            #endif // EXTRUDERS > 4
          #endif // EXTRUDERS > 3
        #endif // EXTRUDERS > 2
      #endif // EXTRUDERS > 1
    }

    END_MENU();
  }

  /**
   *
   * "Control" > "Retract" submenu
   *
   */
  #if ENABLED(FWRETRACT)

    void lcd_control_retract_menu() {
      START_MENU();
      MENU_BACK(MSG_CONTROL);
      MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &autoretract_enabled);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
      #if EXTRUDERS > 1
        MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_SWAP, &swap_retract_length, 0, 100);
      #endif
      MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate_mm_s, 1, 999);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, -100, 100);
      #if EXTRUDERS > 1
        MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER_SWAP, &swap_retract_recover_length, -100, 100);
      #endif
      MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate_mm_s, 1, 999);
      #if EXTRUDERS > 1
        MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVER_SWAPF, &swap_retract_recover_feedrate_mm_s, 1, 999);
      #endif
      END_MENU();
    }

  #endif // FWRETRACT

  #if ENABLED(SDSUPPORT)

    #if !PIN_EXISTS(SD_DETECT)
      void lcd_sd_refresh() {
        card.initsd();
        encoderTopLine = 0;
      }
    #endif

    void lcd_sd_updir() {
      encoderPosition = card.updir() ? ENCODER_STEPS_PER_MENU_ITEM : 0;
      encoderTopLine = 0;
      screen_changed = true;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    }

    /**
     *
     * "Print from SD" submenu
     *
     */

    #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
      uint32_t last_sdfile_encoderPosition = 0xFFFF;

      void lcd_reselect_last_file() {
        if (last_sdfile_encoderPosition == 0xFFFF) return;
        #if ENABLED(DOGLCD)
          // Some of this is a hack to force the screen update to work.
          // TODO: Fix the real issue that causes this!
          lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
          _lcd_synchronize();
          safe_delay(50);
          _lcd_synchronize();
          lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
          drawing_screen = screen_changed = true;
        #endif

        lcd_goto_screen(lcd_sdcard_menu, last_sdfile_encoderPosition);
        defer_return_to_status = true;
        last_sdfile_encoderPosition = 0xFFFF;

        #if ENABLED(DOGLCD)
          lcd_update();
        #endif
      }
    #endif

    void lcd_sdcard_menu() {
      ENCODER_DIRECTION_MENUS();

      const uint16_t fileCnt = card.get_num_Files();

      START_MENU();
      MENU_BACK(MSG_MAIN);
      card.getWorkDirName();
      if (card.filename[0] == '/') {
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
        #endif
      }
      else {
        MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
      }

      for (uint16_t i = 0; i < fileCnt; i++) {
        if (_menuLineNr == _thisItemNr) {
          const uint16_t nr =
            #if ENABLED(SDCARD_RATHERRECENTFIRST) && DISABLED(SDCARD_SORT_ALPHA)
              fileCnt - 1 -
            #endif
          i;

          #if ENABLED(SDCARD_SORT_ALPHA)
            card.getfilename_sorted(nr);
          #else
            card.getfilename(nr);
          #endif

          if (card.filenameIsDir)
            MENU_ITEM(sddirectory, MSG_CARD_MENU, card.filename, card.longFilename);
          else
            MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename);
        }
        else {
          MENU_ITEM_DUMMY();
        }
      }
      END_MENU();
    }

  #endif //SDSUPPORT

  #if ENABLED(LCD_INFO_MENU)

    #if ENABLED(PRINTCOUNTER)
      /**
       *
       * About Printer > Statistics submenu
       *
       */
      void lcd_info_stats_menu() {
        if (lcd_clicked) { return lcd_goto_previous_menu(); }

        char buffer[21];
        printStatistics stats = print_job_timer.getStats();

        START_SCREEN();                                                                                // 12345678901234567890
        STATIC_ITEM(MSG_INFO_PRINT_COUNT ": ", false, false, itostr3left(stats.totalPrints));          // Print Count: 999
        STATIC_ITEM(MSG_INFO_COMPLETED_PRINTS": ", false, false, itostr3left(stats.finishedPrints));   // Completed  : 666

        duration_t elapsed = stats.printTime;
        elapsed.toString(buffer);

        STATIC_ITEM(MSG_INFO_PRINT_TIME ": ");                                           // Total print Time:
        STATIC_ITEM("", false, false, buffer);                                                         // 99y 364d 23h 59m 59s

        elapsed = stats.longestPrint;
        elapsed.toString(buffer);

        STATIC_ITEM(MSG_INFO_PRINT_LONGEST ": ");                                        // Longest job time:
        STATIC_ITEM("", false, false, buffer);                                                         // 99y 364d 23h 59m 59s

        sprintf_P(buffer, PSTR("%ld.%im"), long(stats.filamentUsed / 1000), int16_t(stats.filamentUsed / 100) % 10);
        STATIC_ITEM(MSG_INFO_PRINT_FILAMENT ": ");                                       // Extruded total:
        STATIC_ITEM("", false, false, buffer);                                                         // 125m
        END_SCREEN();
      }
    #endif // PRINTCOUNTER

    /**
     *
     * About Printer > Thermistors
     *
     */
    void lcd_info_thermistors_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();
      #define THERMISTOR_ID TEMP_SENSOR_0
      #include "thermistornames.h"
      STATIC_ITEM("T0: " THERMISTOR_NAME, false, true);
      STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_0_MINTEMP), false);
      STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_0_MAXTEMP), false);

      #if TEMP_SENSOR_1 != 0
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_1
        #include "thermistornames.h"
        STATIC_ITEM("T1: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_1_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_1_MAXTEMP), false);
      #endif

      #if TEMP_SENSOR_2 != 0
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_2
        #include "thermistornames.h"
        STATIC_ITEM("T2: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_2_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_2_MAXTEMP), false);
      #endif

      #if TEMP_SENSOR_3 != 0
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_3
        #include "thermistornames.h"
        STATIC_ITEM("T3: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_3_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_3_MAXTEMP), false);
      #endif

      #if TEMP_SENSOR_4 != 0
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_4
        #include "thermistornames.h"
        STATIC_ITEM("T4: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_4_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_4_MAXTEMP), false);
      #endif

      #if TEMP_SENSOR_BED != 0
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_BED
        #include "thermistornames.h"
        STATIC_ITEM("TBed:" THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(BED_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(BED_MAXTEMP), false);
      #endif
      END_SCREEN();
    }

    /**
     *
     * About Printer > Board Info
     *
     */
    void lcd_info_board_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();
      STATIC_ITEM(BOARD_NAME, true, true);                           // MyPrinterController
      STATIC_ITEM(MSG_INFO_BAUDRATE ": " STRINGIFY(BAUDRATE), true); // Baud: 250000
      STATIC_ITEM(MSG_INFO_PROTOCOL ": " PROTOCOL_VERSION, true);    // Protocol: 1.0
      #if POWER_SUPPLY == 0
        STATIC_ITEM(MSG_INFO_PSU ": Generic", true);
      #elif POWER_SUPPLY == 1
        STATIC_ITEM(MSG_INFO_PSU ": ATX", true);  // Power Supply: ATX
      #elif POWER_SUPPLY == 2
        STATIC_ITEM(MSG_INFO_PSU ": XBox", true); // Power Supply: XBox
      #endif
      END_SCREEN();
    }

    /**
     *
     * About Printer > Printer Info
     *
     */
    void lcd_info_printer_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();
      STATIC_ITEM(MSG_MARLIN, true, true);                             // Marlin
      STATIC_ITEM(SHORT_BUILD_VERSION, true);                          // x.x.x-Branch
      STATIC_ITEM(STRING_DISTRIBUTION_DATE, true);                     // YYYY-MM-DD HH:MM
      STATIC_ITEM(MACHINE_NAME, true);                                 // My3DPrinter
      STATIC_ITEM(WEBSITE_URL, true);                                  // www.my3dprinter.com
      STATIC_ITEM(MSG_INFO_EXTRUDERS ": " STRINGIFY(EXTRUDERS), true); // Extruders: 2
      #if ENABLED(AUTO_BED_LEVELING_3POINT)
        STATIC_ITEM(MSG_3POINT_LEVELING, true);                        // 3-Point Leveling
      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
        STATIC_ITEM(MSG_LINEAR_LEVELING, true);                        // Linear Leveling
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        STATIC_ITEM(MSG_BILINEAR_LEVELING, true);                      // Bi-linear Leveling
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        STATIC_ITEM(MSG_UBL_LEVELING, true);                           // Unified Bed Leveling
      #elif ENABLED(MESH_BED_LEVELING)
        STATIC_ITEM(MSG_MESH_LEVELING, true);                          // Mesh Leveling
      #endif
      END_SCREEN();
    }

    /**
     *
     * "About Printer" submenu
     *
     */
    void lcd_info_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      MENU_ITEM(submenu, MSG_INFO_PRINTER_MENU, lcd_info_printer_menu);        // Printer Info >
      MENU_ITEM(submenu, MSG_INFO_BOARD_MENU, lcd_info_board_menu);            // Board Info >
      MENU_ITEM(submenu, MSG_INFO_THERMISTOR_MENU, lcd_info_thermistors_menu); // Thermistors >
      #if ENABLED(PRINTCOUNTER)
        MENU_ITEM(submenu, MSG_INFO_STATS_MENU, lcd_info_stats_menu);          // Printer Statistics >
      #endif
      END_MENU();
    }
  #endif // LCD_INFO_MENU

  /**
   *
   * LED Menu
   *
   */

  #if ENABLED(LED_CONTROL_MENU)

    #if ENABLED(LED_COLOR_PRESETS)

      void lcd_led_presets_menu() {
        START_MENU();
        #if LCD_HEIGHT > 2
          STATIC_ITEM(MSG_LED_PRESETS, true, true);
        #endif
        MENU_BACK(MSG_LED_CONTROL);
        MENU_ITEM(function, MSG_SET_LEDS_WHITE, leds.set_white);
        MENU_ITEM(function, MSG_SET_LEDS_RED, leds.set_red);
        MENU_ITEM(function, MSG_SET_LEDS_ORANGE, leds.set_orange);
        MENU_ITEM(function, MSG_SET_LEDS_YELLOW,leds.set_yellow);
        MENU_ITEM(function, MSG_SET_LEDS_GREEN, leds.set_green);
        MENU_ITEM(function, MSG_SET_LEDS_BLUE, leds.set_blue);
        MENU_ITEM(function, MSG_SET_LEDS_INDIGO, leds.set_indigo);
        MENU_ITEM(function, MSG_SET_LEDS_VIOLET, leds.set_violet);
        END_MENU();
      }
    #endif // LED_COLOR_PRESETS

    void lcd_led_custom_menu() {
      START_MENU();
      MENU_BACK(MSG_LED_CONTROL);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_INTENSITY_R, &leds.color.r, 0, 255, leds.update, true);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_INTENSITY_G, &leds.color.g, 0, 255, leds.update, true);
      MENU_ITEM_EDIT_CALLBACK(int8, MSG_INTENSITY_B, &leds.color.b, 0, 255, leds.update, true);
      #if ENABLED(RGBW_LED) || ENABLED(NEOPIXEL_LED)
        MENU_ITEM_EDIT_CALLBACK(int8, MSG_INTENSITY_W, &leds.color.w, 0, 255, leds.update, true);
        #if ENABLED(NEOPIXEL_LED)
          MENU_ITEM_EDIT_CALLBACK(int8, MSG_LED_BRIGHTNESS, &leds.color.i, 0, 255, leds.update, true);
        #endif
      #endif
      END_MENU();
    }

    void lcd_led_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      if (leds.lights_on)
        MENU_ITEM(function, MSG_LEDS_OFF, leds.toggle);
      else
        MENU_ITEM(function, MSG_LEDS_ON, leds.toggle);
      MENU_ITEM(function, MSG_SET_LEDS_DEFAULT, leds.set_default);
      #if ENABLED(LED_COLOR_PRESETS)
        MENU_ITEM(submenu, MSG_LED_PRESETS, lcd_led_presets_menu);
      #endif
      MENU_ITEM(submenu, MSG_CUSTOM_LEDS, lcd_led_custom_menu);
      END_MENU();
    }

  #endif // LED_CONTROL_MENU

  /**
   *
   * Filament Change Feature Screens
   *
   */
  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    // Portions from STATIC_ITEM...
    #define HOTEND_STATUS_ITEM() do { \
      if (_menuLineNr == _thisItemNr) { \
        if (lcdDrawUpdate) { \
          lcd_implementation_drawmenu_static(_lcdLineNr, PSTR(MSG_FILAMENT_CHANGE_NOZZLE), false, true); \
          lcd_implementation_hotend_status(_lcdLineNr); \
        } \
        if (_skipStatic && encoderLine <= _thisItemNr) { \
          encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
          ++encoderLine; \
        } \
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT; \
      } \
      ++_thisItemNr; \
    }while(0)

    void lcd_advanced_pause_toocold_menu() {
      START_MENU();
      STATIC_ITEM(MSG_HEATING_FAILED_LCD, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_MINTEMP STRINGIFY(EXTRUDE_MINTEMP) ".");
      MENU_BACK(MSG_BACK);
      #if LCD_HEIGHT > 4
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_MENU();
    }

    void lcd_advanced_pause_resume_print() {advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;}
    void lcd_advanced_pause_extrude_more() {advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;}

    void lcd_advanced_pause_option_menu() {
      START_MENU();
      if(pause_filament_runout) {STATIC_ITEM("Filament runout", true, true);}
      else                      {STATIC_ITEM("Change filament", true, true);}
      MENU_ITEM(function, "Back                \x04", lcd_advanced_pause_resume_print);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_EXTRUDE, lcd_advanced_pause_extrude_more);
      END_MENU();
    }

    void lcd_advanced_pause_screens() {
      START_SCREEN();
      if(pause_filament_runout) {STATIC_ITEM("Filament runout", true, true);}
      else                      {STATIC_ITEM("Change filament", true, true);}
      LCD_PRINT_EXT_TEMP();

      switch (pause_status){
        case pause_init:
          STATIC_ITEM("Status: moving");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
        case pause_unload:
          STATIC_ITEM("Status: unloading");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
        case pause_wait:
          STATIC_ITEM("Status: heating");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
        case pause_heatup:
          STATIC_ITEM("Status: standby");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Click to reheat.");
          break;
        case pause_insert:
          STATIC_ITEM("Status: waiting");               
          STATIC_ITEM("Please, load filament");
          STATIC_ITEM("and click to continue");
          break;
        case pause_load:
          STATIC_ITEM("Status: loading");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
        case pause_extrude:
          STATIC_ITEM("Status: extruding");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
        case pause_resume:
          STATIC_ITEM("Status: resuming");               
          STATIC_ITEM(" ");
          STATIC_ITEM("Please wait.");
          break;
      }
      END_SCREEN();
    }

    #define ADVANCED_PAUSE_SETUP \
      defer_return_to_status = true; \
      beevc_screen_constant_update = true

    void lcd_advanced_pause_show_message(const AdvancedPauseMessage message) {
      switch (message) {
        case ADVANCED_PAUSE_MESSAGE_INIT:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_init;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_UNLOAD:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_unload;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_INSERT:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_insert;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_LOAD:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_load;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_EXTRUDE:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_extrude;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_heatup;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_wait;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_RESUME:
          ADVANCED_PAUSE_SETUP;
          pause_status = pause_resume;
          lcd_goto_screen(lcd_advanced_pause_screens);
          break;
        case ADVANCED_PAUSE_MESSAGE_OPTION:
          ADVANCED_PAUSE_SETUP;
          advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
          lcd_goto_screen(lcd_advanced_pause_option_menu);
          break;

        case ADVANCED_PAUSE_MESSAGE_STATUS:
          beevc_screen_constant_update = false;
          lcd_return_to_status();
          beevc_force_screen_update();
          break;

    // BEEVC - Sensorless homing auto calibration
    case SENSORLESS_HOMING_CALIBRATION_X:
     defer_return_to_status = true;
         lcd_goto_screen(lcd_sensorless_homing_calibration_x);
         break;
     case SENSORLESS_HOMING_CALIBRATION_Y:
      defer_return_to_status = true;
          lcd_goto_screen(lcd_sensorless_homing_calibration_y);
          break;
      case SENSORLESS_HOMING_CALIBRATION_X_DONE:
       defer_return_to_status = true;
           lcd_goto_screen(lcd_sensorless_homing_calibration_x_done);
           break;
       case SENSORLESS_HOMING_CALIBRATION_Y_DONE:
        defer_return_to_status = true;
            lcd_goto_screen(lcd_sensorless_homing_calibration_y_done);
            break;
      case SENSORLESS_HOMING_CALIBRATION_HOMING:
       defer_return_to_status = true;
           lcd_goto_screen(lcd_sensorless_homing_calibration_homing);
           break;
      // case SENSORLESS_HOMING_CALIBRATION_DONE:
      //  defer_return_to_status = true;
      //      lcd_goto_screen(lcd_sensorless_homing_calibration_done);
      //      break;
      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  /**
   *
   * Functions for editing single values
   *
   * The "DEFINE_MENU_EDIT_TYPE" macro generates the functions needed to edit a numerical value.
   *
   * For example, DEFINE_MENU_EDIT_TYPE(int16_t, int3, itostr3, 1) expands into these functions:
   *
   *   bool _menu_edit_int3();
   *   void menu_edit_int3(); // edit int16_t (interactively)
   *   void menu_edit_callback_int3(); // edit int16_t (interactively) with callback on completion
   *   void _menu_action_setting_edit_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue);
   *   void menu_action_setting_edit_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue);
   *   void menu_action_setting_edit_callback_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue, const screenFunc_t callback, const bool live); // edit int16_t with callback
   *
   * You can then use one of the menu macros to present the edit interface:
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *
   * This expands into a more primitive menu item:
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   * ...which calls:
   *       menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   */
  #define DEFINE_MENU_EDIT_TYPE(_type, _name, _strFunc, _scale) \
    bool _menu_edit_ ## _name() { \
      ENCODER_DIRECTION_NORMAL(); \
      if ((int32_t)encoderPosition < 0) encoderPosition = 0; \
      if ((int32_t)encoderPosition > maxEditValue) encoderPosition = maxEditValue; \
      if (lcdDrawUpdate) \
        lcd_implementation_drawedit(editLabel, _strFunc(((_type)((int32_t)encoderPosition + minEditValue)) * (1.0 / _scale))); \
      if (lcd_clicked || (liveEdit && lcdDrawUpdate)) { \
        _type value = ((_type)((int32_t)encoderPosition + minEditValue)) * (1.0 / _scale); \
        if (editValue != NULL) *((_type*)editValue) = value; \
        if (liveEdit) (*callbackFunc)(); \
        if (lcd_clicked) lcd_goto_previous_menu(); \
      } \
      return lcd_clicked; \
    } \
    void menu_edit_ ## _name() { _menu_edit_ ## _name(); } \
    void menu_edit_callback_ ## _name() { if (_menu_edit_ ## _name()) (*callbackFunc)(); } \
    void _menu_action_setting_edit_ ## _name(const char * const pstr, _type* const ptr, const _type minValue, const _type maxValue) { \
      lcd_save_previous_screen(); \
      \
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; \
      \
      editLabel = pstr; \
      editValue = ptr; \
      minEditValue = minValue * _scale; \
      maxEditValue = maxValue * _scale - minEditValue; \
      encoderPosition = (*ptr) * _scale - minEditValue; \
    } \
    void menu_action_setting_edit_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_ ## _name; \
    } \
    void menu_action_setting_edit_callback_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue, const screenFunc_t callback, const bool live) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_callback_ ## _name; \
      callbackFunc = callback; \
      liveEdit = live; \
    } \
    typedef void _name

  DEFINE_MENU_EDIT_TYPE(uint32_t, long5, ftostr5rj, 0.1);
  DEFINE_MENU_EDIT_TYPE(int16_t, int3, itostr3, 1);
  DEFINE_MENU_EDIT_TYPE(uint8_t, int8, i8tostr3, 1);
  DEFINE_MENU_EDIT_TYPE(float, float3, ftostr3, 1.0);
  DEFINE_MENU_EDIT_TYPE(float, float32, ftostr32, 100.0);
  DEFINE_MENU_EDIT_TYPE(float, float43, ftostr43sign, 1000.0);
  DEFINE_MENU_EDIT_TYPE(float, float5, ftostr5rj, 0.01);
  DEFINE_MENU_EDIT_TYPE(float, float51, ftostr51sign, 10.0);
  DEFINE_MENU_EDIT_TYPE(float, float52, ftostr52sign, 100.0);
  DEFINE_MENU_EDIT_TYPE(float, float62, ftostr62rj, 100.0);

  /**
   *
   * Handlers for Keypad input
   *
   */
  #if ENABLED(ADC_KEYPAD)

    inline bool handle_adc_keypad() {
      #define ADC_MIN_KEY_DELAY 100
      if (buttons_reprapworld_keypad) {
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        if (encoderDirection == -1) { // side effect which signals we are inside a menu
          if      (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)  encoderPosition -= ENCODER_STEPS_PER_MENU_ITEM;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)    encoderPosition += ENCODER_STEPS_PER_MENU_ITEM;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_LEFT)  { menu_action_back(); lcd_quick_feedback(); }
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT) { lcd_return_to_status(); lcd_quick_feedback(); }
        }
        else {
          if (buttons_reprapworld_keypad & (EN_REPRAPWORLD_KEYPAD_DOWN|EN_REPRAPWORLD_KEYPAD_UP|EN_REPRAPWORLD_KEYPAD_RIGHT)) {
                 if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)  encoderPosition += ENCODER_PULSES_PER_STEP;
            else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)    encoderPosition -= ENCODER_PULSES_PER_STEP;
            else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT) encoderPosition = 0;
          }
        }
        #if ENABLED(ADC_KEYPAD_DEBUG)
          SERIAL_PROTOCOLLNPAIR("buttons_reprapworld_keypad = ", (uint32_t)buttons_reprapworld_keypad);
          SERIAL_PROTOCOLLNPAIR("encoderPosition = ", (uint32_t)encoderPosition);
        #endif
        next_button_update_ms = millis() + ADC_MIN_KEY_DELAY;
        return true;
      }

      return false;
    }

  #elif ENABLED(REPRAPWORLD_KEYPAD)

    void _reprapworld_keypad_move(const AxisEnum axis, const int16_t dir) {
      move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
      encoderPosition = dir;
      switch (axis) {
        case X_AXIS: lcd_move_x(); break;
        case Y_AXIS: lcd_move_y(); break;
        case Z_AXIS: lcd_move_z();
        default: break;
      }
    }
    void reprapworld_keypad_move_z_up()    { _reprapworld_keypad_move(Z_AXIS,  1); }
    void reprapworld_keypad_move_z_down()  { _reprapworld_keypad_move(Z_AXIS, -1); }
    void reprapworld_keypad_move_x_left()  { _reprapworld_keypad_move(X_AXIS, -1); }
    void reprapworld_keypad_move_x_right() { _reprapworld_keypad_move(X_AXIS,  1); }
    void reprapworld_keypad_move_y_up()    { _reprapworld_keypad_move(Y_AXIS, -1); }
    void reprapworld_keypad_move_y_down()  { _reprapworld_keypad_move(Y_AXIS,  1); }
    void reprapworld_keypad_move_home()    { enqueue_and_echo_commands_P(PSTR("G28")); } // move all axes home and wait
    void reprapworld_keypad_move_menu()    { lcd_goto_screen(lcd_move_menu); }

    inline void handle_reprapworld_keypad() {

      static uint8_t keypad_debounce = 0;

      if (!REPRAPWORLD_KEYPAD_PRESSED) {
        if (keypad_debounce > 0) keypad_debounce--;
      }
      else if (!keypad_debounce) {
        keypad_debounce = 2;

        if (REPRAPWORLD_KEYPAD_MOVE_MENU)       reprapworld_keypad_move_menu();

        #if DISABLED(DELTA) && Z_HOME_DIR == -1
          if (REPRAPWORLD_KEYPAD_MOVE_Z_UP)     reprapworld_keypad_move_z_up();
        #endif

        if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
          #if ENABLED(DELTA) || Z_HOME_DIR != -1
            if (REPRAPWORLD_KEYPAD_MOVE_Z_UP)   reprapworld_keypad_move_z_up();
          #endif
          if (REPRAPWORLD_KEYPAD_MOVE_Z_DOWN)   reprapworld_keypad_move_z_down();
          if (REPRAPWORLD_KEYPAD_MOVE_X_LEFT)   reprapworld_keypad_move_x_left();
          if (REPRAPWORLD_KEYPAD_MOVE_X_RIGHT)  reprapworld_keypad_move_x_right();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_DOWN)   reprapworld_keypad_move_y_down();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_UP)     reprapworld_keypad_move_y_up();
        }
        else {
          if (REPRAPWORLD_KEYPAD_MOVE_HOME)     reprapworld_keypad_move_home();
        }
      }
    }

  #endif // REPRAPWORLD_KEYPAD

  /**
   *
   * Menu actions
   *
   */
  void _menu_action_back() { lcd_goto_previous_menu(); }
  void menu_action_submenu(screenFunc_t func) { lcd_save_previous_screen(); lcd_goto_screen(func); }
  void menu_action_gcode(const char* pgcode) { enqueue_and_echo_commands_P(pgcode); }
  void menu_action_function(screenFunc_t func) { (*func)(); }

  #if ENABLED(SDSUPPORT)

    void menu_action_sdfile(const char* filename, char* longFilename) {
      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        last_sdfile_encoderPosition = encoderPosition;  // Save which file was selected for later use
      #endif
      UNUSED(longFilename);
      card.openAndPrintFile(filename);

      #ifdef BEEVC_Restore
        //Starting a new print so recovered files can be deleted
        toRecover = false;
        enqueue_and_echo_commands_P(PSTR("M712"));
    	#endif

      lcd_return_to_status();
    }

    void menu_action_sddirectory(const char* filename, char* longFilename) {
      UNUSED(longFilename);
      card.chdir(filename);
      encoderTopLine = 0;
      encoderPosition = 2 * ENCODER_STEPS_PER_MENU_ITEM;
      screen_changed = true;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    }

  #endif //SDSUPPORT

  void menu_action_setting_edit_bool(const char* pstr, bool* ptr) { UNUSED(pstr); *ptr ^= true; lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; }
  void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callback) {
    menu_action_setting_edit_bool(pstr, ptr);
    (*callback)();
  }

#endif // ULTIPANEL

void lcd_init() {

  lcd_implementation_init();

  #if ENABLED(NEWPANEL)
    #if BUTTON_EXISTS(EN1)
      SET_INPUT_PULLUP(BTN_EN1);
    #endif

    #if BUTTON_EXISTS(EN2)
      SET_INPUT_PULLUP(BTN_EN2);
    #endif

    #if BUTTON_EXISTS(ENC)
      SET_INPUT_PULLUP(BTN_ENC);
    #endif

    #if ENABLED(REPRAPWORLD_KEYPAD) && DISABLED(ADC_KEYPAD)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif

    #if BUTTON_EXISTS(UP)
      SET_INPUT(BTN_UP);
    #endif
    #if BUTTON_EXISTS(DWN)
      SET_INPUT(BTN_DWN);
    #endif
    #if BUTTON_EXISTS(LFT)
      SET_INPUT(BTN_LFT);
    #endif
    #if BUTTON_EXISTS(RT)
      SET_INPUT(BTN_RT);
    #endif

  #else // !NEWPANEL

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register
      SET_OUTPUT(SR_DATA_PIN);
      SET_OUTPUT(SR_CLK_PIN);
    #elif defined(SHIFT_CLK)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      OUT_WRITE(SHIFT_EN, LOW);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif // SR_LCD_2W_NL

  #endif // !NEWPANEL

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // UNKNOWN
  #endif

  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  lcd_buttons_update();

  #if ENABLED(ULTIPANEL)
    encoderDiff = 0;
  #endif
}

int16_t lcd_strlen(const char* s) {
  int16_t i = 0, j = 0;
  while (s[i]) {
    if (PRINTABLE(s[i])) j++;
    i++;
  }
  return j;
}

int16_t lcd_strlen_P(const char* s) {
  int16_t j = 0;
  while (pgm_read_byte(s)) {
    if (PRINTABLE(pgm_read_byte(s))) j++;
    s++;
  }
  return j;
}

bool lcd_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - LCD_UPDATE_INTERVAL / 2;
  }
  return blink != 0;
}

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Set lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NOW on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (lcdDrawUpdate) { redraw }
 *     - Before exiting the handler set lcdDrawUpdate to:
 *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
 *       - LCDVIEW_REDRAW_NOW to draw now (including remaining stripes).
 *       - LCDVIEW_CALL_REDRAW_NEXT to draw now and get LCDVIEW_REDRAW_NOW on the next loop.
 *       - LCDVIEW_CALL_NO_REDRAW to draw now and get LCDVIEW_NONE on the next loop.
 *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
 *             so don't change lcdDrawUpdate without considering this.
 *
 *   After the menu handler callback runs (or not):
 *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
 *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
 *
 * No worries. This function is only called from the main thread.
 */
void lcd_update() {

  #if ENABLED(ULTIPANEL)
    static millis_t return_to_status_ms = 0;

    // Handle any queued Move Axis motion
    manage_manual_move();

    // Update button states for LCD_CLICKED, etc.
    // After state changes the next button update
    // may be delayed 300-500ms.
    lcd_buttons_update();

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      // Don't run the debouncer if UBL owns the display
      #define UBL_CONDITION !lcd_external_control
    #else
      #define UBL_CONDITION true
    #endif

    // If the action button is pressed...
    if (UBL_CONDITION && LCD_CLICKED) {
      if (!wait_for_unclick) {           // If not waiting for a debounce release:
        wait_for_unclick = true;         //  Set debounce flag to ignore continous clicks
        lcd_clicked = !wait_for_user;    //  Keep the click if not waiting for a user-click
        wait_for_user = false;           //  Any click clears wait for user
        lcd_quick_feedback();            //  Always make a click sound
      }
    }
    else wait_for_unclick = false;
  #endif

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)

    const bool sd_status = IS_SD_INSERTED;
    if (sd_status != lcd_sd_status && lcd_detected()) {

      if (sd_status) {
        unsigned long slow_card_timeout = millis() + 1000UL;
        do {
          card.initsd();
          thermalManager.manage_heater();
        } while (!card.cardOK && PENDING(millis(), slow_card_timeout));
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_INSERTED);
      }
      else {
        card.release();
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_REMOVED);
      }

      lcd_sd_status = sd_status;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
      lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
        #if ENABLED(LCD_PROGRESS_BAR)
          currentScreen == lcd_status_screen ? CHARSET_INFO : CHARSET_MENU
        #endif
      );
    }

  #endif //SDSUPPORT && SD_DETECT_PIN

  const millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)
    #if ENABLED(DOGLCD)
      || drawing_screen
    #endif
    ) {

    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      lcd_implementation_update_indicators();
    #endif

    #if ENABLED(ULTIPANEL)

      #if ENABLED(LCD_HAS_SLOW_BUTTONS)
        slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
      #endif

      #if ENABLED(ADC_KEYPAD)

        if (handle_adc_keypad())
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;

      #elif ENABLED(REPRAPWORLD_KEYPAD)

        handle_reprapworld_keypad();

      #endif

      const bool encoderPastThreshold = (abs(encoderDiff) >= ENCODER_PULSES_PER_STEP);
      if (encoderPastThreshold || lcd_clicked) {
        if (encoderPastThreshold) {
          int32_t encoderMultiplier = 1;

          #if ENABLED(ENCODER_RATE_MULTIPLIER)

            if (encoderRateMultiplierEnabled) {
              int32_t encoderMovementSteps = abs(encoderDiff) / ENCODER_PULSES_PER_STEP;

              if (lastEncoderMovementMillis) {
                // Note that the rate is always calculated between two passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                float encoderStepRate = float(encoderMovementSteps) / float(ms - lastEncoderMovementMillis) * 1000.0;

                if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)     encoderMultiplier = 100;
                else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC) encoderMultiplier = 10;

                #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                  SERIAL_ECHO_START();
                  SERIAL_ECHOPAIR("Enc Step Rate: ", encoderStepRate);
                  SERIAL_ECHOPAIR("  Multiplier: ", encoderMultiplier);
                  SERIAL_ECHOPAIR("  ENCODER_10X_STEPS_PER_SEC: ", ENCODER_10X_STEPS_PER_SEC);
                  SERIAL_ECHOPAIR("  ENCODER_100X_STEPS_PER_SEC: ", ENCODER_100X_STEPS_PER_SEC);
                  SERIAL_EOL();
                #endif // ENCODER_RATE_MULTIPLIER_DEBUG
              }

              lastEncoderMovementMillis = ms;
            } // encoderRateMultiplierEnabled
          #endif //ENCODER_RATE_MULTIPLIER

          encoderPosition += (encoderDiff * encoderMultiplier) / ENCODER_PULSES_PER_STEP;
          encoderDiff = 0;
        }
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
    #endif // ULTIPANEL


    // B2X300 - keep screen updating
    if(beevc_screen_constant_update){
      beevc_screen_constant_update_time = millis() + 200;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    // We arrive here every ~100ms when idling often enough.
    // Instead of tracking the changes simply redraw the Info Screen ~1 time a second.
    if (
      #if ENABLED(ULTIPANEL)
        currentScreen == lcd_status_screen &&
      #endif
      !lcd_status_update_delay--
    ) {
      lcd_status_update_delay = 9
        #if ENABLED(DOGLCD)
          + 3
        #endif
      ;
      max_display_update_time--;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    #if ENABLED(SCROLL_LONG_FILENAMES)
      // If scrolling of long file names is enabled and we are in the sd card menu,
      // cause a refresh to occur until all the text has scrolled into view.
      if (currentScreen == lcd_sdcard_menu && filename_scroll_pos < filename_scroll_max && !lcd_status_update_delay--) {
        lcd_status_update_delay = 6;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        filename_scroll_pos++;
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      }
    #endif

    // then we want to use 1/2 of the time only.
    uint16_t bbr2 = planner.block_buffer_runtime() >> 1;

    #if ENABLED(DOGLCD)
      #define IS_DRAWING drawing_screen
    #else
      #define IS_DRAWING false
    #endif

    if ((lcdDrawUpdate || IS_DRAWING) && (!bbr2 || bbr2 > max_display_update_time)) {

      if (!IS_DRAWING) switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          lcdDrawUpdate = LCDVIEW_NONE;
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW: // set by handlers, then altered after (rarely occurs here)
        case LCDVIEW_CALL_REDRAW_NEXT:  // set by handlers, then altered after (never occurs here?)
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
        case LCDVIEW_NONE:
          break;
      } // switch

      #if ENABLED(ADC_KEYPAD)
        buttons_reprapworld_keypad = 0;
      #endif

      #if ENABLED(ULTIPANEL)
        #define CURRENTSCREEN() (*currentScreen)(), lcd_clicked = false
      #else
        #define CURRENTSCREEN() lcd_status_screen()
      #endif

      #if ENABLED(DOGLCD)
        if (!drawing_screen) {                        // If not already drawing pages
          u8g.firstPage();                            // Start the first page
          drawing_screen = 1;                         // Flag as drawing pages
        }
        lcd_setFont(FONT_MENU);                       // Setup font for every page draw
        u8g.setColorIndex(1);                         // And reset the color
        CURRENTSCREEN();                              // Draw and process the current screen

        // The screen handler can clear drawing_screen for an action that changes the screen.
        // If still drawing and there's another page, update max-time and return now.
        // The nextPage will already be set up on the next call.
        if (drawing_screen && (drawing_screen = u8g.nextPage())) {
          NOLESS(max_display_update_time, millis() - ms);
          return;
        }
      #else
        CURRENTSCREEN();
      #endif

      // Keeping track of the longest time for an individual LCD update.
      // Used to do screen throttling when the planner starts to fill up.
      NOLESS(max_display_update_time, millis() - ms);
    }

    #if ENABLED(ULTIPANEL)

      // Return to Status Screen after a timeout
      if (currentScreen == lcd_status_screen || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        lcd_return_to_status();

    #endif // ULTIPANEL

    if (!IS_DRAWING) switch (lcdDrawUpdate) {
      case LCDVIEW_CLEAR_CALL_REDRAW:
        lcd_implementation_clear();
      case LCDVIEW_CALL_REDRAW_NEXT:
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        break;
      case LCDVIEW_REDRAW_NOW:
        lcdDrawUpdate = LCDVIEW_NONE;
        break;
      case LCDVIEW_NONE:
        break;
    } // switch

  } // ELAPSED(ms, next_lcd_update_ms)
}

inline void pad_message_string() {
  uint8_t i = 0, j = 0;
  char c;
  while ((c = lcd_status_message[i]) && j < LCD_WIDTH) {
    if (PRINTABLE(c)) j++;
    i++;
  }
  if (true
    #if ENABLED(STATUS_MESSAGE_SCROLLING)
      && j < LCD_WIDTH
    #endif
  ) {
    // pad with spaces to fill up the line
    while (j++ < LCD_WIDTH) lcd_status_message[i++] = ' ';
    // chop off at the edge
    lcd_status_message[i] = '\0';
  }
}

void lcd_finishstatus(const bool persist=false) {

  pad_message_string();

  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif
  lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

  #if ENABLED(FILAMENT_LCD_DISPLAY) && ENABLED(SDSUPPORT)
    previous_lcd_status_ms = millis();  //get status message to show up for a while
  #endif

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    status_scroll_pos = 0;
  #endif
}

#if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
  void dontExpireStatus() { expire_status_ms = 0; }
#endif

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(const char * const message, const bool persist) {
  if (lcd_status_message_level > 0) return;
  strncpy(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(persist);
}

void lcd_setstatusPGM(const char * const message, int8_t level) {
  if (level < 0) level = lcd_status_message_level = 0;
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  strncpy_P(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(level > 0);
}

void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) {
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(lcd_status_message, 3 * (LCD_WIDTH), fmt, args);
  va_end(args);
  lcd_finishstatus(level > 0);
}

void lcd_setalertstatusPGM(const char * const message) {
  lcd_setstatusPGM(message, 1);
  #if ENABLED(ULTIPANEL)
    lcd_return_to_status();
  #endif
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

#if HAS_LCD_CONTRAST

  void set_lcd_contrast(const uint16_t value) {
    lcd_contrast = constrain(value, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX);
    u8g.setContrast(lcd_contrast);
  }

#endif

#if ENABLED(ULTIPANEL)

  /**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if defined(EN_A) && defined(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_SHIFT_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0


  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void lcd_buttons_update() {
    static uint8_t lastEncoderBits;
    const millis_t now = millis();
    if (ELAPSED(now, next_button_update_ms)) {

      #if ENABLED(NEWPANEL)
        uint8_t newbutton = 0;

        #if BUTTON_EXISTS(EN1)
          if (BUTTON_PRESSED(EN1)) newbutton |= EN_A;
        #endif

        #if BUTTON_EXISTS(EN2)
          if (BUTTON_PRESSED(EN2)) newbutton |= EN_B;
        #endif

        #if BUTTON_EXISTS(ENC)
          if (BUTTON_PRESSED(ENC)) newbutton |= EN_C;
        #endif

        #if LCD_HAS_DIRECTIONAL_BUTTONS

          // Manage directional buttons
          #if ENABLED(REVERSE_MENU_DIRECTION)
            #define _ENCODER_UD_STEPS (ENCODER_STEPS_PER_MENU_ITEM * encoderDirection)
          #else
            #define _ENCODER_UD_STEPS ENCODER_STEPS_PER_MENU_ITEM
          #endif
          #if ENABLED(REVERSE_ENCODER_DIRECTION)
            #define ENCODER_UD_STEPS _ENCODER_UD_STEPS
            #define ENCODER_LR_PULSES ENCODER_PULSES_PER_STEP
          #else
            #define ENCODER_UD_STEPS -(_ENCODER_UD_STEPS)
            #define ENCODER_LR_PULSES -(ENCODER_PULSES_PER_STEP)
          #endif

          if (false) {
            // for the else-ifs below
          }
          #if BUTTON_EXISTS(UP)
            else if (BUTTON_PRESSED(UP)) {
              encoderDiff = -(ENCODER_UD_STEPS);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(DWN)
            else if (BUTTON_PRESSED(DWN)) {
              encoderDiff = ENCODER_UD_STEPS;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(LFT)
            else if (BUTTON_PRESSED(LFT)) {
              encoderDiff = -(ENCODER_LR_PULSES);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(RT)
            else if (BUTTON_PRESSED(RT)) {
              encoderDiff = ENCODER_LR_PULSES;
              next_button_update_ms = now + 300;
            }
          #endif

        #endif // LCD_HAS_DIRECTIONAL_BUTTONS

        buttons = newbutton;
        #if ENABLED(LCD_HAS_SLOW_BUTTONS)
          buttons |= slow_buttons;
        #endif

        #if ENABLED(ADC_KEYPAD)

          uint8_t newbutton_reprapworld_keypad = 0;
          buttons = 0;
          if (buttons_reprapworld_keypad == 0) {
            newbutton_reprapworld_keypad = get_ADC_keyValue();
            if (WITHIN(newbutton_reprapworld_keypad, 1, 8))
              buttons_reprapworld_keypad = _BV(newbutton_reprapworld_keypad - 1);
          }

        #elif ENABLED(REPRAPWORLD_KEYPAD)

          GET_SHIFT_BUTTON_STATES(buttons_reprapworld_keypad);

        #endif

      #else // !NEWPANEL

        GET_SHIFT_BUTTON_STATES(buttons);

      #endif

    } // next_button_update_ms

    // Manage encoder rotation
    #if ENABLED(REVERSE_MENU_DIRECTION) && ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff -= encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff += encoderDirection)
    #elif ENABLED(REVERSE_MENU_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff += encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
    #elif ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff--)
      #define ENCODER_DIFF_CCW (encoderDiff++)
    #else
      #define ENCODER_DIFF_CW  (encoderDiff++)
      #define ENCODER_DIFF_CCW (encoderDiff--)
    #endif
    #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: ENCODER_DIFF_CW; break; case _E2: ENCODER_DIFF_CCW; }

    uint8_t enc = 0;
    if (buttons & EN_A) enc |= B01;
    if (buttons & EN_B) enc |= B10;
    if (enc != lastEncoderBits) {
      switch (enc) {
        case encrot0: ENCODER_SPIN(encrot3, encrot1); break;
        case encrot1: ENCODER_SPIN(encrot0, encrot2); break;
        case encrot2: ENCODER_SPIN(encrot1, encrot3); break;
        case encrot3: ENCODER_SPIN(encrot2, encrot0); break;
      }
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (lcd_external_control) {
          ubl.encoder_diff = encoderDiff;   // Make the encoder's rotation available to G29's Mesh Editor
          encoderDiff = 0;                  // We are going to lie to the LCD Panel and claim the encoder
                                            // knob has not turned.
        }
      #endif
      lastEncoderBits = enc;
    }
  }

  #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && ENABLED(DETECT_DEVICE)
    bool lcd_detected() { return lcd.LcdDetected() == 1; }
  #else
    bool lcd_detected() { return true; }
  #endif

  #if ENABLED(G26_MESH_VALIDATION)
    void lcd_chirp() {
      #if ENABLED(LCD_USE_I2C_BUZZER)
        lcd.buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
      #elif PIN_EXISTS(BEEPER)
        buzzer.tone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
      #endif
    }
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
    bool is_lcd_clicked() { return LCD_CLICKED; }
    void wait_for_release() {
      while (is_lcd_clicked()) safe_delay(50);
      safe_delay(50);
    }
  #endif

#endif // ULTIPANEL

#if ENABLED(ADC_KEYPAD)

  typedef struct {
    uint16_t ADCKeyValueMin, ADCKeyValueMax;
    uint8_t  ADCKeyNo;
  } _stADCKeypadTable_;

  static const _stADCKeypadTable_ stADCKeyTable[] PROGMEM = {
    // VALUE_MIN, VALUE_MAX, KEY
    { 4000, 4096, BLEN_REPRAPWORLD_KEYPAD_F1 + 1 },     // F1
    { 4000, 4096, BLEN_REPRAPWORLD_KEYPAD_F2 + 1 },     // F2
    { 4000, 4096, BLEN_REPRAPWORLD_KEYPAD_F3 + 1 },     // F3
    {  300,  500, BLEN_REPRAPWORLD_KEYPAD_LEFT + 1 },   // LEFT
    { 1900, 2200, BLEN_REPRAPWORLD_KEYPAD_RIGHT + 1 },  // RIGHT
    {  570,  870, BLEN_REPRAPWORLD_KEYPAD_UP + 1 },     // UP
    { 2670, 2870, BLEN_REPRAPWORLD_KEYPAD_DOWN + 1 },   // DOWN
    { 1150, 1450, BLEN_REPRAPWORLD_KEYPAD_MIDDLE + 1 }, // ENTER
  };

  uint8_t get_ADC_keyValue(void) {
    if (thermalManager.ADCKey_count >= 16) {
      const uint16_t currentkpADCValue = thermalManager.current_ADCKey_raw >> 2;
      #if ENABLED(ADC_KEYPAD_DEBUG)
        SERIAL_PROTOCOLLN(currentkpADCValue);
      #endif
      thermalManager.current_ADCKey_raw = 0;
      thermalManager.ADCKey_count = 0;
      if (currentkpADCValue < 4000)
        for (uint8_t i = 0; i < ADC_KEY_NUM; i++) {
          const uint16_t lo = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMin),
                         hi = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMax);
          if (WITHIN(currentkpADCValue, lo, hi)) return pgm_read_byte(&stADCKeyTable[i].ADCKeyNo);
        }
    }
    return 0;
  }
#endif

#endif // ULTRA_LCD
