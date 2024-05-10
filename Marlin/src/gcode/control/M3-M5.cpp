/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "../gcode.h"
#include "../../feature/spindle_laser.h"
#include "../../module/planner.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

  
#if ENABLED(BRAILLERAP_ENABLE)

static uint32_t _time_power_on = 0;
static uint32_t _time_power_off = 0;
static uint32_t _time_planner = 0;
static uint32_t _total_time_power_on = 0;
static uint32_t _total_time_power_off = 0;
static uint32_t _total_time_planner = 0;

uint32_t get_total_power_on (void) { return _total_time_power_on;}
uint32_t get_total_power_off (void) { return _total_time_power_off;}
void brap_report (void)
{
  SERIAL_ECHOLNPGM("total on:", _total_time_power_on, " ");
  SERIAL_ECHOLNPGM("total off:", _total_time_power_off, " ");
  SERIAL_ECHOLNPGM("total planner:", _total_time_planner, " ");
  SERIAL_ECHOLNPGM("  ");

}
#endif
/**
 * Laser:
 *  M3 - Laser ON/Power (Ramped power)
 *  M4 - Laser ON/Power (Ramped power)
 *  M5 - Set power output to 0 (leaving inline mode unchanged).
 *
 *  M3I - Enable continuous inline power to be processed by the planner, with power
 *        calculated and set in the planner blocks, processed inline during stepping.
 *        Within inline mode M3 S-Values will set the power for the next moves e.g. G1 X10 Y10 powers on with the last S-Value.
 *        M3I must be set before using planner-synced M3 inline S-Values (LASER_POWER_SYNC).
 *
 *  M4I - Set dynamic mode which calculates laser power OCR based on the current feedrate.
 *
 *  M5I - Clear inline mode and set power to 0.
 *
 * Spindle:
 *  M3 - Spindle ON (Clockwise)
 *  M4 - Spindle ON (Counter-clockwise)
 *  M5 - Spindle OFF
 *  
 * Parameters:
 *  S<power> - Set power. S0 will turn the spindle/laser off.
 *
 *  If no PWM pin is defined then M3/M4 just turns it on or off.
 *
 *  At least 12.8kHz (50Hz * 256) is needed for Spindle PWM.
 *  Hardware PWM is required on AVR. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENA_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */
void GcodeSuite::M3_M4(const bool is_M4) {
  
  if (cutter.cutter_mode == CUTTER_MODE_STANDARD)
  {
    _time_planner = millis();
    planner.synchronize();   // Wait for previous movement commands (G0/G1/G2/G3) to complete before changing power
    _total_time_planner += millis () - _time_planner;
  }
  #if LASER_SAFETY_TIMEOUT_MS > 0
    reset_stepper_timeout(); // Reset timeout to allow subsequent G-code to power the laser (imm.)
  #endif

  #if ENABLED(BRAILLERAP_ENABLE) // Laser / spindle in BRAILLERAP Mode
  #if ENABLED(BRAILLERAP_DEBUG_ENABLE)
    SERIAL_ECHOLNPGM ("M3:", millis()) 
  #endif
  if (parser.seenval('S')) {
      uint16_t v = parser.value_ushort();
      if (v == 0)
      {
        _time_power_off = millis ();
        #if ENABLED(DEBUG_LEVELING_FEATURE)
            DEBUG_ECHO(">>> magnet off  ");
        #endif
        bool enabled = cutter.enabled();
        cutter.set_enabled(false);
        cutter.apply_power(SPINDLE_LASER_PWM_POWEROFF);
        
        if (enabled)
          //cutter.power_delay(false);
          delay (SPINDLE_LASER_POWERDOWN_DELAY);
        _total_time_power_off += millis () - _time_power_off;
      }
      else if (v == 1)
      {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          DEBUG_ECHO(">>> magnet on  ");
          DEBUG_ECHOLNPGM(">>> spindle pin (", SPINDLE_LASER_PWM_PIN, ")");
        #endif
        _time_power_on = millis ();
        cutter.set_enabled(true);
        cutter.apply_power(SPINDLE_LASER_PWM_POWERON);
        //cutter.power_delay(true);
        delay (SPINDLE_LASER_POWERUP_DELAY);
        _total_time_power_on += millis ()  - _time_power_on;
        _time_power_off = millis ();
        #if BRAILLERAP_AUTODISABL_MAGNET
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            DEBUG_ECHO(">>> magnet off  ");
          #endif
          cutter.set_enabled(false);
          cutter.apply_power(SPINDLE_LASER_PWM_POWEROFF);
          //cutter.power_delay(false);
          delay (SPINDLE_LASER_POWERDOWN_DELAY);
        #endif
        _total_time_power_off += millis () - _time_power_off;
      }
      else if (v > 1)
      {
        cutter.set_enabled(true);
        cutter.apply_power(SPINDLE_LASER_PWM_POWERON);
        //cutter.power_delay(true);
        delay (SPINDLE_LASER_POWERUP_DELAY);
        
      }
  }
  #else // Standard laser/spindle mode
    
  #if ENABLED(LASER_FEATURE)
    if (parser.seen_test('I')) {
      cutter.cutter_mode = is_M4 ? CUTTER_MODE_DYNAMIC : CUTTER_MODE_CONTINUOUS;
      cutter.inline_power(0);
      cutter.set_enabled(true);
    }
  #endif

  auto get_s_power = [] {
    float u;
    if (parser.seenval('S')) {
      const float v = parser.value_float();
      u = TERN(LASER_POWER_TRAP, v, cutter.power_to_range(v));
    }
    else if (cutter.cutter_mode == CUTTER_MODE_STANDARD)
      u = cutter.cpwr_to_upwr(SPEED_POWER_STARTUP);

    cutter.menuPower = cutter.unitPower = u;

    // PWM not implied, power converted to OCR from unit definition and on/off if not PWM.
    cutter.power = TERN(SPINDLE_LASER_USE_PWM, cutter.upower_to_ocr(u), u > 0 ? 255 : 0);
    return u;
  };

  if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS || cutter.cutter_mode == CUTTER_MODE_DYNAMIC) {  // Laser power in inline mode
    #if ENABLED(LASER_FEATURE)
      planner.laser_inline.status.isPowered = true;                                                 // M3 or M4 is powered either way
      get_s_power();                                                                                // Update cutter.power if seen
      #if ENABLED(LASER_POWER_SYNC)
        // With power sync we only set power so it does not effect queued inline power sets
        planner.buffer_sync_block(BLOCK_BIT_LASER_PWR);                                            // Send the flag, queueing inline power
      #else
        planner.synchronize();
        cutter.inline_power(cutter.power);
      #endif
    #endif
  }
  else {
    cutter.set_enabled(true);
    get_s_power();
    cutter.apply_power(
      #if ENABLED(SPINDLE_SERVO)
        cutter.unitPower
      #elif ENABLED(SPINDLE_LASER_USE_PWM)
        cutter.upower_to_ocr(cutter.unitPower)
      #else
        cutter.unitPower > 0 ? 255 : 0
      #endif
    );
    TERN_(SPINDLE_CHANGE_DIR, cutter.set_reverse(is_M4));
  }
  #endif
}

/**
 * M5 - Cutter OFF (when moves are complete)
 */
void GcodeSuite::M5() {
  planner.synchronize();
  cutter.power = 0;
  cutter.apply_power(0);                          // M5 just kills power, leaving inline mode unchanged
  if (cutter.cutter_mode != CUTTER_MODE_STANDARD) {
    if (parser.seen_test('I')) {
      TERN_(LASER_FEATURE, cutter.inline_power(cutter.power));
      cutter.set_enabled(false);                  // Needs to happen while we are in inline mode to clear inline power.
      cutter.cutter_mode = CUTTER_MODE_STANDARD;  // Switch from inline to standard mode.
    }
  }
  cutter.set_enabled(false);                      // Disable enable output setting
}

#endif // HAS_CUTTER
