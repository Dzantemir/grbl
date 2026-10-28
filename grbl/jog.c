/*
  jog.h - Jogging methods
  Part of Grbl

  Copyright (c) 2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


// Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
  // Initialize planner data struct for jogging motions.
  // NOTE: Spindle and coolant are allowed to fully function with overrides during a jog.
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_block->values.n;
  #endif

  // ---- ALOE PATCH: optional hard-limit motion lock ($28/$T) ----
  // $28=0 leaves stock Grbl Jog behavior untouched. While protection is active and ANY
  // limit input is pressed, ALL axes are locked in both directions; the way out is the
  // firmware-side escape command $T<axis><dir> or disabling $28. The limit inputs are
  // sampled for the $28 feature only, so stock builds keep their original code path.
  uint8_t limit_state = 0;
  if (settings.limit_motion_protection) {
    limit_state = limits_get_state();
  }

  // Stock soft-limit ($20) travel check, exactly as in vanilla Grbl (FIXED7: the
  // fork-added bypass is gone - the $T escape never goes through the jog path).
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) {
      return(STATUS_TRAVEL_EXCEEDED);
    }
  }

  if (limit_protection_active() && limit_state) {
    float current_mpos[N_AXIS];
    system_convert_array_steps_to_mpos(current_mpos, sys_position);
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
      // Any active limit input locks every axis; no direction guessing is involved.
      // FIXED7: there is no jog-based recovery anymore - the only way off a pressed
      // switch is the firmware-side $T<axis><dir> escape (see system.c / limits.c).
      if (gc_block->values.xyz[idx] != current_mpos[idx]) { return(STATUS_TRAVEL_EXCEEDED); }
    }
  }
  // ---- END PATCH ----

  // Valid jog motion. Plan, set state, and execute.
  mc_line(gc_block->values.xyz,pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { // Check if there is a block to execute.
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  // NOTE: Manual start. No state machine required.
    }
  }

  return(STATUS_OK);
}
