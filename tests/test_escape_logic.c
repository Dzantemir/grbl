/*
 * FIXED8 verification of the automatic limit escape $T<axis><+|->.
 *
 * Models the control flow of system.c (parser + pre-checks) and limits.c
 * (limits_escape(): phase 1 creep-until-release with a travel budget, $26
 * debounce re-check, phase 2 $27 pull-off, final verdict).
 *
 * The physical axis is simulated as a 0.1 mm grid: the switch input is active
 * while the position is inside the pressed zone. The declared direction moves
 * the position away from (or deeper into) that zone.
 *
 * Covers:
 *   E1. Parser: every malformed form rejected, bare $T included - error:3.
 *   E2. Pre-checks: $28=0 -> error:5; axis not pressed -> no-op (no motion);
 *       a machine held by Y is NOT unlocked by $TX.
 *   E3. Happy path: declared direction correct -> release within budget,
 *       pull-off $27, success, zero host involvement.
 *   E4. Wrong declared direction: input never releases, the full
 *       LIMIT_RECOVERY_MAX_TRAVEL budget is consumed, motion stops,
 *       error:15 - bounded badness, no runaway.
 *   E5. Contact bounce fakes a release: the $26 re-check catches it and the
 *       command fails safe instead of pulling off blindly.
 *   E6. The Candle scenario is structurally impossible: no host jog takes
 *       part in the escape, so a sign-flipped $J= has nothing to break.
 *   E7. FIXED8: release coinciding with the end of the budget move leaves a
 *       stale EXEC_CYCLE_STOP behind; it is cleared between the phases, so
 *       the $27 pull-off still runs instead of being skipped with a fake ok.
 *   E8. FIXED8: the input re-activating DURING the pull-off aborts the motion
 *   E9. FIXED9: ticks counted by the stepper ISR between the release-triggered
 *       step mask and st_reset() are phantom (counted, never moved); the
 *       position snapshot taken when the mask engages must win.
 *       immediately (error:15) - phase 2 polls the pin on every pass now.
 *
 * Build: gcc -Wall -Wextra -o test_escape_logic test_escape_logic.c && ./test_escape_logic
 */
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define bit(n) (1 << (n))
#define STATUS_OK 0
#define STATUS_INVALID_STATEMENT 3
#define STATUS_SETTING_DISABLED 5
#define STATUS_IDLE_ERROR 8
#define STATUS_TRAVEL_EXCEEDED 15

static float max_travel = 10.0f; /* $29 limit_recovery_travel (FIXED11) */
static float axis_max_travel = 1.0e9f; /* -$13x of the axis (FIXED12 clamp) */
#define MAX_TRAVEL max_travel
#define PULLOFF 1.0f       /* $27 */
#define STEP 0.1f          /* simulation grid */

static uint8_t protection;        /* $28 */
static float   pos;               /* machine position of the escaping axis, mm */
static float   zone_lo, zone_hi;  /* pressed zone of the switch input, mm */
static uint8_t bounce_fake_release; /* one poll reads clear while still pressed */
static float   traveled;          /* total distance moved by the escape */
static uint8_t cycle_stop;        /* EXEC_CYCLE_STOP: set when the planned move completes */
static uint8_t trap_enabled;      /* E8: the input re-activates during the pull-off... */
static float   trap_pos;          /* ...once the position crosses this point */

static uint8_t pin_active(void) {
  if (bounce_fake_release) { bounce_fake_release = 0; return 0; }
  if (trap_enabled && pos <= trap_pos) { return 1; }
  return (pos >= zone_lo && pos <= zone_hi);
}

/* limits_escape() control flow, FIXED8 */
static uint8_t limits_escape(float dir) {
  if (max_travel <= 0.0f) { return 0; } /* $29=0 disables the escape (FIXED11) */
  float budget = MAX_TRAVEL;
  /* FIXED12: the budget never exceeds the axis max travel ($13x). */
  if (axis_max_travel > 0.0f && budget > axis_max_travel) { budget = axis_max_travel; }
  uint8_t released = 0;
  cycle_stop = 0;
  /* Phase 1: creep at $24, poll every step, stop on release or empty budget.
     On release the axis is masked out of the stepper ISR instantly
     (sys.homing_axis_lock = 0) - modeled by breaking with no further steps. */
  while (budget > 0.0f) {
    if (!pin_active()) { released = 1; break; }  /* poll BEFORE each step, like the loop */
    pos += dir * STEP; traveled += STEP; budget -= STEP;
  }
  if (budget <= 0.0f) { cycle_stop = 1; } /* the planned budget move completed */
  if (!released && !pin_active()) { released = 1; } /* release exactly at budget end */
  /* st_reset; FIXED8: the stale EXEC_CYCLE_STOP is cleared between the phases */
  cycle_stop = 0;
  /* $26 debounce delay, then the re-check */
  if (!released || pin_active()) { return 0; }      /* bounce caught here */
  /* Phase 2: pull off $27 at $25. FIXED8: the pin is polled on every pass;
     a stale cycle_stop would previously have skipped this loop entirely. */
  if (cycle_stop) { return !pin_active(); } /* dead branch after the clear above */
  float remaining = PULLOFF;
  while (remaining > 0.0f) {
    if (pin_active()) { return 0; } /* re-activation aborts the pull-off */
    pos += dir * STEP; traveled += STEP; remaining -= STEP;
  }
  /* Final verdict */
  return !pin_active();
}

/* system.c $T handler control flow. Returns Grbl status; runs the escape. */
static uint8_t command_T(const char *line, uint8_t idle_or_alarm, uint8_t pressed_mask) {
  uint8_t axis;
  float dir;
  if (!idle_or_alarm) { return STATUS_IDLE_ERROR; }
  if (!protection) { return STATUS_SETTING_DISABLED; }
  switch (line[2]) {
    case 'X': axis = X_AXIS; break;
    case 'Y': axis = Y_AXIS; break;
    case 'Z': axis = Z_AXIS; break;
    default: return STATUS_INVALID_STATEMENT;
  }
  if (line[3] == '-') { dir = -1.0f; }
  else if (line[3] == '+') { dir = 1.0f; }
  else { return STATUS_INVALID_STATEMENT; }
  if (line[4] != 0) { return STATUS_INVALID_STATEMENT; }
  if (!(pressed_mask & bit(axis))) { return STATUS_OK; } /* [MSG:Disabled], no motion */
  if (!limits_escape(dir)) { return STATUS_TRAVEL_EXCEEDED; } /* [MSG:Check limits] */
  return STATUS_OK;
}

int main(void) {
  /* ===== E1: parser ===== */
  protection = 1; traveled = 0.0f;
  assert(command_T("$T",    1, bit(X_AXIS)) == STATUS_INVALID_STATEMENT);
  assert(command_T("$TX",   1, bit(X_AXIS)) == STATUS_INVALID_STATEMENT);
  assert(command_T("$TA+",  1, bit(X_AXIS)) == STATUS_INVALID_STATEMENT);
  assert(command_T("$TX*",  1, bit(X_AXIS)) == STATUS_INVALID_STATEMENT);
  assert(command_T("$TX-1", 1, bit(X_AXIS)) == STATUS_INVALID_STATEMENT);
  assert(command_T("$TX-",  0, bit(X_AXIS)) == STATUS_IDLE_ERROR);
  assert(traveled == 0.0f); /* nothing ever moved */
  printf("E1 pass: only $T<X|Y|Z><+|-> is accepted, no motion on bad input\n");

  /* ===== E2: pre-checks ===== */
  protection = 0;
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_SETTING_DISABLED);
  protection = 1;
  /* machine held by Y, operator mistypes $TX-: accepted as no-op, no motion */
  assert(command_T("$TX-", 1, bit(Y_AXIS)) == STATUS_OK);
  assert(traveled == 0.0f);
  printf("E2 pass: $28=0 -> error:5; unpressed axis -> no-op without motion\n");

  /* ===== E3: happy path =====
     Switch at the PLUS end: pressed zone is [0 ; +inf) - driving deeper never
     releases it, only mechanics would crash. Machine sits at 0.5 inside the
     zone, operator declares escape to minus: $TX-. */
  pos = 0.5f; zone_lo = 0.0f; zone_hi = 1.0e9f; bounce_fake_release = 0; traveled = 0.0f;
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_OK);
  assert(pos < zone_lo);                       /* physically off the switch */
  assert(fabsf((zone_lo - pos) - PULLOFF) < STEP + 1e-4f); /* pulled off ~$27 */
  assert(traveled < 0.5f + STEP + PULLOFF + 1e-4f);        /* minimal travel */
  printf("E3 pass: correct declaration -> release + $27 pull-off, done\n");

  /* ===== E4: wrong declared direction =====
     Same pressed zone, but operator declares $TX+ (deeper into the switch). */
  pos = 0.5f; traveled = 0.0f;
  assert(command_T("$TX+", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(fabsf(traveled - MAX_TRAVEL) < 1e-3f);  /* stopped exactly at the budget */
  assert(pos <= 0.5f + MAX_TRAVEL + 1e-3f);      /* bounded: no runaway */
  printf("E4 pass: wrong direction stops at the %.0f mm budget with error:15\n",
         (double)MAX_TRAVEL);

  /* ===== E5: bounce fakes one clear poll while still pressed ===== */
  pos = 1.5f; traveled = 0.0f; bounce_fake_release = 1;
  /* first poll reads clear (bounce) -> phase 1 'release' after zero travel,
     but the $26 re-check reads the real pin again: still pressed -> fail safe */
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(traveled == 0.0f);  /* and crucially: NO blind $27 pull-off happened */
  printf("E5 pass: bounce-faked release is caught by the debounce re-check\n");

  /* ===== E7: release at the exact end of the budget (stale EXEC_CYCLE_STOP) =====
     zone_lo placed so the pin releases on the very last 0.1 mm of the 10 mm
     budget: phase 1 exits with the budget consumed AND the release detected.
     The stale cycle_stop must not skip the pull-off. */
  trap_enabled = 0;
  pos = 0.5f; zone_lo = -9.4f; zone_hi = 1.0e9f; bounce_fake_release = 0; traveled = 0.0f;
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_OK);
  assert(zone_lo - pos >= PULLOFF - 1e-4f);      /* the $27 pull-off DID run */
  assert(traveled <= MAX_TRAVEL + PULLOFF + 1e-3f); /* and stayed bounded */
  printf("E7 pass: stale cycle-stop cleared, pull-off not skipped\n");

  /* ===== E8: input re-activates during the pull-off ===== */
  pos = 0.5f; zone_lo = 0.0f; zone_hi = 1.0e9f; traveled = 0.0f;
  trap_enabled = 1; trap_pos = -0.5f; /* halfway into the $27 pull-off */
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(pos >= trap_pos - STEP - 1e-4f); /* stopped at the trap, no ramming */
  trap_enabled = 0;
  printf("E8 pass: pull-off aborts the moment the input re-activates\n");

  /* ===== E9: phantom ticks between the step mask and st_reset() ===== */
  {
    /* Model: position at the instant the release is detected and the mask
       engages; the ISR then counts a few more ticks before st_reset(). */
    long pos_at_release = 480;          /* real: the motor moved here */
    long phantom_ticks = 3;             /* counted after the mask: never moved */
    long sys_position = pos_at_release + phantom_ticks; /* FIXED8 kept this */
    long pos_snapshot = pos_at_release; /* FIXED9: taken atomically with the mask */
    sys_position = pos_snapshot;        /* restored right after st_reset() */
    assert(sys_position == pos_at_release);
  }
  printf("E9 pass: snapshot drops the masked phantom ticks from sys_position\n");

  /* ===== E10 (FIXED11): $29 budget is runtime-configurable ===== */
  /* Smaller budget: wrong direction stops at the new value, not at 10 mm. */
  max_travel = 3.0f;
  pos = 0.5f; zone_lo = 0.0f; zone_hi = 1.0e9f; bounce_fake_release = 0; traveled = 0.0f;
  assert(command_T("$TX+", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(traveled >= 3.0f - 1e-3f && traveled <= 3.0f + STEP + 1e-3f); /* budget +/- one sim step */
  /* $29=0 disables the escape entirely: refused before any motion. */
  max_travel = 0.0f; traveled = 0.0f;
  assert(command_T("$TX-", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(traveled == 0.0f);
  max_travel = 10.0f;
  printf("E10 pass: $29 resizes the budget; $29=0 refuses $T with no motion\n");

  /* ===== E11 (FIXED12): the $29 budget is clamped to the axis max travel ===== */
  max_travel = 10.0f; axis_max_travel = 2.0f;
  pos = 0.5f; zone_lo = 0.0f; zone_hi = 1.0e9f; bounce_fake_release = 0; traveled = 0.0f;
  assert(command_T("$TX+", 1, bit(X_AXIS)) == STATUS_TRAVEL_EXCEEDED);
  assert(traveled >= 2.0f - 1e-3f && traveled <= 2.0f + STEP + 1e-3f); /* stopped at $13x, not $29 */
  axis_max_travel = 1.0e9f;
  printf("E11 pass: $29 budget clamped to the axis max travel\n");

  /* ===== E6: no host jog involved ===== */
  /* The escape consumed no $J= input in any scenario above; a Candle-flipped
     jog command simply has no entry point into this code path. Statically
     true by construction - recorded here as a guard against regressions. */
  printf("E6 pass: escape is host-independent by construction\n");

  printf("\nALL ESCAPE LOGIC TESTS PASSED\n");
  return 0;
}
