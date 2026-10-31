/*
 * Acceptance tests for the operator's contract (build 20261029_ALOE_FIXED8):
 *
 *   R1. $28=1 and ANY limit input active: ALL axes are locked in BOTH
 *       directions (jog error:15, a g-code motion attempt raises ALARM:1
 *       via a stock-soft-limit-style controlled stop) and $H is refused. There is
 *       no unlock flag: the lock is a stateless function of the pins.
 *   R2. $T<axis><+|-> performs the whole escape INSIDE the firmware as a
 *       synchronous homing-style motion: creep at $24 up to the budget until
 *       the input releases, then pull off $27 at $25. ok on success,
 *       error:15 when the budget runs out with the input still active.
 *       No host jog participates - a sign-flipped $J= cannot move the machine.
 *   R3. After a successful escape the input is free, so the gate is open by
 *       itself; the operator homes next. A later press locks again instantly.
 *   R4. Bare/malformed $T -> error:3. $T on an unpressed axis -> no-op that
 *       does NOT unlock a machine held by another axis. $28=0 -> error:5 and
 *       fully stock behavior, $20 soft limits are stock in every mode.
 *
 * Re-hosts the decision expressions from jog.c, motion_control.c, system.c.
 *
 * Build: gcc -Wall -Wextra -o test_requirements test_requirements.c && ./test_requirements
 */
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#define N_AXIS 3
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define bit(n) (1 << (n))
#define STATUS_OK 0
#define STATUS_INVALID_STATEMENT 3
#define STATUS_SETTING_DISABLED 5
#define STATUS_TRAVEL_EXCEEDED 15

static uint8_t protection;   /* $28 */
static uint8_t limit_inputs; /* limit pins, bit per axis */

#define limit_protection_active() (protection)
static uint8_t limits_get_state(void) { return limit_inputs; }

/* jog.c gate */
static uint8_t jog_status(float *target, float *current) {
  uint8_t limit_state = 0;
  if (protection) { limit_state = limits_get_state(); }
  if (limit_protection_active() && limit_state) {
    uint8_t i;
    for (i = 0; i < N_AXIS; i++) {
      if (target[i] != current[i]) { return STATUS_TRAVEL_EXCEEDED; }
    }
  }
  return STATUS_OK;
}

/* $H interlock, system.c */
static uint8_t homing_blocked(void) {
  return (limit_protection_active() && limits_get_state()) ? 1 : 0;
}

/* $T pre-checks, system.c (motion itself is covered by test_escape_logic.c).
   Returns the status the command would produce BEFORE any motion; *runs is set
   when the escape motion would actually start. */
static uint8_t command_T_prechecks(const char *line, uint8_t *runs) {
  uint8_t axis;
  *runs = 0;
  if (!protection) { return STATUS_SETTING_DISABLED; }
  switch (line[2]) {
    case 'X': axis = X_AXIS; break;
    case 'Y': axis = Y_AXIS; break;
    case 'Z': axis = Z_AXIS; break;
    default: return STATUS_INVALID_STATEMENT;
  }
  if (line[3] != '-' && line[3] != '+') { return STATUS_INVALID_STATEMENT; }
  if (line[4] != 0) { return STATUS_INVALID_STATEMENT; }
  if (!(limits_get_state() & bit(axis))) { return STATUS_OK; } /* no-op */
  *runs = 1;
  return STATUS_OK;
}

int main(void) {
  float cur[3] = {10.0f, 20.0f, 5.0f};
  float x_plus[3]  = {10.001f, 20.0f, 5.0f};
  float x_minus[3] = {9.999f,  20.0f, 5.0f};
  float y_move[3]  = {10.0f,   21.0f, 5.0f};
  uint8_t runs;

  /* ===== R1 ===== */
  protection = 1;
  limit_inputs = bit(X_AXIS);
  assert(jog_status(x_plus, cur)  == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(x_minus, cur) == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(y_move, cur)  == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(cur, cur)     == STATUS_OK); /* zero-length passes */
  assert(homing_blocked() == 1);
  printf("R1 pass: any active limit locks ALL axes both ways, $H blocked\n");

  /* ===== R2: $T runs the escape itself ===== */
  assert(command_T_prechecks("$TX-", &runs) == STATUS_OK && runs == 1);
  /* while the synchronous escape runs, protocol_main_loop is busy inside
     system_execute_line: no $J= line can even be parsed concurrently. The
     Candle sign-flip bug has no entry point. Motion specifics: see
     test_escape_logic.c (budget, pull-off, bounce, wrong direction). */
  printf("R2 pass: $TX- starts the firmware escape, host jog not involved\n");

  /* ===== R3: after release the gate is simply open ===== */
  limit_inputs = 0; /* escape succeeded: input free */
  assert(jog_status(x_plus, cur) == STATUS_OK);
  assert(homing_blocked() == 0); /* $H is the expected next step */
  limit_inputs = bit(Y_AXIS); /* a later press locks again, statelessly */
  assert(jog_status(y_move, cur) == STATUS_TRAVEL_EXCEEDED);
  assert(homing_blocked() == 1);
  printf("R3 pass: stateless gate - open when free, locked when pressed\n");

  /* ===== R4 ===== */
  limit_inputs = bit(Y_AXIS);
  assert(command_T_prechecks("$T",    &runs) == STATUS_INVALID_STATEMENT && !runs);
  assert(command_T_prechecks("$TX",   &runs) == STATUS_INVALID_STATEMENT && !runs);
  assert(command_T_prechecks("$TA-",  &runs) == STATUS_INVALID_STATEMENT && !runs);
  assert(command_T_prechecks("$TX-9", &runs) == STATUS_INVALID_STATEMENT && !runs);
  /* $TX- while only Y is pressed: polite no-op, machine stays locked */
  assert(command_T_prechecks("$TX-", &runs) == STATUS_OK && runs == 0);
  assert(jog_status(y_move, cur) == STATUS_TRAVEL_EXCEEDED);
  /* $28=0: stock Grbl + error:5 on $T */
  protection = 0;
  limit_inputs = bit(X_AXIS);
  assert(jog_status(x_plus, cur)  == STATUS_OK);
  assert(jog_status(x_minus, cur) == STATUS_OK);
  assert(homing_blocked() == 0);
  assert(command_T_prechecks("$TX-", &runs) == STATUS_SETTING_DISABLED && !runs);
  /* re-enabling $28 restores the lock immediately, no reset needed */
  protection = 1;
  assert(jog_status(x_plus, cur) == STATUS_TRAVEL_EXCEEDED);
  printf("R4 pass: malformed $T error:3, unpressed-axis no-op, $28=0 stock\n");

  printf("\nALL REQUIREMENT TESTS PASSED\n");
  return 0;
}
