/*
 * Decision-table tests for the $28 limit lock (FIXED8).
 *
 * The protection is a pure binary gate with NO unlock flag: while $28=1, ANY
 * active limit input locks ALL axes in both directions (jog error:15, g-code
 * attempt -> ALARM:1, $H refused). No direction is ever latched or guessed - with
 * LIMITS_TWO_SWITCHES_ON_AXES both ends of an axis share one input, so the
 * firmware cannot know which end is pressed. The ONLY way off a pressed switch
 * is the synchronous firmware escape $T<axis><+|-> (covered by
 * test_escape_logic.c), which runs as a homing-style system motion and never
 * passes through these gates.
 *
 * Build: gcc -Wall -Wextra -o test_guard_logic test_guard_logic.c && ./test_guard_logic
 */
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#define N_AXIS 3
#define X_AXIS 0
#define Y_AXIS 1
#define bit(n) (1 << (n))
#define STATUS_OK 0
#define STATUS_TRAVEL_EXCEEDED 15

static uint8_t protection;
static uint8_t limit_inputs;

/* system.h macro, FIXED7: no recovery flag anymore */
#define limit_protection_active() (protection)

static uint8_t limits_get_state(void) {
  return limit_inputs; /* stock-shaped: reports pins, nothing else */
}

/* jog.c decision block, verbatim semantics. FIXED10: unconditional refusal -
   the former per-axis target==current comparison let a zero-length jog slip
   through to mc_line(), whose $28 guard answered with a full ALARM:1. */
static uint8_t jog_status(float *target, float *current) {
  (void)target; (void)current;
  uint8_t limit_state = 0;
  if (protection) { limit_state = limits_get_state(); }
  if (limit_protection_active() && limit_state) {
    return STATUS_TRAVEL_EXCEEDED;
  }
  return STATUS_OK;
}

/* mc_line() decision block, FIXED8: controlled stop + ALARM:1 instead of the
   former silent clamp. Returns 1 when the alarm fires (motion refused). */
static uint8_t mc_line_alarm(void) {
  if (limit_protection_active() && limits_get_state()) {
    return 1; /* feed hold (if cycle) -> mc_reset -> EXEC_ALARM_HARD_LIMIT */
  }
  return 0; /* motion proceeds to the planner untouched */
}


/* protocol.c EXEC_CYCLE_STOP gate (verbatim semantics of the branch at
   protocol.c:376): after a feed hold completes, the state machine returns to
   IDLE only for "motion complete" events - which requires sys.soft_limit (or
   jog cancel). Without the flag it parks in HOLD waiting for a resume (~). */
#define GSTATE_IDLE 0
#define GSTATE_HOLD 1
static int cycle_stop_result_state(int state, int soft_limit, int jog_cancel) {
  if ((state == GSTATE_HOLD) && !soft_limit && !jog_cancel) {
    return GSTATE_HOLD; /* hold complete: wait for resume (~) or reset */
  }
  return GSTATE_IDLE;   /* motion complete: cycle/jog/homing/soft-limit events */
}

/* mc_line() guard, FIXED9: can the "wait for IDLE" loop exit?
   FIXED8 regression: the guard copied the limits_soft_check() wait loop but
   not the sys.soft_limit flag -> the loop span forever in HOLD, and a ~
   received meanwhile could resume buffered motion into the pressed switch. */
static int guard_wait_exits(int sets_soft_limit) {
  int soft_limit = sets_soft_limit;  /* FIXED9 sets it before waiting */
  int state = GSTATE_HOLD;           /* feed hold just completed */
  state = cycle_stop_result_state(state, soft_limit, 0);
  return state == GSTATE_IDLE;       /* loop condition: while (state != IDLE) */
}

int main(void) {
  float cur[3] = {10.0f, 0.0f, 0.0f};

  /* 1. No limit active: everything moves, protection irrelevant to gate */
  protection = 1; limit_inputs = 0;
  float t[3] = {11.0f, 5.0f, -2.0f};
  assert(jog_status(t, cur) == STATUS_OK);
  printf("1. no limit active: free motion\n");

  /* 2. X limit pressed: every axis locked in every direction */
  limit_inputs = bit(X_AXIS);
  float txp[3] = {10.001f, 0, 0}, txm[3] = {9.999f, 0, 0};
  float ty[3] = {10.0f, 1.0f, 0}, tz[3] = {10.0f, 0, 1.0f};
  assert(jog_status(txp, cur) == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(txm, cur) == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(ty, cur)  == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(tz, cur)  == STATUS_TRAVEL_EXCEEDED);
  assert(mc_line_alarm() == 1); /* g-code motion attempt -> ALARM:1, not "ok" */
  /* FIXED10: even a zero-length jog answers error:15 while a switch is pressed
     (it used to slip through to mc_line() and trip ALARM:1 - 5th audit). */
  assert(jog_status(cur, cur) == STATUS_TRAVEL_EXCEEDED);
  printf("2. X pressed: all axes blocked (jog error:15, g-code ALARM:1)\n");

  /* 3. Historical bounce scenario: X held, Y moving, X input bounces.
        No latch exists, so after the resulting alarm+reset the machine is
        simply fully locked - stale DIR levels can never poison a decision. */
  limit_inputs = bit(X_AXIS);
  for (int stale_dir = 0; stale_dir < 4; stale_dir++) {
    (void)stale_dir; /* no direction state exists to consult */
    assert(jog_status(txp, cur) == STATUS_TRAVEL_EXCEEDED);
    assert(jog_status(txm, cur) == STATUS_TRAVEL_EXCEEDED);
  }
  printf("3. switch bounce cannot matter: no direction state is kept\n");

  /* 4. Multiple limits pressed: same total lock */
  limit_inputs = bit(X_AXIS) | bit(Y_AXIS);
  assert(jog_status(ty, cur)  == STATUS_TRAVEL_EXCEEDED);
  assert(jog_status(txm, cur) == STATUS_TRAVEL_EXCEEDED);
  printf("4. multi-axis press: total lock, only the $T escape can move\n");

  /* 5. Release: gate opens by itself, no flags to clean up */
  limit_inputs = 0;
  assert(jog_status(t, cur) == STATUS_OK);
  limit_inputs = bit(Y_AXIS); /* a later press locks again instantly */
  assert(jog_status(ty, cur) == STATUS_TRAVEL_EXCEEDED);
  printf("5. release opens the gate, re-press locks it again - stateless\n");

  /* 6. $28=0: pins may be pressed, the gate never engages */
  protection = 0;
  limit_inputs = bit(X_AXIS) | bit(Y_AXIS);
  assert(jog_status(txp, cur) == STATUS_OK);
  assert(jog_status(ty, cur)  == STATUS_OK);
  assert(mc_line_alarm() == 0); /* no alarm, target reaches the planner intact */
  printf("6. $28=0: gate fully inactive, stock behavior\n");

  /* G-HOLD: the FIXED9 guard must exit the post-hold wait loop */
  assert(guard_wait_exits(1) == 1); /* FIXED9: soft_limit set -> HOLD -> IDLE -> alarm */
  assert(guard_wait_exits(0) == 0); /* FIXED8 bug model: stuck in HOLD forever */
  printf("G-HOLD pass: sys.soft_limit releases the wait loop into IDLE\n");

  printf("\nALL GUARD LOGIC TESTS PASSED\n");
  return 0;
}
