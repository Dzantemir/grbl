/*
 * Host-side verification of the EEPROM migrations: v10 -> v12 and v11 -> v12
 * ($28 byte appended in FIXED-era v11, $29 float appended in FIXED11/v12).
 * The EEPROM is emulated as a byte array; the checksum routines are copied
 * verbatim from grbl/eeprom.c. The test runs BOTH the old (buggy) and the new
 * (fixed) migration algorithms against a realistic v10 record.
 *
 * Build: gcc -Wall -o test_eeprom_migration test_eeprom_migration.c && ./test_eeprom_migration
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <assert.h>

#define N_AXIS 3
#define EEPROM_ADDR_GLOBAL 1U

/* ---- emulated EEPROM (1 KiB, like ATmega328p) ---- */
static uint8_t eeprom[1024];
static uint8_t eeprom_get_char(unsigned int addr) { return eeprom[addr]; }
static void eeprom_put_char(unsigned int addr, uint8_t v) { eeprom[addr] = v; }

/* Verbatim copies from grbl/eeprom.c (including the stock '||' quirk). */
static void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) {
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++));
  }
  eeprom_put_char(destination, checksum);
}
static int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) {
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;
    *(destination++) = data;
  }
  return(checksum == eeprom_get_char(source));
}

/* settings_t as it was in SETTINGS_VERSION 10 (before $28). */
typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
} settings_v10_t;

/* settings_t as it was in SETTINGS_VERSION 11 (v10 + the $28 byte). */
typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
  uint8_t limit_motion_protection;
} settings_v11_t;

/* current settings_t (v12): v11 + appended $29 float. */
typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
  uint8_t limit_motion_protection;
  float limit_recovery_travel;
} settings_t;

#define DEFAULT_RECOVERY_TRAVEL 10.0f /* LIMIT_RECOVERY_MAX_TRAVEL, default for $29 */

static int old_migration(settings_v11_t *out) {
  /* The exact buggy expression from the pre-fix firmware (v11 era):
     legacy size derived from the NEW struct size minus one byte. Padding
     makes this larger than the real v10 record, so the checksum byte is read
     from the wrong address and the check almost always fails. */
  uint8_t legacy_settings_size = sizeof(settings_v11_t) - sizeof(out->limit_motion_protection);
  if (!(memcpy_from_eeprom_with_checksum((char*)out, EEPROM_ADDR_GLOBAL, legacy_settings_size))) {
    return 0;
  }
  out->limit_motion_protection = 1;
  return 1;
}

static int new_migration_v10(settings_t *out) {
  settings_v10_t legacy;
  if (!(memcpy_from_eeprom_with_checksum((char*)&legacy, EEPROM_ADDR_GLOBAL, sizeof(legacy)))) {
    return 0;
  }
  memset(out, 0, sizeof(*out));
  memcpy(out, &legacy, sizeof(legacy));
  out->limit_motion_protection = 1;
  out->limit_recovery_travel = DEFAULT_RECOVERY_TRAVEL;
  return 1;
}

static int new_migration_v11(settings_t *out) {
  settings_v11_t legacy;
  if (!(memcpy_from_eeprom_with_checksum((char*)&legacy, EEPROM_ADDR_GLOBAL, sizeof(legacy)))) {
    return 0;
  }
  memset(out, 0, sizeof(*out));
  memcpy(out, &legacy, sizeof(legacy));
  out->limit_recovery_travel = DEFAULT_RECOVERY_TRAVEL;
  return 1;
}

int main(void) {
  printf("sizeof(settings_v10_t) = %zu\n", sizeof(settings_v10_t));
  printf("sizeof(settings_v11_t) = %zu\n", sizeof(settings_v11_t));
  printf("sizeof(settings_t)     = %zu\n", sizeof(settings_t));
  /* The layout guards the firmware relies on: */
  assert(sizeof(settings_v10_t) == offsetof(settings_v11_t, limit_motion_protection));
  assert(sizeof(settings_v10_t) == offsetof(settings_t, limit_motion_protection));
  assert(sizeof(settings_v11_t) == offsetof(settings_t, limit_recovery_travel));

  /* Build a realistic v10 record with distinctive, non-default machine values. */
  settings_v10_t v10;
  memset(&v10, 0, sizeof(v10));
  v10.steps_per_mm[0] = 800.0f; v10.steps_per_mm[1] = 800.0f; v10.steps_per_mm[2] = 4000.0f;
  v10.max_rate[0] = 12000.0f;
  v10.acceleration[1] = 250.0f*3600.0f;
  v10.max_travel[0] = -300.0f;
  v10.pulse_microseconds = 5;
  v10.step_invert_mask = 0;
  v10.dir_invert_mask = 2;       /* user's $3=2 */
  v10.flags = 0b00111000;        /* hard limit + homing + soft limit on */
  v10.homing_dir_mask = 1;
  v10.homing_debounce_delay = 250;
  v10.homing_pulloff = 3.0f;
  v10.junction_deviation = 0.02f;

  /* Simulate a *used* EEPROM: areas outside the record hold leftover data
     (not the 0xFF of a blank chip), which is the common case in the field. */
  memset(eeprom, 0x55, sizeof(eeprom));
  eeprom[0] = 10; /* SETTINGS_VERSION 10 */
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&v10, sizeof(v10));
  /* checksum now sits at EEPROM_ADDR_GLOBAL + 92; the old code looks at +95. */
  printf("\ncorrect checksum address = %u, address the old code checks = %u\n",
         (unsigned)(EEPROM_ADDR_GLOBAL + sizeof(v10)),
         (unsigned)(EEPROM_ADDR_GLOBAL + (sizeof(settings_v11_t) - 1)));

  settings_t out;
  settings_v11_t out11;

  /* 1) Old algorithm: expected to FAIL (this failure wiped all EEPROM data). */
  memset(&out11, 0xAA, sizeof(out11));
  int old_ok = old_migration(&out11);
  printf("[old migration] returned %s\n", old_ok ? "OK" : "FAIL (-> firmware restored ALL EEPROM defaults)");
  assert(old_ok == 0);

  /* 2) New algorithm: must succeed and preserve every value. */
  memset(&out, 0xAA, sizeof(out));
  int new_ok = new_migration_v10(&out);
  printf("[new migration v10] returned %s\n", new_ok ? "OK" : "FAIL");
  assert(new_ok == 1);
  assert(memcmp(&out, &v10, sizeof(v10)) == 0);
  assert(out.limit_motion_protection == 1);
  assert(out.limit_recovery_travel == DEFAULT_RECOVERY_TRAVEL);
  assert(out.steps_per_mm[0] == 800.0f);
  assert(out.dir_invert_mask == 2);
  assert(out.max_travel[0] == -300.0f);
  assert(out.homing_debounce_delay == 250);
  printf("[new migration v10] all machine settings preserved, $28 -> 1, $29 -> 10\n");

  /* 3) Corrupted record: damage the stored checksum byte directly. (The stock
     Grbl checksum is a weak, non-cryptographic 8-bit sum due to its '||' quirk,
     so flipping the checksum byte itself is the deterministic way to exercise
     the failure path.) The firmware must then fall back to safe defaults. */
  unsigned int checksum_addr = EEPROM_ADDR_GLOBAL + (unsigned int)sizeof(settings_v10_t);
  uint8_t good_checksum = eeprom[checksum_addr];
  eeprom[checksum_addr] = (uint8_t)(good_checksum ^ 0xFF);
  assert(new_migration_v10(&out) == 0);
  eeprom[checksum_addr] = good_checksum; /* restore for the next tests */
  printf("[new migration v10] correctly rejects a corrupted v10 record\n");

  /* 4) v11 -> v12 migration: a FIXED8..FIXED10 user updates to FIXED11.
     Use $28=0 (non-default) to prove the value is preserved, not re-defaulted. */
  settings_v11_t v11;
  memset(&v11, 0, sizeof(v11));
  memcpy(&v11, &v10, sizeof(v10));
  v11.limit_motion_protection = 0;
  memset(eeprom, 0x55, sizeof(eeprom));
  eeprom[0] = 11;
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&v11, sizeof(v11));
  memset(&out, 0xAA, sizeof(out));
  assert(new_migration_v11(&out) == 1);
  assert(memcmp(&out, &v10, sizeof(v10)) == 0);
  assert(out.limit_motion_protection == 0); /* user's $28=0 kept */
  assert(out.limit_recovery_travel == DEFAULT_RECOVERY_TRAVEL);
  printf("[new migration v11] machine settings and $28 preserved, $29 -> 10\n");

  /* 5) Round-trip: writing the v12 record and reading it back works. */
  memset(eeprom, 0xFF, sizeof(eeprom));
  eeprom[0] = 12;
  out.limit_recovery_travel = 4.5f; /* a user-tuned $29 */
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&out, sizeof(out));
  settings_t reread;
  assert(memcpy_from_eeprom_with_checksum((char*)&reread, EEPROM_ADDR_GLOBAL, sizeof(reread)));
  assert(reread.limit_motion_protection == 0);
  assert(reread.limit_recovery_travel == 4.5f);
  printf("[v12 round-trip] record writes/reads with valid checksum\n");

  printf("\nALL MIGRATION TESTS PASSED\n");
  return 0;
}
