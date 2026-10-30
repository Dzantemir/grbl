![GitHub Logo](https://github.com/gnea/gnea-Media/blob/master/Grbl%20Logo/Grbl%20Logo%20250px.png?raw=true)

# Grbl 1.1h — fork with limit switch protection (`$28`/`$29`) and in-firmware limit escape (`$T`)

[Русская версия](README.ru.md)

A fork of [gnea/grbl](https://github.com/gnea/grbl) v1.1h for the Arduino 328p
(Uno/Nano) that fixes the weakest spot of stock GRBL: **what happens when a
limit switch is already pressed**. Stock GRBL leaves you with a machine that
either refuses to move or lets the host jog it *further into* the switch.
This fork locks the machine down and then drives off the switch **entirely
inside the firmware** — no host jogging involved.

Build tag: **`20261030_ALOE_FIXED11`** (visible in the `$I` response).

## What's different from stock

### `$29` — `$T` escape budget, mm (new setting, default `10`)

How far the `$T` escape may creep while searching for the switch release
(formerly the compile-time `LIMIT_RECOVERY_MAX_TRAVEL`). Tune it to your
switches: it should exceed the worst-case overtravel but stay within the
mechanical margin. `$29=0` disables the `$T` escape entirely (`error:15`);
negative values are rejected. Existing EEPROM settings migrate automatically
(`$29` arrives as `10.0`).

### `$28` — limit motion protection (new setting, default `1`)

While **any** limit input is active, all motion is locked on **all axes in
both directions**:

* jog commands return `error:15`;
* a regular g-code motion attempt triggers a controlled stop (feed hold
  first when a program is streaming) followed by `ALARM:1` — lines are never
  silently acked without motion;
* homing `$H` is refused (it would slam a pressed switch with
  `LIMITS_TWO_SWITCHES_ON_AXES`).

The lock is **stateless** — a pure function of the pins, with no flags or
latched state. The moment every switch is released, the machine is unlocked.
The firmware never guesses which end of an axis is pressed (with two switches
per input it physically cannot know).

`$28=0` returns the firmware to fully stock behavior (`$T` then answers
`error:5`).

Note: this fork's `config.h` enables the stock `LIMITS_TWO_SWITCHES_ON_AXES`
option (both ends of an axis share one input). With `$28=0` the firmware
behaves exactly like vanilla GRBL *compiled with that same option* — e.g.
`$H` with a pressed switch still trips the stock `ALARM:1`.

**`$28` is not a substitute for `$21`.** The lock gates *new* motion at
planning time; motion already in the planner buffer keeps executing and is
stopped only by the hard-limit interrupt. Run the machine with `$21=1`;
`$28` adds the pressed-switch lockout and the `$T` recovery on top of it.

### `$T<axis><dir>` — automatic limit escape (new command)

The operator declares which axis is pressed and which way is free — the only
thing the firmware cannot know — and the firmware does the rest by itself as
a homing-style system motion (planner-direct, `STATE_HOMING`, limit
interrupts disabled for the duration, exactly like `$H`):

```
$TX-   "X is pressed, escape toward negative"
$TX+ / $TY- / $TY+ / $TZ- / $TZ+
```

1. **Creep** in the declared direction at the homing feed rate `$24`,
   polling the axis input every cycle, up to the `$29` budget
   (default 10 mm).
2. **Stop + debounce**: instant stepper stop on release, `$26` ms settle
   delay, then the input is re-read — a bounce-faked "release" fails the
   command honestly instead of blindly pulling off.
3. **Pull-off** a further `$27` mm at the homing seek rate `$25`.
4. **Verdict**: input free → `ok`; budget exhausted or still pressed (wrong
   declared direction, wiring, jammed switch) → `error:15` +
   `[MSG:Check Limits]`.

The `$29` budget caps the **search phase**: on failure (including a wrongly
declared direction) the axis stops within `$29` mm — it cannot run away. On
success the total travel is the release distance **plus `$27`**; the
pull-off only runs after the debounced re-check has confirmed the switch is
released, so it always moves away from a free switch.

Accepted from **Idle and Alarm** (like `$H`). Reset / safety door during the
escape abort it exactly like homing (`ALARM:6` / `ALARM:7`). The status
report shows `Home` while the escape runs.

### Why in-firmware escape matters

The incident that triggered this fork: a host-side jog bug (Candle computes
continuous-jog distances from a cached `$20` soft-limit state; at the border
the distance goes negative and an **X+ button press is sent as `X-1.005`**).
Any recovery scheme that relies on host jogging inherits every host bug.
`$T` performs the whole move itself, so a sign-flipped host command simply
has nowhere to land: while the switch is pressed the machine is locked, and
during the escape the host is not involved at all.

### Hard-limit ISR hardening

`HARD_LIMIT_FORCE_STATE_CHECK` is enabled: on a limit pin edge the ISR
re-reads the actual pin state and raises `ALARM:1` only if a switch is
really active — no false alarms on the *release* edge of a switch.
(`ENABLE_SOFTWARE_DEBOUNCE` is unchanged from stock and disabled.)

## Recovery workflow

Switch pressed, **no alarm** (noticed in Idle; jog answers `error:15`):

```
$TX+        escape the pressed X switch toward positive
$H          re-home: position after any limit event is untrusted
```

**Hard-limit trip (`ALARM:1`)**: stock GRBL enters its critical lock and
ignores line commands until a realtime reset — `$X`/`$T` sent before the
reset are discarded (upstream behavior):

```
Ctrl-X      realtime reset (0x18) — the only exit from the critical lock
$TX+        after reset the firmware boots into Alarm (homing enabled,
            $22=1); $T is accepted straight from Alarm, $X not required
$H
```

## Reference

| Command / situation | Response |
|---|---|
| `$TX+` … `$TZ-`, escape succeeds | `ok` |
| budget used up / input still active | `error:15` + `[MSG:Check Limits]` (repeatable) |
| bare `$T`, unknown axis, missing sign | `error:3` |
| `$T...` while `$28=0` | `error:5` |
| declared axis is not pressed | `[MSG:Disabled]` + `ok`, no motion (`$TX` cannot unlock a machine held by Y) |
| `$T...` while running / jogging | `error:8` |
| safety door ajar | `error:13` |

Settings used by the escape (all standard, nothing hardcoded):

| Setting | Role in `$T` |
|---|---|
| `$24` homing feed | creep speed of the release search |
| `$25` homing seek | pull-off speed |
| `$26` debounce, ms | settle delay before the release is trusted |
| `$27` pull-off, mm | distance driven off the released switch |
| `$29` budget, mm | search budget (default 10, `$29=0` disables `$T`) |

Keep the `$29` budget within the mechanical overtravel of your switches.

## Companion Candle plugin

[candle-plugin-limit-monitor](https://github.com/Dzantemir/candle-plugin-limit-monitor) —
a Candle panel that shows the live `Pn:` pin state per axis, warns when a
switch is pressed and offers one-click `$TX±` escape buttons (enabled in
Idle/Alarm only; harmless `error:2` on stock GRBL).

## Building

Standard GRBL build: `make` with avr-gcc for `atmega328p` @ 16 MHz, or flash
the prebuilt hex from the releases.

**Flash usage depends on the compiler.** Reference numbers for FIXED11:
avr-gcc 7.x ≈ **31.1 KB**, avr-gcc 14.x ≈ **32 128 bytes** — both fit the
32 256-byte application area of an Uno/Nano with the Optiboot (0.5 KB)
bootloader. **Nano boards with the old 2 KB bootloader (30 720-byte limit)
cannot fit this fork** — burn Optiboot ("Uno" bootloader) first or use an
Uno. The image is nearly full either way: extra `config.h` options (e.g.
`ENABLE_SOFTWARE_DEBOUNCE`, +~70 bytes) may not fit without trimming
something else — always check the `avr-size` output against 32 256.

Keep `$24`, `$25` and `$27` **greater than zero**: stock GRBL accepts `0`
for these settings, but both stock homing and the `$T` escape need a real
feed rate and a real pull-off distance ( `$27=0` silently skips the pull-off
and leaves the carriage on the very edge of the switch).

The protection/escape logic is additionally covered by host-side unit tests
(`tests/`, plain gcc, no hardware needed).

## Upstream, license, credits

Everything else is unmodified GRBL v1.1h: a no-compromise, high performance,
low cost motion controller for CNC milling, written in highly optimized C,
up to 30 kHz of stable, jitter-free step pulses.

* Docs: the official **[GRBL wiki](https://github.com/gnea/grbl/wiki)**
* Upstream: [gnea/grbl](https://github.com/gnea/grbl), lead developer
  Sungeun "Sonny" Jeon, Ph.D. (@chamnit), built on Grbl v0.6 by
  Simen Svale Skogsrud
* License: **GPLv3**, same as upstream — see [COPYING](COPYING)
