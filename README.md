# Triangle maquette

Arduino program for the Triangle kinetic sculpture maquette. Two stepper motors drive a linkage of drinking straws, through belts to the pivots, and take it through a choreographed sequence of poses.

## Hardware

- **Controller:** Arduino Micro.
- **Motors:** 2 × 17HS08-1004-ME1K Nema 17 steppers, each with a 1000-line magnetic encoder (4000 counts/rev plus an index pulse).
- **Drivers:** 2 × DM320T stepper drivers, with DIP switches set to 1.3 A peak and 8000 steps/rev.
- **Belts:** 2GT belts, with a 36T pulley on each motor and a 16T pulley on each pivot, so the pivot turns 2.25× the motor.
- **Power:** a 12–24 V supply for the drivers. The Micro runs from USB, or in the finished build from a 5 V buck converter.

The step count is checked against the encoders at the end of every stage. The sequence stops if they disagree by more than 5°, which would mean a jam or a fault.

## Documents (`src/docs`)

| File | What it is |
|---|---|
| `Triangle sequence.jpg` | The hand-drawn sequence sheet: the design intent for each pose |
| `wiring.pdf` | Prototype wiring (v5), with a connection checklist and DIP settings |
| `wiring-v6.pdf` | The same wiring on the carrier board's pins |
| `board.pdf` | Carrier board layout for an ElectroCookie half-size protoboard |
| Datasheets | Motor and encoder, DM320T manual, Arduino Micro pinout |

## Code

| File | What it does |
|---|---|
| `include/config.h` | Hardware settings, motion defaults, homing settings, and the pin map |
| `include/sequence.h` | **The choreography:** one row per stage, with angles, hold time, speed and acceleration. Edit this to tweak the sequence. |
| `src/main.cpp` | Runs the sequence, checks the encoders, and handles keyboard control |
| `src/homing.cpp` | Homing on the encoder index pulse, and its calibration |
| `src/hwtest/` | Wiring test sketch |

**Pin map:** set `CARRIER_BOARD` in `config.h` to match the wiring. Use `false` for the prototype wiring in `wiring.pdf`, and `true` for the carrier board, or for the prototype rewired from `wiring-v6.pdf`.

**Angles:** these are pivot degrees, measured from the start pose. For the left straw, forward is negative; for the right straw, forward is positive. Forward is clockwise when viewed from the left elevation.

## Building and uploading (PlatformIO)

| Environment | Use |
|---|---|
| `micro` | The sculpture program |
| `micro_hwtest` | Wiring test: move each motor by a small angle and check the encoders |
| `uno` | Legacy; the pin map is written for the Micro |

```
pio run -e micro -t upload
pio device monitor -e micro
```

## Running it

Once calibrated, the program homes on the index pulses at power-on, then moves to the start pose. After that it waits for `r`. Before calibration, 0° is wherever the straws are when the Micro starts.

Serial monitor keys:

| Key | Action |
|---|---|
| `r` | Restart: return to the start pose, then run the sequence |
| `x` | Stop both motors immediately |
| `c` | Continue after a stop |
| `p` | Print the encoder positions, in degrees |
| `h` | Home on the index pulses |
| `i` | Calibration step 1, with the straws off: find each motor's index pulse |
| `o` | Save the current pose as the start pose. Use it after `i`, or after homing, to move the start pose. |

## Calibrating the start pose

1. Take the straws off and press `i`, then `y`. Each motor turns until it finds its index pulse.
2. Switch the motor power off, but keep the USB connected so the encoders keep counting.
3. Fit the straws with each one about 5–10° forward of its start pose. Then turn each straw back to the exact start pose.
4. Press `o`. The offsets are saved in EEPROM, so they survive power-off and new uploads.

To move the start pose later: home, switch the motor power off, set the straws by hand, then press `o`.
