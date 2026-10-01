# Triangle maquette

Arduino program for the Triangle kinetic sculpture maquette. Two stepper motors drive a linkage of drinking straws, through belts to the pivots, and take it through a choreographed sequence of poses.

## Hardware

- **Controller:** Arduino Micro.
- **Motors:** 2 × 17HS08-1004-ME1K Nema 17 steppers, each with a 1000-line magnetic encoder (4000 counts/rev plus an index pulse).
- **Drivers:** 2 × DM320T stepper drivers, with DIP switches set to 1.3 A peak and 8000 steps/rev.
- **Belts:** 2GT belts, 194 mm, with 16T pulleys at both ends (1:1).
- **Power:** a 12–24 V supply for the drivers. The Micro runs from USB, or in the finished build from a 5 V buck converter.

The step count is checked against the encoders at the end of every stage. The sequence stops if they disagree by more than 5°, which would mean a jam or a fault.

## Documents (`src/docs`)

| File | What it is |
|---|---|
| `Triangle sequence.jpg` | The hand-drawn sequence sheet: the design intent for each pose |
| `wiring.pdf` | Prototype wiring (v5), with a connection checklist and DIP settings |
| `wiring-v6.pdf` | The same wiring on the carrier board's pins |
| `wiring-v7.pdf` | v6 plus Q1, which holds the drivers off until the program starts |
| `board.pdf` | Carrier board layout for an ElectroCookie half-size protoboard |
| Datasheets | Motor and encoder, DM320T manual, Arduino Micro pinout |

## Code

| File | What it does |
|---|---|
| `include/config.h` | Hardware settings, motion defaults, and the pin map |
| `include/sequence.h` | **The choreography:** one row per stage, with angles, hold time, speed and acceleration. Edit this to tweak the sequence. |
| `src/main.cpp` | Runs the sequence, checks the encoders, and handles keyboard control |
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

The program assumes the straws are in the start pose when it starts. Every run of the sequence ends back there, so that's where the last run left them.

At power-on the drivers are held off by Q1, a 2N2222 on their ENA inputs, so the motors are free and don't jerk. The encoders count from this moment, so the current pose is 0°. Pressing `r` switches the drivers on. Each motor jerks by up to a few degrees as its driver takes hold. The encoders measure the jerk, and the straws move smoothly back to 0° before the sequence starts.

Serial monitor keys:

| Key | Action |
|---|---|
| `r` | Start: switch the motors on, return to the start pose, then run the sequence |
| `x` | Stop both motors immediately. They stay on and hold their position. |
| `c` | Continue after a stop |
| `p` | Print the encoder positions, in degrees |
| `f` | Free the motors (drivers off), so the straws can be turned by hand |
| `h` | Make the current pose the start pose |

## Setting the start pose

1. Stop the sequence (`x`) if it's running.
2. Press `f`, then set the straws to the start pose by hand.
3. Press `h`.

The start pose isn't saved. If the power is cut mid-sequence, put the straws back in the start pose by hand before the next start.

**Without Q1 fitted** (ENA unconnected), the drivers are always on and `f` has no effect. Plug in the USB before the motor power, so the encoders are already counting when the jerk happens.
