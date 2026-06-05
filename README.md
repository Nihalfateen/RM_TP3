# RM TP3 - Path Finder Robot

This repository contains the Assignment 2 work for the Mobile Robotics Path Finder Robot.

## Project Files

- `prd.md`: product requirements document for the assignment.
- `rm_deti_rob/src/rm-pathfinder.c`: first Path Finder agent implementation.
- `rm_deti_rob/src/rm-example.c`: original example from the provided package.
- `rm_deti_rob/src/rm-mr32.c` and `rm_deti_rob/src/rm-mr32.h`: robot support library.

## Current Implementation

The current implementation in `rm-pathfinder.c` covers the main assignment behavior:

- reading the 5-bit ground sensor mask with `readLineSensors(0)`
- following the black line using differential wheel speeds
- searching when the line is temporarily lost
- detecting intersections with debounce
- exploring with a left-hand rule: `L`, then `S`, then `R`, then `B`
- recording and optimizing the exploration path
- detecting the target as a sustained wide black marker
- stopping early on sustained wide readings before the longer target scan can overshoot
- checking for the target while clearing intersections after turns
- returning on the optimized path
- using encoder odometry to stop near the starting pose

The robot uses `closedLoopControl(true)`, so wheel encoders are active for motor control and dead-reckoning pose estimation.

## Build on Linux

The provided PIC32 compiler archive contains Linux x86_64 binaries. It does not run directly on macOS ARM.

On a Linux machine or VM:

```sh
cd /opt
tar xvfz /path/to/pic32-64-2017_09_15.tgz
export PATH="$PATH:/opt/pic32mx/bin"
cd /path/to/RM_TP3/rm_deti_rob/src
pcompile rm-pathfinder.c rm-mr32.c
```

Expected output:

```text
rm-pathfinder.hex
```

## Upload to Robot

After connecting the robot by USB:

```sh
ldpic32 -w rm-pathfinder.hex
pterm
```

Then press the robot start button to run the agent and the stop button to stop it.

## Build on macOS

The compiler from the assignment archive is a Linux executable, so macOS cannot run it directly.

Options:

- use the university Linux environment
- use a Linux VM
- use Docker with an x86_64 Linux container, if Docker Desktop is running

## Robot Tuning Checklist

1. Test `rm-pathfinder.c` on the real robot.
2. Tune `BASE_SPEED`, `TURN_GAIN`, `SEARCH_SPEED`, and `TURN_SPEED`.
3. Confirm the bit order of the five ground sensors.
4. Tune `TARGET_WIDTH_MIN_TICKS` so intersections are not confused with the target.
5. Tune `TARGET_EARLY_CONFIRM_TICKS` if the robot still passes the target or stops on normal intersections. The current value is `8`.
6. Tune `LOST_LINE_DEAD_END_TICKS` so small gaps are not treated as dead ends.
7. Tune `START_REACHED_RADIUS_MM` for reliable final stopping with encoder odometry.
8. Run and record the three required demo runs.
