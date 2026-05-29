# RM TP3 - Path Finder Robot

This repository contains the Assignment 2 work for the Mobile Robotics Path Finder Robot.

## Project Files

- `prd.md`: product requirements document for the assignment.
- `rm_deti_rob/src/rm-pathfinder.c`: first Path Finder agent implementation.
- `rm_deti_rob/src/rm-example.c`: original example from the provided package.
- `rm_deti_rob/src/rm-mr32.c` and `rm_deti_rob/src/rm-mr32.h`: robot support library.

## Current Starting Point

The first implementation focuses on:

- reading the 5-bit ground sensor mask with `readLineSensors(0)`
- following the black line using differential wheel speeds
- searching when the line is temporarily lost

This is the correct first milestone because intersections, target detection, and shortest-path return all depend on stable line following.

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

## Next Development Steps

1. Test `rm-pathfinder.c` on the real robot.
2. Tune `BASE_SPEED`, `TURN_GAIN`, and `SEARCH_SPEED`.
3. Confirm the bit order of the five ground sensors.
4. Add left-path choice at intersections.
5. Add target detection for the wide black line.
6. Add path recording.
7. Add path simplification and return mode.
