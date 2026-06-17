# RM TP3 - Path Finder Robot

This repository contains the Assignment 2 work for the Mobile Robotics Path Finder Robot.

## Goal

Develop an autonomous line-following robot that explores an unknown maze with a left-hand navigation strategy, detects intersections and dead ends, identifies the target area through line-width analysis, and returns to the starting position.

The robot records executed movements (`L`, `S`, `R`, and `B`) while exploring, uses those moves to build a topological representation of the path, optimizes the recorded route by removing unnecessary backtracking, computes the return path, and navigates back autonomously. Encoder odometry maintains an estimated pose so the robot can monitor its movement and improve return-to-start stopping behavior.

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
- probing target-like wide readings immediately before the robot can complete another loop
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
4. Tune `TARGET_WIDTH_MIN_TICKS` so intersections are not confused with the target. The current value is `18`.
5. Tune `TARGET_BACKUP_MAX_TICKS` if the robot confirms the target but stops slightly after it. The current value is `0`.
6. Tune `LOST_LINE_DEAD_END_TICKS` so small gaps are not treated as dead ends.

7. Tune `START_REACHED_RADIUS_MM` for reliable final stopping with encoder odometry.


#DP32BL0316$
DETPIC32, Bootloader V0.3
Universidade de Aveiro, DETI
J.L.Azevedo, 2011

Path Finder Robot - exploration left probe-target-v24-junction-lockout
Press start to run
Start pose reset x=0 y=0 theta=0mrad
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=1
Intersection rejected width=1
Intersection detected
Line width ticks=1
Intersection rejected width=1
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Intersection detected
Line width ticks=1
Intersection rejected width=1
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Intersection detected
Line width ticks=1
Intersection rejected width=1
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=1
Intersection rejected width=1
Intersection detected
Line width ticks=2
Intersection rejected width=2
Intersection detected
Line width ticks=1
Intersection rejected width=1
Intersection detected
Line width ticks=4
Exploration move: L
Recorded move=L
Intersection cleared
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Intersection detected
Line width ticks=4
Exploration move: L
Recorded move=L
Intersection cleared
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection clear timeout; holding junction lockout
[ERROR] Junction lockout failed - stable line not reacquired
Press start to run
Start pose reset x=0 y=0 theta=0mrad
Intersection detected
Line width ticks=5
Exploration move: L
Recorded move=L
Intersection cleared
Intersection detected
Line width ticks=3
Exploration move: L
Recorded move=L
Intersection cleared
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=1
Intersection rejected width=1
Intersection detected
Line width ticks=1
Intersection rejected width=1
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=2
Intersection rejected width=2
Intersection detected
Line width ticks=6
Exploration move: L
Recorded move=L
Intersection cleared
Dead end detected
Intersection cleared
Intersection detected
Line width ticks=2
Intersection rejected width=2
Intersection detected
Line width ticks=30
Target confirmed at intersection sensors=11111 width=30
Target pose x=714 y=311 theta=21mrad
Recorded path (6 moves): LLBBLB
Optimized path (4 moves): LLLB
Return path (4 moves): BRRR
Target reached; press start for return or stop to reset









8. Tune `RETURN_START_LOST_LINE_TICKS` if the robot reaches the start line end but stops too early or too late.
9. Run and record the three required demo runs.









