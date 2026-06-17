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

Path Finder Robot - exploration left probe-target-v26-width-safe
Press start to run
Start pose reset x=0 y=0 theta=0mrad
Intersection detected
Line width ticks=3
Exploration move: L
Line width ticks=3
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Line width ticks=1
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Line width ticks=1
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Exploration move failed; retrying same move
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Dead end detected
Line width ticks=6
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Line width ticks=4
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=0
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=4
Exploration move: L
Line width ticks=7
Line width ticks=26
Target confirmed while clearing intersection sensors=11100 width=26
Target pose x=586 y=383 theta=4294966130mrad
Recorded path (14 moves): LLLLLSBBBBBBBL
Optimized path (6 moves): LLLLLR
Return path (6 moves): LRRRRR
Recorded move=L
Return navigation starting
Intersection cleared
Return candidate detected
Line width ticks=2
Return move: L
[WARN] Return move failed; trying bounded recovery
Return move recovery
[ERROR] Return move failed - move not consumed
Final pose x=155 y=591 theta=2588mrad
Press start to run
Start pose reset x=0 y=0 theta=0mrad
Intersection detected
Line width ticks=4
Exploration move: L
Line width ticks=5
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Stable line reacquired
Dead end ignored - line recovered
Intersection detected
Line width ticks=1
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Checking lost line before dead end
Dead end detected
Dead end turn searching for line
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=1
Exploration move: L
Line width ticks=3
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Dead end detected
Dead end turn searching for line
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Exploration move failed; retrying same move
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Stable line reacquired
Dead end ignored - line recovered
Intersection detected
Line width ticks=1
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Exploration move failed; retrying same move
Exploration move: L
Intersection cleared
Recorded move=L
Checking lost line before dead end
Dead end detected
Dead end turn searching for line
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Line width ticks=2
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Intersection cleared
Recorded move=L
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=12
Exploration move: L
Line width ticks=6
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Exploration move failed; retrying same move
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: L
Exploration move failed; retrying same move
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=3
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Dead end detected
Line width ticks=1
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Line width ticks=2
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Line width ticks=2
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Line width ticks=7
Line width ticks=1
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Dead end turn searching for line
Intersection cleared
Recorded move=B
Intersection detected
Line width ticks=2
Exploration move: L
Line width ticks=3
Line width ticks=1
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=2
Exploration move: L
Intersection cleared
Recorded move=L
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Intersection detected
Line width ticks=1
Exploration move: S
Intersection cleared
Recorded move=S
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Dead end detected
Intersection cleared
Recorded move=B
Checking lost line before dead end
Stable line reacquired
Dead end ignored - line recovered
Intersection detected
Line width ticks=5
Exploration move: L
Line width ticks=17
Target confirmed while clearing intersection sensors=11111 width=17
Target pose x=545 y=818 theta=4294965348mrad
Recorded path (48 moves): LLLSLLBLLSBLLLSLLLBLLLBLLLLLLSBBBBBBBBBBBBLLSSBB
Optimized path (26 moves): LLLSLSLRLLSLLSLSLLLLLSLLSS
Return path (26 moves): SSRRSRRRRRSRSRRSRRLRSRSRRR
Recorded move=L
Return navigation starting
Intersection cleared
Return candidate detected
Line width ticks=1
Return move: S
Return move completed
Intersection cleared
Return index=1/26
Return candidate detected
Line width ticks=2
Return move: S
Return move completed
Intersection cleared
Return index=2/26
Return candidate detected
Line width ticks=1
Return move: R
Return move completed
Intersection cleared
Return index=3/26
Return candidate detected
Line width ticks=2
Return move: R
Return move completed
Intersection clear timeout; holding junction lockout
Junction clear recovery searching for stable line
[ERROR] Junction lockout failed - stable line not reacquired
[WARN] Return line reacquisition failed; trying bounded recovery
Return line recovery
[ERROR] Return line reacquisition failed after move
Final pose x=616 y=1792 theta=1666mrad
















8. Tune `RETURN_START_LOST_LINE_TICKS` if the robot reaches the start line end but stops too early or too late.
9. Run and record the three required demo runs.









