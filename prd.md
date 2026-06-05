# PRD: Path Finder Robot

## 1. Product Overview

The Path Finder Robot is a mobile robotics project for a small differential-drive robot equipped with five IR ground sensors. The robot must follow black lines on a white floor, explore a line-map environment, detect a final target position, and return to the starting position using the shortest path.

The robot is programmed in C/C++ for a PIC32 microcontroller and is tested on a physical robot platform developed at DETI.

## 2. Objectives

- Build an autonomous agent that follows a black line on a white floor.
- Handle intersections by choosing the left path during exploration.
- Explore the full path map until the target is found.
- Detect the final target position, marked by a wide black line.
- Return from the target to the start position using the shortest path.
- Stop accurately at the starting position after returning.
- Minimize the return time while keeping movement stable and reliable.

## 2.1 Implemented Agent

The implemented agent is `rm_deti_rob/src/rm-pathfinder.c`.

Current implementation version:

- `left-hand-v7-encoder-return`

The agent uses:

- the five IR ground sensors through `readLineSensors(0)`
- differential wheel commands through `setVel2(left, right)`
- encoder-based closed-loop motor control through `closedLoopControl(true)`
- encoder odometry through `setRobotPos(0, 0, 0)` and `getRobotPos(...)`

The encoder odometry is used during the return phase to infer when the robot has arrived near the original start pose after executing the optimized return decisions.

## 3. Users and Stakeholders

- Student developers implementing and testing the robot behavior.
- Course instructors evaluating the implementation, report, presentation, and demo.
- Demo observers watching the robot complete three test runs.

## 4. Scope

### In Scope

- Line-following behavior using the five IR ground sensors.
- Intersection detection.
- Left-hand exploration strategy.
- Target detection using the wide black line marker.
- Path recording during exploration.
- Shortest-path calculation or path optimization for the return phase.
- Return navigation to the start position.
- Robot stop behavior at the start position.
- Physical robot testing and calibration.

### Out of Scope

- Obstacle avoidance.
- Camera-based navigation.
- Wireless control.
- SLAM or map-building beyond the line graph needed for shortest-path return.
- Dynamic environments where the line map changes during execution.
- Handling maps with cycles, since the assignment states that the path pattern has no cycles.

## 5. Assumptions

- The environment consists of black lines on a white floor.
- The black-line graph has no cycles.
- The robot starts on the path at a known start position.
- The target is represented by a wide black line.
- The robot has access to five IR ground sensors and wheel encoders.
- Wheel speeds can be controlled independently.
- The robot can be programmed using the PIC32 toolchain.
- During exploration, choosing left at intersections is sufficient to explore the full map.

## 6. Functional Requirements

### FR1: Line Following

The robot shall continuously read the five IR ground sensors and adjust wheel speeds to stay centered on the black line.

Acceptance criteria:

- The robot follows straight and curved black lines without leaving the path.
- Small sensor noise does not cause unstable movement.
- The robot can recover from minor deviations from the line.

### FR2: Intersection Detection

The robot shall detect intersections using the ground sensor pattern.

Acceptance criteria:

- The robot identifies when multiple path options are available.
- The robot distinguishes a normal line from an intersection.
- The robot slows down or stabilizes before making intersection decisions if needed.

### FR3: Left-First Exploration

When an intersection provides multiple options, the robot shall choose the left option during the exploration phase.

Acceptance criteria:

- At each detected intersection, the robot turns left when a left branch exists.
- If no left branch exists, the robot continues straight when possible, turns right when right is the only available branch, or turns back at a dead end.
- Exploration decisions are recorded for later return-path optimization.

### FR4: Target Detection

The robot shall detect the final position marked by a wide black line.

Acceptance criteria:

- The robot identifies the target marker reliably.
- The robot does not confuse normal intersections with the wide black target line.
- Once the target is detected, the robot switches from exploration mode to return mode.

### FR5: Path Recording

The robot shall record the path taken during exploration.

Acceptance criteria:

- Each relevant movement decision is stored as `L`, `S`, `R`, or `B`.
- The recorded path is sufficient to compute the route back to the start.
- Dead-end or unnecessary exploration segments can be removed or ignored for the shortest return path.

### FR6: Shortest Return Path

The robot shall compute or derive the shortest path from the target back to the start.

Acceptance criteria:

- The return path avoids unnecessary branches explored earlier.
- The return route is shorter than simply reversing every exploration movement when dead ends were visited.
- The robot can execute the optimized route using stored navigation decisions.

### FR7: Return Navigation

The robot shall navigate from the target back to the start using the optimized return path.

Acceptance criteria:

- The robot follows the line during the return phase.
- At intersections, the robot selects the correct direction according to the optimized return path.
- The robot minimizes return time while maintaining reliable tracking.

### FR8: Stop at Start

The robot shall stop at the start position after completing the return.

Acceptance criteria:

- The robot detects or infers that it has reached the start.
- Both wheel speeds are set to zero.
- The robot remains stopped after completion.

## 7. Non-Functional Requirements

### Reliability

- The robot should complete three demo runs consistently.
- Sensor thresholds should be calibrated for the test floor and lighting conditions.

### Performance

- Line-following corrections should be fast enough to prevent path loss.
- Return phase should be optimized for time after the target is found.

### Maintainability

- Code should be organized into clear modules or functions:
  - sensor reading
  - line following
  - intersection detection
  - target detection
  - path recording
  - path optimization
  - movement control

### Safety

- Motor speeds should remain within safe limits.
- The robot should stop if the line is lost for a defined timeout.

## 8. Hardware and Software Constraints

- Robot platform: DETI differential-drive robot.
- Controller: PIC32 microcontroller.
- Programming language: C/C++.
- Sensors:
  - five IR ground sensors
  - wheel encoders
- Actuators:
  - left wheel motor
  - right wheel motor
- Toolchain:
  - PIC32 compiler tools installed under `/opt`
  - `pcompile`
  - `ldpic32`
  - `pterm`

## 9. Proposed Robot Behavior

The robot should operate as a finite-state machine.

### State 1: Idle

- Wait for the start button or start condition.
- Initialize sensors, thresholds, path memory, and motor control.

### State 2: Explore

- Follow the black line.
- Detect intersections.
- Prefer the left branch when it exists.
- If no left branch exists, choose straight, then right, then backtrack.
- Record movement decisions.
- Detect dead ends when the line is lost for a sustained interval and record `B`.
- Continue until the target marker is detected.

### State 3: Target Found

- Stop briefly or stabilize.
- Convert the exploration path into an optimized return path.
- Switch to return mode.

### State 4: Return

- Follow the black line back toward the start.
- Use the optimized path decisions at intersections.
- Increase speed if stable and safe.
- After all return decisions are executed, use encoder odometry to stop when the robot is within the configured start radius.

### State 5: Finished

- Stop at the start position.
- Keep motors off.

## 10. Path Strategy

During exploration, the robot records decisions such as:

- `L`: left turn
- `S`: straight
- `R`: right turn
- `B`: backtrack or dead end

After reaching the target, the recorded path should be simplified to remove unnecessary moves. Since the map has no cycles, dead-end exploration branches can be eliminated, allowing the robot to return using the shortest known route.

Implemented simplification:

- Convert moves to relative angles: `S = 0`, `R = 90`, `B = 180`, `L = 270`.
- Whenever a pattern `move, B, move` appears at the end of the path, replace it with the equivalent single relative turn.
- Reverse the optimized start-to-target route.
- Invert left/right moves to obtain the target-to-start return route.

Example:

- Raw exploration path: `L B R`
- Angle sum: `270 + 180 + 90 = 540`, equivalent to `180`
- Optimized move: `B`

## 11. Sensor Interpretation

The five IR sensors should be interpreted as a binary or thresholded pattern:

- black detected: sensor value above or below calibrated black threshold, depending on sensor output
- white detected: sensor value outside the black threshold

Possible patterns:

- center sensor detects black: robot is centered
- left sensors detect black: line is to the left, correct left
- right sensors detect black: line is to the right, correct right
- multiple sensors detect black widely: possible intersection or target
- all or most sensors detect black for a sustained distance: possible wide target marker
- no sensors detect black: line lost or gap

The implementation currently treats a non-normal, multi-branch sensor pattern as an intersection candidate. It confirms target detection by measuring how many 20 ms ticks the robot remains over a wide black reading while moving slowly forward. A sustained wide reading is treated as the target marker.

## 11.1 Tunable Constants

The following constants in `rm-pathfinder.c` should be tuned on the physical robot:

- `BASE_SPEED`: normal line-following speed.
- `TURN_GAIN`: line correction strength.
- `SEARCH_SPEED`: recovery speed when the line is temporarily lost.
- `TURN_SPEED`: in-place turning speed.
- `TARGET_WIDTH_MIN_TICKS`: minimum wide-line duration needed to confirm the target.
- `LOST_LINE_DEAD_END_TICKS`: line-loss duration before a dead end is recorded.
- `START_REACHED_RADIUS_MM`: odometry radius used to stop near the start on return.

## 12. Testing Plan

### Unit-Level Tests

- Validate sensor threshold calibration.
- Validate line-position calculation from the five sensors.
- Validate turn command functions.
- Validate path simplification logic with predefined path sequences.

### Integration Tests on Robot

1. Compile on Linux with the PIC32 toolchain:

   ```sh
   cd rm_deti_rob/src
   pcompile rm-pathfinder.c rm-mr32.c
   ```

2. Upload:

   ```sh
   ldpic32 -w rm-pathfinder.hex
   pterm
   ```

3. Press the robot start button and verify:

   - stable line following
   - left-first behavior at intersections
   - `L/S/R/B` moves printed in the terminal
   - target detection on the wide black line
   - optimized and return paths printed
   - return navigation stops near the start position using encoder odometry

4. Run the demo three times and tune the constants if the robot overshoots intersections, misclassifies the target, or stops too far from the start.

### Integration Tests

- Test line following on:
  - straight lines
  - curves
  - sharp turns
  - intersections
- Test target detection separately from intersections.
- Test switching from exploration mode to return mode.

### Demo Tests

- Run the robot on the final map three times.
- Record whether the robot:
  - follows the line correctly
  - chooses left at intersections
  - finds the target
  - returns through the shortest path
  - stops at the start

## 13. Success Metrics

- Robot completes all three demo runs.
- Robot finds the target without manual intervention.
- Robot returns to the start using an optimized path.
- Robot stops at the start position.
- Robot behavior is stable and repeatable.
- Report and presentation clearly explain the design, implementation, and results.

## 14. Deliverables

- Source code for the developed robotic agents.
- PDF report using the Springer LNCS paper template.
- Presentation.
- Live demo with three runs.

## 15. Risks and Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Sensor thresholds vary with lighting | Robot may lose the line | Add calibration and robust thresholds |
| Intersection confused with target | Wrong state transition | Require sustained wide-line detection for target |
| Robot overshoots intersections | Wrong turns | Reduce speed near intersections |
| Path memory too small | Cannot store route | Use compact turn encoding |
| Fast return causes instability | Robot leaves path | Tune return speed and use correction control |
| Wheel slip affects turning | Inaccurate turns | Use sensor feedback after turns, optionally encoder support |

## 16. Milestones

1. Set up PIC32 toolchain and verify example code.
2. Implement and calibrate sensor reading.
3. Implement basic line following. Completed.
4. Implement intersection detection and left-first decisions. Completed.
5. Implement target detection. Completed.
6. Implement path recording. Completed.
7. Implement path optimization. Completed.
8. Implement return navigation. Completed.
9. Implement start-position detection and final stopping behavior. Completed with encoder odometry.
10. Tune speeds and thresholds on the real robot.
11. Prepare report, presentation, and demo.

## 17. Current Project Status

- The robot follows the black line using `readLineSensors(0)` and differential speed correction.
- Exploration uses a left-hand rule: choose `L` when available, otherwise `S`, then `R`, then `B`.
- Dead ends are detected when the line is lost for `LOST_LINE_DEAD_END_TICKS`; the robot records `B` and turns around.
- Intersections are debounced before action to reduce duplicate decisions from the same physical junction.
- The target is confirmed by measuring sustained wide black readings with `TARGET_WIDTH_MIN_TICKS`.
- The raw exploration path is stored in a fixed-size buffer, optimized by simplifying dead-end patterns, then reversed and inverted for return.
- Return navigation follows the line and executes the optimized return decisions at detected intersections.
- After the return decisions are exhausted, encoder odometry is used to stop near the original start pose.

Remaining work is physical calibration: tune speeds, line-width timing, dead-end timing, and odometry stop radius on the real robot and record the three demo runs.
