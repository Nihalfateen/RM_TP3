#include "rm-mr32.h"

#define BASE_SPEED 30
#define TURN_GAIN 6
#define SEARCH_SPEED 20
#define TURN_SPEED 25
#define INTERSECTION_SPEED 20

#define INTERSECTION_APPROACH_TICKS 8
#define SENSOR_TO_AXLE_ALIGN_TICKS 5
#define LEFT_TURN_MIN_TICKS 12
#define LEFT_TURN_MAX_TICKS 100
#define RIGHT_TURN_MIN_TICKS 12
#define RIGHT_TURN_MAX_TICKS 100
#define TURN_AROUND_MIN_TICKS 20
#define TURN_AROUND_MAX_TICKS 100
#define TURN_AROUND_EXTENDED_MAX_TICKS 160
#define TURN_CENTER_LOST_TICKS 2
#define TARGET_MIN_BLACK_SENSORS 3
#define LINE_WIDTH_SCAN_MAX_TICKS 30
/* Width-only target confirmation must stay above ordinary intersections.
   With slow active probing, normal intersections can reach 10 ticks with
   fullMask=0, so width alone needs a larger margin. */
#define TARGET_WIDTH_MIN_TICKS 18
/* Confirmed from physical test: target gives sensors=11111 for 30 ticks.
   Normal intersections never sustain all-5-sensors. 5 ticks is safe. */
#define TARGET_FULL_MASK_MIN_TICKS 5
#define TARGET_BACKUP_MAX_TICKS 20
#define LINE_WIDTH_SCAN_SPEED 12
#define INTERSECTION_CLEAR_TICKS 8
#define INTERSECTION_CLEAR_MAX_TICKS 100
#define INTERSECTION_DEBOUNCE_SAMPLES 3
#define INTERSECTION_DEBOUNCE_MIN_HITS 2
#define JUNCTION_REARM_COOLDOWN_TICKS 10
#define BRANCH_PROBE_MAX_TICKS 70
#define BRANCH_PROBE_SETTLE_TICKS 8
#define BRANCH_PROBE_MIN_LINE_TICKS 30
#define BRANCH_PROBE_MIN_NEXT_JUNCTION_TICKS 14
#define BRANCH_PROBE_LOST_TICKS 4
#define BRANCH_PROBE_BACKUP_EXTRA_TICKS 2
#define RETURN_INTERSECTION_DETECT_TICKS 3
#define LOST_LINE_SEARCH_TICKS 4
#define LOST_LINE_DEAD_END_TICKS 16
#define DEAD_END_REACQUIRE_MAX_TICKS 35
#define DEAD_END_REACQUIRE_HITS 2
#define TARGET_RETURN_PAUSE_TICKS 150
/* FIX BUG-05: reduced from 850 to 300 mm. 850 caused the robot to stop
   ~60 cm before the real origin. 300 mm gives encoder-drift tolerance
   without stopping a full tile early. */
#define START_REACHED_RADIUS_MM 300
#define RETURN_START_MIN_TICKS 20
#define RETURN_START_LOST_LINE_TICKS 10
#define RETURN_START_SEARCH_MAX_TICKS 600
#define CODE_VERSION "probe-target-v44-snapshot-decision"

#define MIN_SPEED -100
#define MAX_SPEED 100

#define SENSOR_LEFT_2 0x10
#define SENSOR_LEFT_1 0x08
#define SENSOR_CENTER 0x04
#define SENSOR_RIGHT_1 0x02
#define SENSOR_RIGHT_2 0x01

#define SENSOR_MASK 0x1F
#define LEFT_BRANCH (SENSOR_LEFT_2 | SENSOR_LEFT_1)
#define CENTER_BRANCH SENSOR_CENTER
#define RIGHT_BRANCH (SENSOR_RIGHT_1 | SENSOR_RIGHT_2)
#define LED_EXPLORING 0x01
#define LED_INTERSECTION 0x03
#define LED_TARGET_FOUND 0x0F

#define PATH_MAX_MOVES 128

static char pathMemory[PATH_MAX_MOVES + 1];
static unsigned int pathLength = 0;
static int pathOverflow = 0;

static char optimizedPath[PATH_MAX_MOVES + 1];
static unsigned int optimizedPathLength = 0;
static int optimizedPathOverflow = 0;

static char returnPath[PATH_MAX_MOVES + 1];
static unsigned int returnPathLength = 0;
static int returnPathOverflow = 0;

/* FIX BUG-TARGET: tracks how many consecutive ticks all 5 sensors were
   active during the last measureLineWidthTicks() call. A rectangular
   block target saturates all sensors even when the forward path is short,
   so widthTicks alone (which stops counting when the center sensor clears)
   misses it. This counter lets main() apply a second detection criterion. */
static int lastFullMaskTicks = 0;
static int lastProbeEndedAtIntersection = 0;
static int afterDeadEndBacktrack = 0;
static char lastDeadEndEntryMove = '\0';

typedef enum
{
    MODE_IDLE,
    MODE_EXPLORE,
    MODE_TARGET_FOUND,
    MODE_RETURN,
    MODE_FINISHED
} RobotMode;

static void resetPath(void)
{
    pathLength = 0;
    pathOverflow = 0;
    pathMemory[0] = '\0';
}

static void resetOptimizedPath(void)
{
    optimizedPathLength = 0;
    optimizedPathOverflow = 0;
    optimizedPath[0] = '\0';
}

static void resetReturnPath(void)
{
    returnPathLength = 0;
    returnPathOverflow = 0;
    returnPath[0] = '\0';
}

static void recordMove(char move)
{
    if (pathLength >= PATH_MAX_MOVES)
    {
        pathOverflow = 1;
        return;
    }

    pathMemory[pathLength] = move;
    pathLength++;
    pathMemory[pathLength] = '\0';
}

static char directDeadEndEntryMove(void)
{
    char move;

    if (pathLength == 0)
        return '\0';

    move = pathMemory[pathLength - 1];
    if (move == 'L' || move == 'S' || move == 'R')
        return move;

    return '\0';
}

static int moveToDegrees(char move)
{
    if (move == 'L')
        return -90;
    if (move == 'R')
        return 90;
    if (move == 'S')
        return 0;
    if (move == 'B')
        return 180;

    return 0;
}

static char degreesToMove(int degrees)
{
    while (degrees < 0)
        degrees += 360;
    degrees %= 360;

    if (degrees == 0)
        return 'S';
    if (degrees == 90)
        return 'R';
    if (degrees == 180)
        return 'B';
    if (degrees == 270)
        return 'L';

    return '\0';
}

static void simplifyOptimizedPathEnd(void)
{
    int totalDegrees;
    char replacement;

    if (optimizedPathLength < 3)
        return;

    if (optimizedPath[optimizedPathLength - 2] != 'B')
        return;

    totalDegrees = moveToDegrees(optimizedPath[optimizedPathLength - 3]) + moveToDegrees(optimizedPath[optimizedPathLength - 2]) + moveToDegrees(optimizedPath[optimizedPathLength - 1]);

    replacement = degreesToMove(totalDegrees);
    if (replacement == '\0')
        return;

    optimizedPathLength -= 3;
    optimizedPath[optimizedPathLength] = replacement;
    optimizedPathLength++;
    optimizedPath[optimizedPathLength] = '\0';
}

static void appendOptimizedMove(char move)
{
    if (optimizedPathLength >= PATH_MAX_MOVES)
    {
        optimizedPathOverflow = 1;
        return;
    }

    optimizedPath[optimizedPathLength] = move;
    optimizedPathLength++;
    optimizedPath[optimizedPathLength] = '\0';
    simplifyOptimizedPathEnd();
}

static void optimizePath(void)
{
    unsigned int i;

    resetOptimizedPath();

    for (i = 0; i < pathLength; i++)
        appendOptimizedMove(pathMemory[i]);
}

static char invertMove(char move)
{
    if (move == 'L')
        return 'R';
    if (move == 'R')
        return 'L';
    if (move == 'S')
        return 'S';
    if (move == 'B')
        return 'B';

    return '\0';
}

static void buildReturnPath(void)
{
    int i;
    char move;

    resetReturnPath();

    for (i = (int)optimizedPathLength - 1; i >= 0; i--)
    {
        move = invertMove(optimizedPath[i]);
        if (move == '\0')
            continue;

        if (returnPathLength >= PATH_MAX_MOVES)
        {
            returnPathOverflow = 1;
            break;
        }

        returnPath[returnPathLength] = move;
        returnPathLength++;
        returnPath[returnPathLength] = '\0';
    }
}

static void printPath(void)
{
    unsigned int i;

    printf("Recorded path (");
    printInt(pathLength, 10);
    printf(" moves): ");

    for (i = 0; i < pathLength; i++)
        printf("%c", pathMemory[i]);

    if (pathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static void printOptimizedPath(void)
{
    unsigned int i;

    printf("Optimized path (");
    printInt(optimizedPathLength, 10);
    printf(" moves): ");

    for (i = 0; i < optimizedPathLength; i++)
        printf("%c", optimizedPath[i]);

    if (optimizedPathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static void printReturnPath(void)
{
    unsigned int i;

    printf("Return path (");
    printInt(returnPathLength, 10);
    printf(" moves): ");

    for (i = 0; i < returnPathLength; i++)
        printf("%c", returnPath[i]);

    if (returnPathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static int clamp(int value, int minValue, int maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

static int lineError(unsigned int sensors)
{
    int weightedSum = 0;
    int activeCount = 0;

    sensors &= SENSOR_MASK;

    if (sensors & SENSOR_LEFT_2)
    {
        weightedSum -= 4;
        activeCount++;
    }
    if (sensors & SENSOR_LEFT_1)
    {
        weightedSum -= 2;
        activeCount++;
    }
    if (sensors & SENSOR_CENTER)
    {
        activeCount++;
    }
    if (sensors & SENSOR_RIGHT_1)
    {
        weightedSum += 2;
        activeCount++;
    }
    if (sensors & SENSOR_RIGHT_2)
    {
        weightedSum += 4;
        activeCount++;
    }

    if (activeCount == 0)
        return 0;

    return weightedSum / activeCount;
}

static void followLine(unsigned int sensors, int *lastDirection)
{
    int error;
    int correction;
    int leftSpeed;
    int rightSpeed;

    sensors &= SENSOR_MASK;

    if (sensors == 0)
    {
        if (*lastDirection < 0)
            setVel2(-SEARCH_SPEED, SEARCH_SPEED);
        else
            setVel2(SEARCH_SPEED, -SEARCH_SPEED);
        return;
    }

    error = lineError(sensors);
    if (error < 0)
        *lastDirection = -1;
    else if (error > 0)
        *lastDirection = 1;

    correction = error * TURN_GAIN;
    leftSpeed = clamp(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
    rightSpeed = clamp(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);

    setVel2(leftSpeed, rightSpeed);
}

static int hasLeftPath(unsigned int sensors)
{
    sensors &= SENSOR_MASK;
    return (sensors & LEFT_BRANCH) != 0;
}

static int hasForwardPath(unsigned int sensors)
{
    sensors &= SENSOR_MASK;
    return (sensors & CENTER_BRANCH) != 0;
}

static int hasRightPath(unsigned int sensors)
{
    sensors &= SENSOR_MASK;
    return (sensors & RIGHT_BRANCH) != 0;
}

static int isNormalLine(unsigned int sensors);
static int isIntersectionCandidate(unsigned int sensors);
static int waitUntilNormalLine(int *lastDirection);

static int isReturnIntersectionCandidate(unsigned int sensors)
{
    sensors &= SENSOR_MASK;
    return isIntersectionCandidate(sensors);
}

static int activeSensorCount(unsigned int sensors)
{
    int count = 0;

    sensors &= SENSOR_MASK;

    if (sensors & SENSOR_LEFT_2)
        count++;
    if (sensors & SENSOR_LEFT_1)
        count++;
    if (sensors & SENSOR_CENTER)
        count++;
    if (sensors & SENSOR_RIGHT_1)
        count++;
    if (sensors & SENSOR_RIGHT_2)
        count++;

    return count;
}

static int isNormalLine(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    if (sensors == SENSOR_CENTER)
        return 1;
    if (sensors == (SENSOR_LEFT_1 | SENSOR_CENTER))
        return 1;
    if (sensors == (SENSOR_CENTER | SENSOR_RIGHT_1))
        return 1;
    if (sensors == (SENSOR_LEFT_1 | SENSOR_CENTER | SENSOR_RIGHT_1))
        return 1;

    return 0;
}

static int isLineWidthReading(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    /* FIX BUG-03 (from v16): removed mandatory hasForwardPath() requirement.
       The target wide line or block can appear at a dead-end where only
       lateral sensors fire. Requiring activeSensorCount >= 3 and at least
       one active path sensor is sufficient. */
    return activeSensorCount(sensors) >= TARGET_MIN_BLACK_SENSORS && (hasLeftPath(sensors) || hasRightPath(sensors) || hasForwardPath(sensors));
}

static int isIntersectionCandidate(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    if (isNormalLine(sensors))
        return 0;

    if (activeSensorCount(sensors) < TARGET_MIN_BLACK_SENSORS)
        return 0;

    if (hasLeftPath(sensors) || hasRightPath(sensors) || hasForwardPath(sensors))
        return 1;

    return 0;
}

static char chooseExplorationMoveFromSensors(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    if (hasLeftPath(sensors))
        return 'L';
    if (hasForwardPath(sensors))
        return 'S';
    if (hasRightPath(sensors))
        return 'R';

    return 'B';
}

/* measureLineWidthTicks() — forward scan with full-mask counter.
 *
 * Physical test confirmed:
 *   - Target block  → wide reading or sustained 11111 full mask
 *   - Normal junctions with slow probing can reach widthTicks ~10, but
 *     without sustained fullMask
 *
 * Strategy: drive forward slowly and count two things simultaneously:
 *   widthTicks    = total ticks isLineWidthReading() stays true
 *   fullMaskTicks = ticks where all 5 sensors fired (11111)
 *
 * Stop the scan as soon as either target threshold is reached. The target is
 * a transverse block, so continuing to measure the full width can push the
 * robot past the marker before it enters MODE_TARGET_FOUND.
 *
 * Width-only confirmation is deliberately stricter than the full-mask path.
 */
static int measureLineWidthTicks(unsigned int firstSensors)
{
    int widthTicks = 0;
    int fullMaskRun = 0;
    unsigned int sensors = firstSensors & SENSOR_MASK;

    lastFullMaskTicks = 0;

    setVel2(LINE_WIDTH_SCAN_SPEED, LINE_WIDTH_SCAN_SPEED);
    while (!stopButton() && widthTicks < LINE_WIDTH_SCAN_MAX_TICKS && isLineWidthReading(sensors))
    {
        widthTicks++;
        if ((sensors & SENSOR_MASK) == SENSOR_MASK)
        {
            fullMaskRun++;
            if (fullMaskRun > lastFullMaskTicks)
                lastFullMaskTicks = fullMaskRun;
        }
        else
        {
            fullMaskRun = 0;
        }

        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;

        if (widthTicks >= TARGET_WIDTH_MIN_TICKS || lastFullMaskTicks >= TARGET_FULL_MASK_MIN_TICKS)
            break;
    }
    setVel2(0, 0);

    printf("Line width ticks=");
    printInt(widthTicks, 10);
    printf(" fullMask=");
    printInt(lastFullMaskTicks, 10);
    printf("\n");

    return widthTicks;
}

static void printCurrentPose(char *label);
static void printSignedInt(int value);
static void driveBackwardTicks(int ticks);

static void confirmTarget(unsigned int sensors, int widthTicks, char *source)
{
    setVel2(0, 0);
    leds(LED_TARGET_FOUND);

    printf("Target confirmed ");
    printf("%s", source);
    printf(" sensors=");
    printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
    printf(" width=");
    printInt(widthTicks, 10);
    printf(" fullMask=");
    printInt(lastFullMaskTicks, 10);
    printf("\n");
    printCurrentPose("Target pose");
    printPath();
    optimizePath();
    printOptimizedPath();
    buildReturnPath();
    printReturnPath();
}

/* FIX BUG-01: added setVel2(0, 0) at the end. Without it motors kept
   running after the timed drive, causing the robot to overshoot
   intersections and miss subsequent detection windows. */
static void driveForwardTicks(int ticks)
{
    int i;

    setVel2(INTERSECTION_SPEED, INTERSECTION_SPEED);
    for (i = 0; i < ticks && !stopButton(); i++)
        waitTick20ms();
    setVel2(0, 0);
}

static void driveBackwardTicks(int ticks)
{
    int i;

    setVel2(-LINE_WIDTH_SCAN_SPEED, -LINE_WIDTH_SCAN_SPEED);
    for (i = 0; i < ticks && !stopButton(); i++)
        waitTick20ms();
    setVel2(0, 0);
}

static int backupUntilLineSeen(int maxTicks)
{
    int ticks = 0;
    int hitTicks = 0;
    unsigned int sensors;

    setVel2(-LINE_WIDTH_SCAN_SPEED, -LINE_WIDTH_SCAN_SPEED);
    while (!stopButton() && ticks < maxTicks)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (sensors != 0)
        {
            hitTicks++;
            if (hitTicks >= DEAD_END_REACQUIRE_HITS)
            {
                setVel2(0, 0);
                printf("Dead end line reacquired backup=");
                printInt(ticks, 10);
                printf("\n");
                return 1;
            }
        }
        else
        {
            hitTicks = 0;
        }
    }

    setVel2(0, 0);
    printf("Dead end line reacquire failed\n");
    return 0;
}

static void printCurrentPose(char *label)
{
    double x;
    double y;
    double theta;

    getRobotPos(&x, &y, &theta);
    printf("%s", label);
    printf(" x=");
    printSignedInt((int)x);
    printf(" y=");
    printSignedInt((int)y);
    printf(" theta=");
    printSignedInt((int)(theta * 1000.0));
    printf("mrad\n");
}

static void printSignedInt(int value)
{
    if (value < 0)
    {
        printf("-");
        printInt((unsigned int)(-value), 10);
    }
    else
    {
        printInt((unsigned int)value, 10);
    }
}

static int isNearStartPose(void)
{
    double x;
    double y;
    double theta;
    double distanceSquared;
    double radiusSquared;

    getRobotPos(&x, &y, &theta);
    distanceSquared = (x * x) + (y * y);
    radiusSquared = (double)START_REACHED_RADIUS_MM * (double)START_REACHED_RADIUS_MM;

    return distanceSquared <= radiusSquared;
}

static int followLineUntilStartPose(int *lastDirection)
{
    int ticks = 0;
    int lostLineTicks = 0;
    unsigned int sensors;

    while (!stopButton() && ticks < RETURN_START_SEARCH_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0);

        if ((sensors & SENSOR_MASK) == 0)
        {
            lostLineTicks++;
            /* FIX BUG-06: guard lost-line stop behind RETURN_START_MIN_TICKS.
               Without this the robot could stop immediately after the last
               return-path turn while still rotating to find the line. */
            if (ticks >= RETURN_START_MIN_TICKS && lostLineTicks >= RETURN_START_LOST_LINE_TICKS)
            {
                printCurrentPose("Stopping at start line end");
                setVel2(0, 0);
                return 1;
            }
        }
        else
        {
            lostLineTicks = 0;
        }

        if (ticks >= RETURN_START_MIN_TICKS && isNearStartPose())
        {
            printCurrentPose("Stopping at");
            setVel2(0, 0);
            return 1;
        }

        if (isReturnIntersectionCandidate(sensors))
        {
            driveForwardTicks(INTERSECTION_APPROACH_TICKS);
            waitUntilNormalLine(lastDirection);
            ticks += INTERSECTION_APPROACH_TICKS;
            continue;
        }

        followLine(sensors, lastDirection);
        ticks++;
    }

    setVel2(0, 0);
    printCurrentPose("Start pose search timeout");
    return 0;
}

static int waitUntilNormalLine(int *lastDirection)
{
    int normalTicks = 0;
    int totalTicks = 0;
    unsigned int sensors;

    while (!stopButton() && totalTicks < INTERSECTION_CLEAR_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0);

        followLine(sensors, lastDirection);

        if (isNormalLine(sensors))
        {
            normalTicks++;
            if (normalTicks >= INTERSECTION_CLEAR_TICKS)
            {
                printf("Intersection cleared\n");
                return 1;
            }
        }
        else
        {
            normalTicks = 0;
        }

        totalTicks++;
    }

    printf("Intersection clear timeout\n");
    return 0;
}

static int turnLeftToLine(void)
{
    int ticks = 0;
    int centerLostTicks = 0;
    unsigned int sensors = 0;

    setVel2(-TURN_SPEED, TURN_SPEED);

    while (!stopButton() && ticks < LEFT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (sensors & SENSOR_CENTER)
        {
            if (ticks >= LEFT_TURN_MIN_TICKS && centerLostTicks >= TURN_CENTER_LOST_TICKS)
            {
                setVel2(0, 0);
                return 1;
            }
        }
        else if (ticks >= TURN_CENTER_LOST_TICKS)
        {
            centerLostTicks++;
        }
    }

    setVel2(0, 0);
    return 0;
}

static int turnRightToLine(void)
{
    int ticks = 0;
    int centerLostTicks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while (!stopButton() && ticks < RIGHT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (sensors & SENSOR_CENTER)
        {
            if (ticks >= RIGHT_TURN_MIN_TICKS && centerLostTicks >= TURN_CENTER_LOST_TICKS)
            {
                setVel2(0, 0);
                return 1;
            }
        }
        else if (ticks >= TURN_CENTER_LOST_TICKS)
            centerLostTicks++;
    }

    setVel2(0, 0);
    return 0;
}

static int turnAroundToLineWithMax(int maxTicks)
{
    int ticks = 0;
    int centerLostTicks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while (!stopButton() && ticks < maxTicks)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (sensors & SENSOR_CENTER)
        {
            if (ticks >= TURN_AROUND_MIN_TICKS && centerLostTicks >= TURN_CENTER_LOST_TICKS)
            {
                setVel2(0, 0);
                return 1;
            }
        }
        else if (ticks >= TURN_CENTER_LOST_TICKS)
            centerLostTicks++;
    }

    setVel2(0, 0);
    return 0;
}

static int turnAroundToLine(void)
{
    return turnAroundToLineWithMax(TURN_AROUND_MAX_TICKS);
}

static int turnAroundToLineExtended(void)
{
    return turnAroundToLineWithMax(TURN_AROUND_EXTENDED_MAX_TICKS);
}

static int turnAroundToLineRobust(char *label)
{
    if (turnAroundToLine())
        return 1;

    printf("%s extended turn\n", label);
    return turnAroundToLineExtended();
}

static void executeReturnMove(char move)
{
    printf("Return move: %c\n", move);

    if (move == 'L')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        turnLeftToLine();
    }
    else if (move == 'R')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        turnRightToLine();
    }
    else if (move == 'B')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        turnAroundToLine();
    }
    else if (move == 'S')
        driveForwardTicks(INTERSECTION_APPROACH_TICKS);
}

static int moveIntoBranch(char move)
{
    if (move == 'L')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return turnLeftToLine();
    }

    if (move == 'R')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return turnRightToLine();
    }

    if (move == 'B')
    {
        driveForwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return turnAroundToLineRobust("Probe B");
    }

    if (move == 'S')
    {
        driveForwardTicks(INTERSECTION_APPROACH_TICKS);
        return 1;
    }

    return 0;
}

static int confirmBranchLine(int *lastDirection, int *probeTicks)
{
    int ticks = 0;
    int consecutiveLineTicks = 0;
    int lostTicks = 0;
    unsigned int sensors;

    *probeTicks = 0;

    while (!stopButton() && ticks < BRANCH_PROBE_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;
        *probeTicks = ticks;

        if (sensors == 0)
        {
            lostTicks++;
            if (lostTicks >= BRANCH_PROBE_LOST_TICKS)
            {
                setVel2(0, 0);
                return 0;
            }
        }
        else
        {
            lostTicks = 0;
            if (ticks > BRANCH_PROBE_MIN_NEXT_JUNCTION_TICKS && isIntersectionCandidate(sensors))
            {
                lastProbeEndedAtIntersection = 1;
                setVel2(0, 0);
                return 1;
            }

            if (ticks > BRANCH_PROBE_SETTLE_TICKS && isNormalLine(sensors))
            {
                consecutiveLineTicks++;
                if (consecutiveLineTicks >= BRANCH_PROBE_MIN_LINE_TICKS)
                {
                    setVel2(0, 0);
                    return 1;
                }
            }
            else
            {
                consecutiveLineTicks = 0;
            }
        }

        followLine(sensors, lastDirection);
    }

    setVel2(0, 0);
    return 0;
}

static int recoverFailedProbe(char move, int probeTicks)
{
    int backupTicks = probeTicks + BRANCH_PROBE_BACKUP_EXTRA_TICKS;
    int recovered = 1;

    printf("Recover probe %c backup=", move);
    printInt(backupTicks, 10);
    printf("\n");

    if (move == 'S')
    {
        driveBackwardTicks(INTERSECTION_APPROACH_TICKS + backupTicks);
        return 1;
    }

    if (move == 'L')
    {
        driveBackwardTicks(backupTicks);
        recovered = turnRightToLine();
        driveBackwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return recovered;
    }

    if (move == 'R')
    {
        driveBackwardTicks(backupTicks);
        recovered = turnLeftToLine();
        driveBackwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return recovered;
    }

    if (move == 'B')
    {
        driveBackwardTicks(backupTicks);
        recovered = turnAroundToLineRobust("Recover B");
        driveBackwardTicks(SENSOR_TO_AXLE_ALIGN_TICKS);
        return recovered;
    }

    return 0;
}

static int probeMove(char move, int *lastDirection)
{
    int savedDirection = *lastDirection;
    int probeTicks = 0;

    printf("Probe move: %c\n", move);

    if (!moveIntoBranch(move))
    {
        printf("Probe %c rejected: turn failed\n", move);
        if (!recoverFailedProbe(move, 0))
        {
            printf("Probe %c recovery failed\n", move);
            *lastDirection = savedDirection;
            return -1;
        }

        *lastDirection = savedDirection;
        return 0;
    }

    if (move == 'L')
        *lastDirection = -1;
    else if (move == 'R')
        *lastDirection = 1;
    else if (move == 'B')
        *lastDirection = 1;

    if (confirmBranchLine(lastDirection, &probeTicks))
    {
        printf("Probe %c accepted ticks=", move);
        printInt(probeTicks, 10);
        printf("\n");
        return 1;
    }

    printf("Probe %c rejected: no stable line ticks=", move);
    printInt(probeTicks, 10);
    printf("\n");
    if (!recoverFailedProbe(move, probeTicks))
    {
        printf("Probe %c recovery failed\n", move);
        *lastDirection = savedDirection;
        return -1;
    }

    *lastDirection = savedDirection;
    return 0;
}

static int activeProbeExplorationMove(char *chosenMove, int *lastDirection)
{
    char baseCandidates[4] = {'L', 'S', 'R', 'B'};
    char candidates[4];
    char skipMove = '\0';
    int candidateCount = 0;
    int probeResult = 0;
    int i;

    lastProbeEndedAtIntersection = 0;

    if (afterDeadEndBacktrack && lastDeadEndEntryMove != '\0')
    {
        /* After returning from a dead-end, the failed branch is behind the
           robot at the parent junction, regardless of the original entry
           move. Skipping B prevents immediately going back into it. */
        skipMove = 'B';
        printf("Backtrack-aware probe entry=");
        printf("%c skip=", lastDeadEndEntryMove);
        printf("%c\n", skipMove);
    }

    for (i = 0; i < 4; i++)
    {
        if (baseCandidates[i] == skipMove)
            continue;

        candidates[candidateCount] = baseCandidates[i];
        candidateCount++;
    }

    printf("Probe order:");
    for (i = 0; i < candidateCount; i++)
        printf(" %c", candidates[i]);
    printf("\n");

    for (i = 0; i < candidateCount; i++)
    {
        probeResult = probeMove(candidates[i], lastDirection);

        if (probeResult < 0)
            return 0;

        if (probeResult)
        {
            *chosenMove = candidates[i];
            afterDeadEndBacktrack = 0;
            printf("Exploration move selected: %c\n", *chosenMove);
            return 1;
        }
    }

    return 0;
}

static int runReturnNavigation(void)
{
    unsigned int sensors;
    unsigned int returnIndex = 0;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int returnCandidateTicks = 0;
    int rearmNormalTicks = 0;
    int completed = 0;

    leds(LED_EXPLORING | LED_TARGET_FOUND);
    printf("Return navigation starting\n");

    turnAroundToLine();
    /* FIX BUG-04: drive forward past the target wide line/block before
       arming intersection detection. Without this, the wide target marker
       is immediately re-detected as an intersection and the first return
       move is consumed prematurely. */
    driveForwardTicks(INTERSECTION_APPROACH_TICKS);
    intersectionArmed = waitUntilNormalLine(&lastDirection);

    while (!stopButton())
    {
        waitTick20ms();
        sensors = readLineSensors(0);

        if (returnIndex >= returnPathLength)
        {
            printf("Return decisions complete; searching start pose\n");
            completed = followLineUntilStartPose(&lastDirection);
            break;
        }

        if (!intersectionArmed)
        {
            if (isNormalLine(sensors))
            {
                rearmNormalTicks++;
                if (rearmNormalTicks >= INTERSECTION_CLEAR_TICKS)
                {
                    intersectionArmed = 1;
                    rearmNormalTicks = 0;
                    returnCandidateTicks = 0;
                    printf("Intersection rearmed\n");
                }
            }
            else
            {
                rearmNormalTicks = 0;
            }
        }

        if (intersectionArmed && isReturnIntersectionCandidate(sensors))
            returnCandidateTicks++;
        else
            returnCandidateTicks = 0;

        if (intersectionArmed && returnCandidateTicks >= RETURN_INTERSECTION_DETECT_TICKS)
        {
            leds(LED_INTERSECTION);
            returnCandidateTicks = 0;
            executeReturnMove(returnPath[returnIndex]);
            returnIndex++;
            intersectionArmed = waitUntilNormalLine(&lastDirection);
            rearmNormalTicks = 0;
            leds(LED_EXPLORING | LED_TARGET_FOUND);
        }
        else
        {
            followLine(sensors, &lastDirection);
        }
    }

    setVel2(0, 0);
    return completed;
}

static int pauseBeforeAutoReturn(void)
{
    int ticks;

    setVel2(0, 0);
    leds(LED_TARGET_FOUND);

    printf("Target reached; auto return in 3 seconds\n");

    for (ticks = 0; ticks < TARGET_RETURN_PAUSE_TICKS && !stopButton(); ticks++)
        waitTick20ms();

    return !stopButton();
}

int main(void)
{
    unsigned int sensors;
    unsigned int junctionSensors;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int intersectionHistory = 0;
    int intersectionHitTicks = 0;
    int targetFound = 0;
    int lineWidthTicks = 0;
    int lostLineTicks = 0;
    int clearResult = 1;
    int junctionCooldownTicks = 0;
    int rearmNormalTicks = 0;
    char chosenMove = 'S';
    RobotMode mode = MODE_IDLE;

    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    leds(0x00);

    printf("Path Finder Robot - exploration left ");
    printf(CODE_VERSION);
    printf("\n");

    while (1)
    {
        printf("Press start to run\n");
        while (!startButton())
            ;

        setRobotPos(0.0, 0.0, 0.0);
        printCurrentPose("Start pose reset");

        mode = MODE_EXPLORE;
        leds(LED_EXPLORING);
        lastDirection = 1;
        intersectionArmed = 1;
        intersectionHistory = 0;
        intersectionHitTicks = 0;
        targetFound = 0;
        lostLineTicks = 0;
        junctionCooldownTicks = 0;
        rearmNormalTicks = 0;
        afterDeadEndBacktrack = 0;
        lastDeadEndEntryMove = '\0';
        resetPath();
        resetOptimizedPath();
        resetReturnPath();

        while (!stopButton())
        {
            waitTick20ms();
            sensors = readLineSensors(0);

            if ((sensors & SENSOR_MASK) == 0)
                lostLineTicks++;
            else
                lostLineTicks = 0;

            if (junctionCooldownTicks > 0)
            {
                junctionCooldownTicks--;
                intersectionHistory = 0;
                intersectionHitTicks = 0;
                lostLineTicks = 0;
                rearmNormalTicks = 0;
            }

            if (!intersectionArmed && junctionCooldownTicks == 0)
            {
                if (isNormalLine(sensors))
                {
                    rearmNormalTicks++;
                    if (rearmNormalTicks >= INTERSECTION_CLEAR_TICKS)
                    {
                        intersectionArmed = 1;
                        rearmNormalTicks = 0;
                        intersectionHistory = 0;
                        intersectionHitTicks = 0;
                        printf("Intersection rearmed\n");
                    }
                }
                else
                {
                    rearmNormalTicks = 0;
                }
            }

            if (intersectionArmed && junctionCooldownTicks == 0 && lostLineTicks >= LOST_LINE_DEAD_END_TICKS)
            {
                leds(LED_INTERSECTION);
                printf("Dead end detected\n");
                lostLineTicks = 0;
                lastDeadEndEntryMove = directDeadEndEntryMove();
                printf("Dead end entry move: ");
                if (lastDeadEndEntryMove == '\0')
                    printf("none");
                else
                    printf("%c", lastDeadEndEntryMove);
                printf("\n");
                recordMove('B');
                if (pathOverflow)
                {
                    setVel2(0, 0);
                    printf("[ERROR] Path memory overflow - map too large\n");
                    break;
                }
                if (!backupUntilLineSeen(DEAD_END_REACQUIRE_MAX_TICKS) || !turnAroundToLineRobust("Dead end"))
                {
                    setVel2(0, 0);
                    printf("Dead end turn failed; stopping\n");
                    break;
                }
                lastDirection = 1;
                afterDeadEndBacktrack = 1;
                clearResult = waitUntilNormalLine(&lastDirection);
                driveForwardTicks(5);
                intersectionArmed = clearResult;
                rearmNormalTicks = 0;
                junctionCooldownTicks = JUNCTION_REARM_COOLDOWN_TICKS;
                leds(LED_EXPLORING);
                continue;
            }

            if (intersectionArmed && junctionCooldownTicks == 0)
            {
                intersectionHistory <<= 1;
                if (isIntersectionCandidate(sensors))
                    intersectionHistory |= 1;
                intersectionHistory &= 0x07;

                intersectionHitTicks = 0;
                if (intersectionHistory & 0x01)
                    intersectionHitTicks++;
                if (intersectionHistory & 0x02)
                    intersectionHitTicks++;
                if (intersectionHistory & 0x04)
                    intersectionHitTicks++;

                if (intersectionHitTicks >= INTERSECTION_DEBOUNCE_MIN_HITS)
                {
                    leds(LED_INTERSECTION);
                    printf("Intersection detected\n");
                    intersectionHistory = 0;
                    intersectionHitTicks = 0;
                    junctionSensors = sensors & SENSOR_MASK;
                    lineWidthTicks = measureLineWidthTicks(junctionSensors);

                    /* TARGET DETECTION: accept a sustained all-sensors marker,
                       or a much wider-than-normal marker. Slow probing made
                       ordinary junctions reach width=10/fullMask=0, so width
                       alone is intentionally stricter. */
                    if (lineWidthTicks >= TARGET_WIDTH_MIN_TICKS || lastFullMaskTicks >= TARGET_FULL_MASK_MIN_TICKS)
                    {
                        targetFound = 1;
                        mode = MODE_TARGET_FOUND;
                        driveBackwardTicks(clamp((lineWidthTicks + 1) / 2, 0, TARGET_BACKUP_MAX_TICKS));
                        confirmTarget(junctionSensors, lineWidthTicks, "at intersection");
                        break;
                    }

                    driveBackwardTicks(clamp(lineWidthTicks, 0, TARGET_BACKUP_MAX_TICKS));

                    if (!stopButton())
                    {
                        chosenMove = chooseExplorationMoveFromSensors(junctionSensors);
                        printf("Decision sensors=");
                        printInt(junctionSensors, 2 | 5 << 16);
                        printf(" move=%c\n", chosenMove);

                        lastProbeEndedAtIntersection = 0;

                        if (chosenMove == 'B')
                        {
                            printf("Decision fallback to active probe\n");
                            if (!activeProbeExplorationMove(&chosenMove, &lastDirection))
                            {
                                setVel2(0, 0);
                                printf("Active probing failed; stopping\n");
                                break;
                            }
                        }
                        else if (!moveIntoBranch(chosenMove))
                        {
                            setVel2(0, 0);
                            printf("Exploration move %c failed; stopping\n", chosenMove);
                            break;
                        }

                        recordMove(chosenMove);
                        if (pathOverflow)
                        {
                            setVel2(0, 0);
                            printf("[ERROR] Path memory overflow - map too large\n");
                            break;
                        }
                        afterDeadEndBacktrack = 0;
                        lastDeadEndEntryMove = '\0';

                        if (chosenMove == 'L')
                            lastDirection = -1;
                        else if (chosenMove == 'R')
                            lastDirection = 1;

                        if (lastProbeEndedAtIntersection)
                        {
                            printf("Probe ended at next intersection\n");
                            intersectionArmed = 1;
                            intersectionHistory = 0;
                            intersectionHitTicks = 0;
                            rearmNormalTicks = 0;
                            junctionCooldownTicks = 0;
                        }
                        else
                        {
                            clearResult = waitUntilNormalLine(&lastDirection);
                            intersectionArmed = clearResult;
                            rearmNormalTicks = 0;
                            junctionCooldownTicks = JUNCTION_REARM_COOLDOWN_TICKS;
                        }
                    }
                    leds(LED_EXPLORING);
                    continue;
                }
            }

            if (!intersectionArmed || intersectionHitTicks < INTERSECTION_DEBOUNCE_MIN_HITS)
            {
                if ((sensors & SENSOR_MASK) == 0 && lostLineTicks >= LOST_LINE_SEARCH_TICKS)
                    setVel2(0, 0);
                else
                    followLine(sensors, &lastDirection);
            }
        }

        setVel2(0, 0);

        if (targetFound && mode == MODE_TARGET_FOUND)
        {
            if (pauseBeforeAutoReturn())
            {
                mode = MODE_RETURN;
                if (runReturnNavigation())
                {
                    mode = MODE_FINISHED;
                    leds(LED_TARGET_FOUND);
                    printf("Finished; press stop to reset\n");
                }
            }

            while (!stopButton())
                waitTick20ms();
        }

        mode = MODE_IDLE;
        leds(0x00);

        while (stopButton())
            ;
    }

    return 0;
}
