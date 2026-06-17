#include "rm-mr32.h"

#define BASE_SPEED 40
#define TURN_GAIN 7
#define SEARCH_SPEED 25
#define TURN_SPEED 40
#define INTERSECTION_SPEED 30

#define INTERSECTION_APPROACH_TICKS 8 
#define INTERSECTION_CONFIRM_TICKS 4
#define INTERSECTION_MIN_WIDTH_TICKS 3
#define REJECTED_INTERSECTION_RECOVERY_TICKS 30
#define LEFT_TURN_MIN_TICKS 8
#define LEFT_TURN_MAX_TICKS 50
#define RIGHT_TURN_MIN_TICKS 8
#define RIGHT_TURN_MAX_TICKS 50
#define TURN_AROUND_MIN_TICKS 20
#define TURN_AROUND_MAX_TICKS 80
#define DEAD_END_TURN_SEARCH_TICKS 160
#define TARGET_MIN_BLACK_SENSORS 3
#define LINE_WIDTH_SCAN_MAX_TICKS 30
#define TARGET_WIDTH_MIN_TICKS 14
#define TARGET_BACKUP_MAX_TICKS 6
#define REJECTED_BACKUP_MIN_TICKS 1
#define LINE_WIDTH_SCAN_SPEED 15
#define INTERSECTION_CLEAR_TICKS 6
#define JUNCTION_LOCKOUT_MIN_TICKS 8
#define JUNCTION_CLEAR_STABLE_TICKS 6
#define JUNCTION_REARM_COOLDOWN_TICKS 10
#define JUNCTION_LOCKOUT_FAIL_TICKS 300
#define INTERSECTION_CLEAR_MAX_TICKS 100
#define LINE_CENTER_STABLE_TICKS 5
#define LINE_CENTER_SETTLE_MAX_TICKS 50
#define INTERSECTION_DEBOUNCE_SAMPLES 3
#define INTERSECTION_DEBOUNCE_MIN_HITS 2
#define RETURN_INTERSECTION_DETECT_TICKS 3
#define LOST_LINE_DEAD_END_TICKS 16
#define DEAD_END_LOCKOUT_TICKS 60
#define DEAD_END_CONFIRM_RECOVERY_TICKS 35
#define JUNCTION_CLEAR_RECOVERY_TICKS (JUNCTION_LOCKOUT_FAIL_TICKS - INTERSECTION_CLEAR_MAX_TICKS)
#define START_REACHED_RADIUS_MM 100
#define RETURN_START_MIN_TICKS 20
#define RETURN_START_LOST_LINE_TICKS 10
#define RETURN_START_SEARCH_MAX_TICKS 600
#define CODE_VERSION "probe-target-v24-junction-lockout"

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

typedef enum
{
    MODE_IDLE,
    MODE_EXPLORE,
    MODE_TARGET_FOUND,
    MODE_RETURN,
    MODE_FINISHED
} RobotMode;

typedef enum
{
    MAP_DECISION_IDLE,
    MAP_DECISION_PENDING
} MappingDecisionState;

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
static int isClearNormalLine(unsigned int sensors);
static int isCenteredNormalLine(unsigned int sensors);
static int isIntersectionCandidate(unsigned int sensors);
static int waitUntilNormalLine(int *lastDirection, int detectTarget);

static int commitPendingMove(MappingDecisionState *decisionState, char move)
{
    if (*decisionState != MAP_DECISION_PENDING)
    {
        printf("[WARN] Ignored move without pending junction decision\n");
        return 1;
    }

    recordMove(move);
    if (pathOverflow)
    {
        setVel2(0, 0);
        printf("[ERROR] Path memory overflow - map too large\n");
        return 0;
    }

    printf("Recorded move=");
    printf("%c\n", move);
    *decisionState = MAP_DECISION_IDLE;
    return 1;
}

static int hasReturnIntersection(unsigned int sensors)
{
    int branches = 0;

    sensors &= SENSOR_MASK;

    if (hasLeftPath(sensors))
        branches++;
    if (hasForwardPath(sensors))
        branches++;
    if (hasRightPath(sensors))
        branches++;

    return branches >= 2;
}

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

    // Made more robust: any clean center-focused reading counts
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

static int isClearNormalLine(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    if (sensors == SENSOR_CENTER)
        return 1;
    if (sensors == (SENSOR_LEFT_1 | SENSOR_CENTER))
        return 1;
    if (sensors == (SENSOR_CENTER | SENSOR_RIGHT_1))
        return 1;

    return 0;
}

static int isCenteredNormalLine(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    if ((sensors & SENSOR_CENTER) == 0)
        return 0;
    if (sensors & (SENSOR_LEFT_2 | SENSOR_RIGHT_2))
        return 0;

    return lineError(sensors) == 0;
}

static int isLineWidthReading(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    return activeSensorCount(sensors) >= TARGET_MIN_BLACK_SENSORS && hasForwardPath(sensors) && (hasLeftPath(sensors) || hasRightPath(sensors));
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

static char chooseExplorationMove(unsigned int sensors)
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

static int measureLineWidthTicks(unsigned int firstSensors)
{
    int widthTicks = 0;
    unsigned int sensors = firstSensors & SENSOR_MASK;

    setVel2(LINE_WIDTH_SCAN_SPEED, LINE_WIDTH_SCAN_SPEED);
    while (!stopButton() && widthTicks < LINE_WIDTH_SCAN_MAX_TICKS && isLineWidthReading(sensors))
    {
        widthTicks++;
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
    }

    setVel2(0, 0);

    printf("Line width ticks=");
    printInt(widthTicks, 10);
    printf("\n");

    return widthTicks;
}

static void printCurrentPose(char *label);
static void driveBackwardTicks(int ticks);

static void confirmTarget(unsigned int sensors, int widthTicks, char *source)
{
    setVel2(0, 0);
    leds(LED_TARGET_FOUND);

    printf("Target confirmed ");
    printf(source);
    printf(" sensors=");
    printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
    printf(" width=");
    printInt(widthTicks, 10);
    printf("\n");
    printCurrentPose("Target pose");
    printPath();
    optimizePath();
    printOptimizedPath();
    buildReturnPath();
    printReturnPath();
}

static int probeTarget(unsigned int sensors, char *source)
{
    int widthTicks;

    if (!isLineWidthReading(sensors))
        return 0;

    widthTicks = measureLineWidthTicks(sensors);
    if (widthTicks < TARGET_WIDTH_MIN_TICKS)
    {
        driveBackwardTicks(clamp(widthTicks / 2, 0, TARGET_BACKUP_MAX_TICKS));
        return 0;
    }

    driveBackwardTicks(clamp(widthTicks / 4, 0, TARGET_BACKUP_MAX_TICKS));
    confirmTarget(sensors, widthTicks, source);
    return 1;
}

static void driveForwardTicks(int ticks)
{
    int i;

    setVel2(INTERSECTION_SPEED, INTERSECTION_SPEED);
    for (i = 0; i < ticks && !stopButton(); i++)
        waitTick20ms();
}

static void driveBackwardTicks(int ticks)
{
    int i;

    setVel2(-LINE_WIDTH_SCAN_SPEED, -LINE_WIDTH_SCAN_SPEED);
    for (i = 0; i < ticks && !stopButton(); i++)
        waitTick20ms();
    setVel2(0, 0);
}

static void printCurrentPose(char *label)
{
    double x;
    double y;
    double theta;

    getRobotPos(&x, &y, &theta);
    printf(label);
    printf(" x=");
    printInt((int)x, 10);
    printf(" y=");
    printInt((int)y, 10);
    printf(" theta=");
    printInt((int)(theta * 1000.0), 10);
    printf("mrad\n");
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
            if (lostLineTicks >= RETURN_START_LOST_LINE_TICKS)
            {
                if (ticks >= RETURN_START_MIN_TICKS && isNearStartPose())
                {
                    printCurrentPose("Stopping at start line end");
                    setVel2(0, 0);
                    return 1;
                }
            }
        }
        else
        {
            lostLineTicks = 0;
        }

        if (isReturnIntersectionCandidate(sensors))
        {
            driveForwardTicks(INTERSECTION_APPROACH_TICKS);
            if (!waitUntilNormalLine(lastDirection, 0))
                return 0;
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

static int settleCenteredOnLine(int *lastDirection)
{
    int ticks;
    int centeredTicks = 0;
    unsigned int sensors;

    printf("Centering on line after junction\n");

    for (ticks = 0; ticks < LINE_CENTER_SETTLE_MAX_TICKS && !stopButton(); ticks++)
    {
        waitTick20ms();
        sensors = readLineSensors(0);
        followLine(sensors, lastDirection);

        if (isCenteredNormalLine(sensors))
        {
            centeredTicks++;
            if (centeredTicks >= LINE_CENTER_STABLE_TICKS)
            {
                printf("Line centered\n");
                return 1;
            }
        }
        else
        {
            centeredTicks = 0;
        }
    }

    setVel2(0, 0);
    printf("[ERROR] Line centering failed\n");
    return 0;
}

static int recoverStableNormalLine(int *lastDirection, int maxTicks, char *label)
{
    int ticks;
    int normalTicks = 0;
    unsigned int sensors;

    printf("%s\n", label);

    for (ticks = 0; ticks < maxTicks && !stopButton(); ticks++)
    {
        waitTick20ms();
        sensors = readLineSensors(0);
        followLine(sensors, lastDirection);

        if (isClearNormalLine(sensors))
        {
            normalTicks++;
            if (normalTicks >= JUNCTION_CLEAR_STABLE_TICKS)
            {
                printf("Stable line reacquired\n");
                return settleCenteredOnLine(lastDirection);
            }
        }
        else
        {
            normalTicks = 0;
        }
    }

    setVel2(0, 0);
    return 0;
}

static int recoverLineBeforeDeadEnd(int *lastDirection)
{
    if (recoverStableNormalLine(lastDirection, DEAD_END_CONFIRM_RECOVERY_TICKS, "Checking lost line before dead end"))
    {
        printf("Dead end ignored - line recovered\n");
        return 1;
    }

    return 0;
}

static int waitUntilNormalLine(int *lastDirection, int detectTarget)
{
    int normalTicks = 0;
    int totalTicks = 0;
    unsigned int sensors;

    (void)detectTarget;

    while (!stopButton() && totalTicks < INTERSECTION_CLEAR_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0);

        followLine(sensors, lastDirection);

        if (isClearNormalLine(sensors))
        {
            normalTicks++;
            if (totalTicks >= JUNCTION_LOCKOUT_MIN_TICKS && normalTicks >= JUNCTION_CLEAR_STABLE_TICKS)
            {
                printf("Intersection cleared\n");
                return settleCenteredOnLine(lastDirection);
            }
        }
        else
        {
            normalTicks = 0;
        }

        totalTicks++;
    }

    printf("Intersection clear timeout; holding junction lockout\n");
    if (recoverStableNormalLine(lastDirection, JUNCTION_CLEAR_RECOVERY_TICKS, "Junction clear recovery searching for stable line"))
    {
        printf("Intersection cleared\n");
        return 1;
    }

    setVel2(0, 0);
    printf("[ERROR] Junction lockout failed - stable line not reacquired\n");
    return 0;
}

static int turnLeftToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(-TURN_SPEED, TURN_SPEED);

    while (!stopButton() && ticks < LEFT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        // Look for the center sensor to re-acquire the line
        if (ticks >= LEFT_TURN_MIN_TICKS && (sensors & SENSOR_CENTER))
        {
            setVel2(0, 0);
            return 1;
        }
    }

    setVel2(0, 0);
    return 0;
}

static int turnRightToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while (!stopButton() && ticks < RIGHT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (ticks >= RIGHT_TURN_MIN_TICKS && (sensors & SENSOR_CENTER))
        {
            setVel2(0, 0);
            return 1;
        }
    }

    setVel2(0, 0);
    return 0;
}

static int turnAroundToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while (!stopButton() && ticks < TURN_AROUND_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (ticks >= TURN_AROUND_MIN_TICKS && isClearNormalLine(sensors))
        {
            setVel2(0, 0);
            return 1;
        }
    }

    setVel2(0, 0);
    return 0;
}

static int turnAroundSearchToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    if (turnAroundToLine())
        return 1;

    printf("Dead end turn searching for line\n");
    setVel2(TURN_SPEED, -TURN_SPEED);

    while (!stopButton() && ticks < DEAD_END_TURN_SEARCH_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if (isClearNormalLine(sensors) || (sensors & SENSOR_CENTER))
        {
            setVel2(0, 0);
            return 1;
        }
    }

    setVel2(0, 0);
    return 0;
}

static void executeReturnMove(char move)
{
    printf("Return move: %c\n", move);

    if (move == 'L')
        turnLeftToLine();
    else if (move == 'R')
        turnRightToLine();
    else if (move == 'B')
        turnAroundToLine();
    else if (move == 'S')
        driveForwardTicks(INTERSECTION_APPROACH_TICKS);
}

static int executeExplorationMove(char move)
{
    printf("Exploration move: %c\n", move);

    if (move == 'L')
        return turnLeftToLine();
    if (move == 'R')
        return turnRightToLine();
    if (move == 'B')
        return turnAroundToLine();
    if (move == 'S')
    {
        driveForwardTicks(INTERSECTION_APPROACH_TICKS);
        return 1;
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
    int completed = 0;

    leds(LED_EXPLORING | LED_TARGET_FOUND);
    printf("Return navigation starting\n");

    turnAroundToLine();
    intersectionArmed = waitUntilNormalLine(&lastDirection, 0);
    if (!intersectionArmed)
        return 0;

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
            intersectionArmed = waitUntilNormalLine(&lastDirection, 0);
            if (!intersectionArmed)
                break;
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

static int waitForReturnStart(void)
{
    setVel2(0, 0);
    leds(LED_TARGET_FOUND);

    printf("Target reached; press start for return or stop to reset\n");

    while (startButton() && !stopButton())
        waitTick20ms();

    while (!startButton() && !stopButton())
        waitTick20ms();

    return !stopButton();
}

int main(void)
{
    unsigned int sensors;
    unsigned int junctionSensors;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int intersectionHitTicks = 0;
    int targetFound = 0;
    int lineWidthTicks = 0;
    int lostLineTicks = 0;
    int clearResult = 1;
    int junctionCooldownTicks = 0;
    int rejectedRecoveryTicks = 0;
    int deadEndLockoutTicks = 0;
    char chosenMove = 'S';
    MappingDecisionState mappingDecision = MAP_DECISION_IDLE;
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
        intersectionHitTicks = 0;
        targetFound = 0;
        lostLineTicks = 0;
        junctionCooldownTicks = 0;
        rejectedRecoveryTicks = 0;
        deadEndLockoutTicks = 0;
        mappingDecision = MAP_DECISION_IDLE;
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

            if (rejectedRecoveryTicks > 0)
            {
                rejectedRecoveryTicks--;
                lostLineTicks = 0;
            }

            if (deadEndLockoutTicks > 0)
            {
                deadEndLockoutTicks--;
                lostLineTicks = 0;
            }

            if (junctionCooldownTicks == 0 && rejectedRecoveryTicks == 0 && deadEndLockoutTicks == 0 && lostLineTicks >= LOST_LINE_DEAD_END_TICKS)
            {
                if (recoverLineBeforeDeadEnd(&lastDirection))
                {
                    lostLineTicks = 0;
                    deadEndLockoutTicks = DEAD_END_LOCKOUT_TICKS;
                    continue;
                }

                leds(LED_INTERSECTION);
                printf("Dead end detected\n");
                lostLineTicks = 0;
                mappingDecision = MAP_DECISION_PENDING;
                if (!turnAroundSearchToLine())
                {
                    mappingDecision = MAP_DECISION_IDLE;
                    setVel2(0, 0);
                    printf("[ERROR] Dead end search failed - line not reacquired\n");
                    break;
                }

                lastDirection = 1;
                clearResult = waitUntilNormalLine(&lastDirection, 1);
                if (clearResult == 2)
                {
                    if (!commitPendingMove(&mappingDecision, 'B'))
                        break;
                    targetFound = 1;
                    mode = MODE_TARGET_FOUND;
                    break;
                }
                if (!clearResult)
                {
                    mappingDecision = MAP_DECISION_IDLE;
                    break;
                }
                if (!commitPendingMove(&mappingDecision, 'B'))
                    break;
                intersectionArmed = clearResult;
                junctionCooldownTicks = JUNCTION_REARM_COOLDOWN_TICKS;
                deadEndLockoutTicks = DEAD_END_LOCKOUT_TICKS;
                leds(LED_EXPLORING);
                continue;
            }

            if (junctionCooldownTicks > 0)
            {
                junctionCooldownTicks--;
                intersectionHitTicks = 0;
                lostLineTicks = 0;
            }

            if (intersectionArmed && junctionCooldownTicks == 0)
            {
                if (isIntersectionCandidate(sensors))
                    intersectionHitTicks++;
                else
                    intersectionHitTicks = 0;

                if (intersectionHitTicks >= INTERSECTION_CONFIRM_TICKS)
                {
                    leds(LED_INTERSECTION);
                    printf("Intersection detected\n");
                    intersectionHitTicks = 0;
                    junctionSensors = sensors & SENSOR_MASK;
                    lineWidthTicks = measureLineWidthTicks(junctionSensors);

                    if (lineWidthTicks < INTERSECTION_MIN_WIDTH_TICKS)
                    {
                        printf("Intersection rejected width=");
                        printInt(lineWidthTicks, 10);
                        printf("\n");
                        driveBackwardTicks(clamp(lineWidthTicks, REJECTED_BACKUP_MIN_TICKS, TARGET_BACKUP_MAX_TICKS));
                        leds(LED_EXPLORING);
                        junctionCooldownTicks = JUNCTION_REARM_COOLDOWN_TICKS;
                        rejectedRecoveryTicks = REJECTED_INTERSECTION_RECOVERY_TICKS;
                        deadEndLockoutTicks = DEAD_END_LOCKOUT_TICKS;
                        lostLineTicks = 0;
                        recoverStableNormalLine(&lastDirection, REJECTED_INTERSECTION_RECOVERY_TICKS, "Rejected intersection recovery");
                        continue;
                    }

                    if (lineWidthTicks >= TARGET_WIDTH_MIN_TICKS)
                    {
                        targetFound = 1;
                        mode = MODE_TARGET_FOUND;
                        driveBackwardTicks(clamp(lineWidthTicks / 4, 0, TARGET_BACKUP_MAX_TICKS));
                        confirmTarget(junctionSensors, lineWidthTicks, "at intersection");
                        break;
                    }

                    driveBackwardTicks(clamp(lineWidthTicks / 2, 0, TARGET_BACKUP_MAX_TICKS));

                    if (!stopButton())
                    {
                        mappingDecision = MAP_DECISION_PENDING;
                        chosenMove = chooseExplorationMove(junctionSensors);
                        if (!executeExplorationMove(chosenMove))
                        {
                            printf("Exploration move failed\n");
                            if (chosenMove == 'L' && hasForwardPath(junctionSensors))
                            {
                                printf("Recovering from failed left; using straight\n");
                                if (turnRightToLine())
                                {
                                    chosenMove = 'S';
                                    if (!executeExplorationMove(chosenMove))
                                        chosenMove = '\0';
                                }
                                else
                                {
                                    chosenMove = '\0';
                                }
                            }
                            else
                            {
                                printf("Recovering with backtrack\n");
                                chosenMove = 'B';
                                if (!executeExplorationMove(chosenMove))
                                    chosenMove = '\0';
                            }
                        }

                        if (chosenMove == '\0')
                        {
                            mappingDecision = MAP_DECISION_IDLE;
                            setVel2(0, 0);
                            printf("[ERROR] Exploration recovery failed - line not reacquired\n");
                            break;
                        }

                        if (chosenMove == 'L')
                            lastDirection = -1;
                        else if (chosenMove == 'R')
                            lastDirection = 1;

                        clearResult = waitUntilNormalLine(&lastDirection, 1);
                        if (clearResult == 2)
                        {
                            if (!commitPendingMove(&mappingDecision, chosenMove))
                                break;
                            targetFound = 1;
                            mode = MODE_TARGET_FOUND;
                            break;
                        }
                        if (!clearResult)
                        {
                            mappingDecision = MAP_DECISION_IDLE;
                            break;
                        }
                        if (!commitPendingMove(&mappingDecision, chosenMove))
                            break;
                        intersectionArmed = clearResult;
                        junctionCooldownTicks = JUNCTION_REARM_COOLDOWN_TICKS;
                        deadEndLockoutTicks = DEAD_END_LOCKOUT_TICKS;
                    }
                    leds(LED_EXPLORING);
                    continue;
                }
            }

            if (!intersectionArmed || intersectionHitTicks < INTERSECTION_CONFIRM_TICKS)
            {
                followLine(sensors, &lastDirection);
            }
        }

        setVel2(0, 0);

        if (targetFound && mode == MODE_TARGET_FOUND)
        {
            if (waitForReturnStart())
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
