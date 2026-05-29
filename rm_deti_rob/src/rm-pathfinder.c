#include "rm-mr32.h"
#include <stdio.h>

#define BASE_SPEED 45
#define TURN_GAIN 7
#define SEARCH_SPEED 25
#define TURN_SPEED 35
#define INTERSECTION_SPEED 30

#define INTERSECTION_APPROACH_TICKS 6
#define LEFT_TURN_MIN_TICKS 8
#define LEFT_TURN_MAX_TICKS 45
#define RIGHT_TURN_MIN_TICKS 8
#define RIGHT_TURN_MAX_TICKS 45
#define TURN_AROUND_MIN_TICKS 18
#define TURN_AROUND_MAX_TICKS 75
#define TARGET_CONFIRM_TICKS 8
#define TARGET_CONFIRM_MIN_HITS 6
#define TARGET_MIN_BLACK_SENSORS 3
#define TARGET_SCAN_SPEED 15
#define TARGET_READY_TICKS 5
#define CODE_VERSION "target-stable-v4"

#define MIN_SPEED -100
#define MAX_SPEED 100

#define SENSOR_LEFT_2  0x10
#define SENSOR_LEFT_1  0x08
#define SENSOR_CENTER  0x04
#define SENSOR_RIGHT_1 0x02
#define SENSOR_RIGHT_2 0x01

#define SENSOR_MASK     0x1F
#define LEFT_BRANCH     (SENSOR_LEFT_2 | SENSOR_LEFT_1)
#define CENTER_BRANCH   SENSOR_CENTER
#define RIGHT_BRANCH    (SENSOR_RIGHT_1 | SENSOR_RIGHT_2)
#define LED_EXPLORING   0x01
#define LED_INTERSECTION 0x03
#define LED_TARGET_FOUND 0x0F

#define PATH_MAX_MOVES 64

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
    if(pathLength >= PATH_MAX_MOVES)
    {
        pathOverflow = 1;
        return;
    }

    pathMemory[pathLength] = move;
    pathLength++;
    pathMemory[pathLength] = '\0';
}

static int moveToAngle(char move)
{
    if(move == 'L')
        return 270;
    if(move == 'S')
        return 0;
    if(move == 'R')
        return 90;
    if(move == 'B')
        return 180;

    return -1;
}

static char angleToMove(int angle)
{
    angle %= 360;

    if(angle < 0)
        angle += 360;

    if(angle == 0)
        return 'S';
    if(angle == 90)
        return 'R';
    if(angle == 180)
        return 'B';
    if(angle == 270)
        return 'L';

    return '\0';
}

static void simplifyOptimizedPathEnd(void)
{
    int firstAngle;
    int secondAngle;
    int thirdAngle;
    char replacement;

    if(optimizedPathLength < 3)
        return;

    if(optimizedPath[optimizedPathLength - 2] != 'B')
        return;

    firstAngle = moveToAngle(optimizedPath[optimizedPathLength - 3]);
    secondAngle = moveToAngle(optimizedPath[optimizedPathLength - 2]);
    thirdAngle = moveToAngle(optimizedPath[optimizedPathLength - 1]);

    if(firstAngle < 0 || secondAngle < 0 || thirdAngle < 0)
        return;

    replacement = angleToMove(firstAngle + secondAngle + thirdAngle);
    if(replacement == '\0')
        return;

    optimizedPathLength -= 3;
    optimizedPath[optimizedPathLength] = replacement;
    optimizedPathLength++;
    optimizedPath[optimizedPathLength] = '\0';
}

static void appendOptimizedMove(char move)
{
    if(moveToAngle(move) < 0)
        return;

    if(optimizedPathLength >= PATH_MAX_MOVES)
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

    for(i = 0; i < pathLength; i++)
        appendOptimizedMove(pathMemory[i]);
}

static char invertMove(char move)
{
    if(move == 'L')
        return 'R';
    if(move == 'R')
        return 'L';
    if(move == 'S')
        return 'S';
    if(move == 'B')
        return 'B';

    return '\0';
}

static void buildReturnPath(void)
{
    int i;
    char move;

    resetReturnPath();

    for(i = (int)optimizedPathLength - 1; i >= 0; i--)
    {
        move = invertMove(optimizedPath[i]);
        if(move == '\0')
            continue;

        if(returnPathLength >= PATH_MAX_MOVES)
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

    for(i = 0; i < pathLength; i++)
        printf("%c", pathMemory[i]);

    if(pathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static void printOptimizedPath(void)
{
    unsigned int i;

    printf("Optimized path (");
    printInt(optimizedPathLength, 10);
    printf(" moves): ");

    for(i = 0; i < optimizedPathLength; i++)
        printf("%c", optimizedPath[i]);

    if(optimizedPathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static void printReturnPath(void)
{
    unsigned int i;

    printf("Return path (");
    printInt(returnPathLength, 10);
    printf(" moves): ");

    for(i = 0; i < returnPathLength; i++)
        printf("%c", returnPath[i]);

    if(returnPathOverflow)
        printf(" [overflow]");

    printf("\n");
}

static int clamp(int value, int minValue, int maxValue)
{
    if(value < minValue)
        return minValue;
    if(value > maxValue)
        return maxValue;
    return value;
}

static int lineError(unsigned int sensors)
{
    int weightedSum = 0;
    int activeCount = 0;

    sensors &= SENSOR_MASK;

    if(sensors & SENSOR_LEFT_2)
    {
        weightedSum -= 4;
        activeCount++;
    }
    if(sensors & SENSOR_LEFT_1)
    {
        weightedSum -= 2;
        activeCount++;
    }
    if(sensors & SENSOR_CENTER)
    {
        activeCount++;
    }
    if(sensors & SENSOR_RIGHT_1)
    {
        weightedSum += 2;
        activeCount++;
    }
    if(sensors & SENSOR_RIGHT_2)
    {
        weightedSum += 4;
        activeCount++;
    }

    if(activeCount == 0)
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

    if(sensors == 0)
    {
        if(*lastDirection < 0)
            setVel2(-SEARCH_SPEED, SEARCH_SPEED);
        else
            setVel2(SEARCH_SPEED, -SEARCH_SPEED);
        return;
    }

    error = lineError(sensors);
    if(error < 0)
        *lastDirection = -1;
    else if(error > 0)
        *lastDirection = 1;

    correction = error * TURN_GAIN;
    leftSpeed = clamp(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
    rightSpeed = clamp(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);

    setVel2(leftSpeed, rightSpeed);
}

static int hasLeftPath(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    return (sensors & LEFT_BRANCH) == LEFT_BRANCH;
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

static int hasLeftIntersection(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    return hasLeftPath(sensors)
        && (hasForwardPath(sensors) || hasRightPath(sensors));
}

static int activeSensorCount(unsigned int sensors);

static int hasReturnIntersection(unsigned int sensors)
{
    int branches = 0;

    sensors &= SENSOR_MASK;

    if(hasLeftPath(sensors))
        branches++;
    if(hasForwardPath(sensors))
        branches++;
    if(hasRightPath(sensors))
        branches++;

    return branches >= 2;
}

static int isStableLineForTarget(unsigned int sensors)
{
    int count;

    sensors &= SENSOR_MASK;
    count = activeSensorCount(sensors);

    return count > 0
        && count <= 3
        && hasForwardPath(sensors)
        && !hasLeftIntersection(sensors);
}

static int activeSensorCount(unsigned int sensors)
{
    int count = 0;

    sensors &= SENSOR_MASK;

    if(sensors & SENSOR_LEFT_2)
        count++;
    if(sensors & SENSOR_LEFT_1)
        count++;
    if(sensors & SENSOR_CENTER)
        count++;
    if(sensors & SENSOR_RIGHT_1)
        count++;
    if(sensors & SENSOR_RIGHT_2)
        count++;

    return count;
}

static int isTargetCandidate(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    return activeSensorCount(sensors) >= TARGET_MIN_BLACK_SENSORS
        && hasForwardPath(sensors)
        && (sensors & (SENSOR_LEFT_1 | SENSOR_LEFT_2))
        && (sensors & (SENSOR_RIGHT_1 | SENSOR_RIGHT_2));
}

static int isSimilarTargetReading(unsigned int firstSensors, unsigned int sensors)
{
    unsigned int firstCore;
    unsigned int currentCore;

    firstSensors &= SENSOR_MASK;
    sensors &= SENSOR_MASK;

    firstCore = firstSensors & (SENSOR_LEFT_1 | SENSOR_CENTER | SENSOR_RIGHT_1);
    currentCore = sensors & (SENSOR_LEFT_1 | SENSOR_CENTER | SENSOR_RIGHT_1);

    return isTargetCandidate(sensors)
        && currentCore == firstCore;
}

static int confirmTarget(unsigned int firstSensors)
{
    int ticks;
    int hits = 0;
    unsigned int sensors = firstSensors & SENSOR_MASK;

    printf("Target scan start sensors=");
    printInt(firstSensors & SENSOR_MASK, 2 | 5 << 16);
    printf("\n");

    setVel2(TARGET_SCAN_SPEED, TARGET_SCAN_SPEED);
    for(ticks = 0; ticks < TARGET_CONFIRM_TICKS && !stopButton(); ticks++)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;

        if(isSimilarTargetReading(firstSensors, sensors))
            hits++;
    }

    setVel2(0, 0);

    printf("Target scan hits=");
    printInt(hits, 10);
    printf(" last=");
    printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
    printf("\n");

    return hits >= TARGET_CONFIRM_MIN_HITS;
}

static void driveForwardTicks(int ticks)
{
    int i;

    setVel2(INTERSECTION_SPEED, INTERSECTION_SPEED);
    for(i = 0; i < ticks && !stopButton(); i++)
        waitTick20ms();
}

static void turnLeftToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(-TURN_SPEED, TURN_SPEED);

    while(!stopButton() && ticks < LEFT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if(ticks >= LEFT_TURN_MIN_TICKS
            && (sensors & SENSOR_CENTER)
            && !hasLeftIntersection(sensors))
        {
            break;
        }
    }
}

static void turnRightToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while(!stopButton() && ticks < RIGHT_TURN_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if(ticks >= RIGHT_TURN_MIN_TICKS
            && (sensors & SENSOR_CENTER)
            && !hasReturnIntersection(sensors))
        {
            break;
        }
    }
}

static void turnAroundToLine(void)
{
    int ticks = 0;
    unsigned int sensors = 0;

    setVel2(TURN_SPEED, -TURN_SPEED);

    while(!stopButton() && ticks < TURN_AROUND_MAX_TICKS)
    {
        waitTick20ms();
        sensors = readLineSensors(0) & SENSOR_MASK;
        ticks++;

        if(ticks >= TURN_AROUND_MIN_TICKS
            && (sensors & SENSOR_CENTER)
            && !hasReturnIntersection(sensors))
        {
            break;
        }
    }
}

static void chooseLeftAtIntersection(void)
{
    driveForwardTicks(INTERSECTION_APPROACH_TICKS);
    if(!stopButton())
        turnLeftToLine();
}

static void executeReturnMove(char move)
{
    printf("Return move: %c\n", move);

    driveForwardTicks(INTERSECTION_APPROACH_TICKS);
    if(stopButton())
        return;

    if(move == 'L')
        turnLeftToLine();
    else if(move == 'R')
        turnRightToLine();
    else if(move == 'B')
        turnAroundToLine();
    else if(move == 'S')
        driveForwardTicks(INTERSECTION_APPROACH_TICKS);
    else
        printf("Unsupported return move ignored: %c\n", move);
}

static int runReturnNavigation(void)
{
    unsigned int sensors;
    unsigned int returnIndex = 0;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int completed = 0;

    leds(LED_EXPLORING | LED_TARGET_FOUND);
    printf("Return navigation starting\n");

    turnAroundToLine();
    intersectionArmed = 0;

    while(!stopButton())
    {
        waitTick20ms();
        sensors = readLineSensors(0);

        if(returnIndex >= returnPathLength)
        {
            setVel2(0, 0);
            printf("Return path complete; temporary stop at presumed start\n");
            completed = 1;
            break;
        }

        if(intersectionArmed && hasReturnIntersection(sensors))
        {
            leds(LED_INTERSECTION);
            executeReturnMove(returnPath[returnIndex]);
            returnIndex++;
            intersectionArmed = 0;
            leds(LED_EXPLORING | LED_TARGET_FOUND);
        }
        else
        {
            followLine(sensors, &lastDirection);

            if(!hasReturnIntersection(sensors))
                intersectionArmed = 1;
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

    while(startButton() && !stopButton())
        waitTick20ms();

    while(!startButton() && !stopButton())
        waitTick20ms();

    return !stopButton();
}

int main(void)
{
    unsigned int sensors;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int targetReadyTicks = 0;
    int targetFound = 0;
    RobotMode mode = MODE_IDLE;

    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    leds(0x00);

    printf("Path Finder Robot - exploration left ");
    printf(CODE_VERSION);
    printf("\n");

    while(1)
    {
        printf("Press start to run\n");
        while(!startButton());

        mode = MODE_EXPLORE;
        leds(LED_EXPLORING);
        lastDirection = 1;
        intersectionArmed = 1;
        targetReadyTicks = 0;
        targetFound = 0;
        resetPath();
        resetOptimizedPath();
        resetReturnPath();

        while(!stopButton())
        {
            waitTick20ms();
            sensors = readLineSensors(0);

            if(isTargetCandidate(sensors))
            {
                if(targetReadyTicks >= TARGET_READY_TICKS && confirmTarget(sensors))
                {
                    setVel2(0, 0);
                    leds(LED_TARGET_FOUND);
                    targetFound = 1;
                    mode = MODE_TARGET_FOUND;

                    printf("Target confirmed sensors=");
                    printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
                    printf("\n");
                    printPath();
                    optimizePath();
                    printOptimizedPath();
                    buildReturnPath();
                    printReturnPath();
                    break;
                }

                printf("Target ignored/rejected ready=");
                printInt(targetReadyTicks, 10);
                printf(" sensors=");
                printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
                printf("\n");
                sensors = readLineSensors(0);
            }

            if(isStableLineForTarget(sensors))
            {
                if(targetReadyTicks < TARGET_READY_TICKS)
                    targetReadyTicks++;
            }
            else if(sensors == 0 || hasLeftIntersection(sensors))
            {
                targetReadyTicks = 0;
            }

            if(intersectionArmed && hasLeftIntersection(sensors))
            {
                leds(LED_INTERSECTION);
                recordMove('L');
                chooseLeftAtIntersection();
                lastDirection = -1;
                intersectionArmed = 0;
                leds(LED_EXPLORING);
            }
            else
            {
                followLine(sensors, &lastDirection);

                if(!hasLeftIntersection(sensors))
                    intersectionArmed = 1;
            }
        }

        setVel2(0, 0);

        if(targetFound && mode == MODE_TARGET_FOUND)
        {
            if(waitForReturnStart())
            {
                mode = MODE_RETURN;
                if(runReturnNavigation())
                {
                    mode = MODE_FINISHED;
                    leds(LED_TARGET_FOUND);
                    printf("Finished; press stop to reset\n");
                }
                else
                {
                    printf("Return interrupted by stop button\n");
                }
            }
            else
            {
                printf("Reset requested at target\n");
            }

            while(!stopButton())
                waitTick20ms();
        }

        mode = MODE_IDLE;
        leds(0x00);

        while(stopButton());
    }

    return 0;
}
