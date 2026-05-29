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
#define TARGET_CONFIRM_TICKS 8
#define TARGET_CONFIRM_MIN_HITS 5
#define TARGET_MIN_BLACK_SENSORS 4
#define TARGET_SCAN_SPEED 15

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

static void resetPath(void)
{
    pathLength = 0;
    pathOverflow = 0;
    pathMemory[0] = '\0';
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
        || ((sensors & LEFT_BRANCH)
            && hasForwardPath(sensors)
            && (sensors & RIGHT_BRANCH));
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

        if(isTargetCandidate(sensors))
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

static void chooseLeftAtIntersection(void)
{
    driveForwardTicks(INTERSECTION_APPROACH_TICKS);
    if(!stopButton())
        turnLeftToLine();
}

int main(void)
{
    unsigned int sensors;
    int lastDirection = 1;
    int intersectionArmed = 1;
    int targetFound = 0;

    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    leds(0x00);

    printf("Path Finder Robot - exploration left\n");

    while(1)
    {
        printf("Press start to run\n");
        while(!startButton());

        leds(LED_EXPLORING);
        lastDirection = 1;
        intersectionArmed = 1;
        targetFound = 0;
        resetPath();

        while(!stopButton())
        {
            waitTick20ms();
            sensors = readLineSensors(0);

            if(isTargetCandidate(sensors))
            {
                if(confirmTarget(sensors))
                {
                    setVel2(0, 0);
                    leds(LED_TARGET_FOUND);
                    targetFound = 1;

                    printf("Target confirmed sensors=");
                    printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
                    printf("\n");
                    printPath();
                    break;
                }

                printf("Target rejected sensors=");
                printInt(sensors & SENSOR_MASK, 2 | 5 << 16);
                printf("\n");
                sensors = readLineSensors(0);
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

        if(targetFound)
        {
            leds(LED_TARGET_FOUND);
            printf("Exploration stopped at target; return phase pending\n");
            while(!stopButton())
                waitTick20ms();
        }

        leds(0x00);

        while(stopButton());
    }

    return 0;
}
