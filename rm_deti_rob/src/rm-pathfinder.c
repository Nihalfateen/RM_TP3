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

static int hasLeftIntersection(unsigned int sensors)
{
    sensors &= SENSOR_MASK;

    return (sensors & LEFT_BRANCH) == LEFT_BRANCH
        && (sensors & (CENTER_BRANCH | RIGHT_BRANCH));
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

    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    leds(0x00);

    printf("Path Finder Robot - exploration left\n");

    while(1)
    {
        printf("Press start to run\n");
        while(!startButton());

        leds(0x01);
        lastDirection = 1;
        intersectionArmed = 1;

        while(!stopButton())
        {
            waitTick20ms();
            sensors = readLineSensors(0);

            if(intersectionArmed && hasLeftIntersection(sensors))
            {
                leds(0x03);
                chooseLeftAtIntersection();
                lastDirection = -1;
                intersectionArmed = 0;
                leds(0x01);
            }
            else
            {
                followLine(sensors, &lastDirection);

                if(!hasLeftIntersection(sensors))
                    intersectionArmed = 1;
            }
        }

        setVel2(0, 0);
        leds(0x00);

        while(stopButton());
    }

    return 0;
}
