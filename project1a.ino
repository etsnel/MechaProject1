#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <PID_v1.h>
#include <Dabble.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>

Servo rservo;
Servo gservo;
Servo eservo;

// Grip Servo Variables
const int gservoPin = 33;
int grip = 0;
int avalue = 0;
double voltage = 0;
int gcurrentAngle = 0;
const int feedbackPin = A3;
double slope = -0.01427;
double intercept = 3.149455;
double intercept2 = 2.813661;
double calcAngle = 0;
double error = 0;

// Raise servo variables
const int rservoPin = 34; // Pin that raises and lowers the arm
int rcurrentAngle = 110;

// Extend servo variables
const int eservoPin = 35;
int ecurrentAngle = 90; // Forward is lower

// Stepper Motor Variables
const int INA1 = 29;
const int INA2 = 28;
const int INA3 = 27;
const int INA4 = 26;
int stepDelay = 0;
int stepCount = 0;
#define stepsPerRevolution 2038

// Output pins used to control motors
const int lpwmPin = 45; // Left motor speed control
const int rpwmPin = 46; // Right motor speed control
const int ldirPin = 41;
const int rdirPin = 42;

// Distance Calc Variables
long eladist = 0;
long eradist = 0;
float inchPerRev = 8.6079;
float countsPerRev = 716;
float revPer = 0;
float actualCount = 0;

// Motor encoder external interrupt pins
const int elaPin = 12;
const int eraPin = A9;

// SD Card Variables
#define cardSelect 53
#define sclk 52
#define mosi 51

const int eepromAddress = 0;

//  PWM
const int MAX_PWM = 255;
const int MIN_PWM = -255;
// Motor timing
// updated on every loop
unsigned long startTimeR = 0; // start timing R interrupts
unsigned long startTimeL = 0; // start timing L interrupts
volatile long elacount = 0;
volatile long eracount = 0;
// PID
const unsigned long SAMPLE_TIME = 100; // time between PID updates
const unsigned long INT_COUNT = 100;   // sufficient interrupts for accurate timing

// PID Settings - Right Motor
double setpointR = 0; // setpoint for is rotational speed in Hz
double inputR = 0;    // input is PWM to motors
double outputR = 0;   // output is rotational speed in Hz
double KpR = 0.014270374, KiR = 0.09419389, KdR = 0;
PID motorR(&inputR, &outputR, &setpointR, KpR, KiR, KdR, DIRECT);

// PID Settings - LEFT Motor
double setpointL = 0; // setpoint is rotational speed in Hz
double inputL = 0;    // input is PWM to motors
double outputL = 0;   // output is rotational speed in Hz
double KpL = 0.013238366, KiL = 0.08738195, KdL = 0;
PID motorL(&inputL, &outputL, &setpointL, KpL, KiL, KdL, DIRECT);

float dutyPercent = 25.0;
const float MAX_ENCODER_HZ = 3250.0;

double storeR = 0; // used for debug print
double storeL = 0; // used for debug print

void setup()
{
    Serial.begin(115200);
    Dabble.begin(9600);
    initMotors();
    initEncoders();
    initPWM();
    initServos_Stepper();
    stepHome();
}

void loop()
{
    Dabble.processInput();

    if (GamePad.isStartPressed())
    {

        while (!GamePad.isSelectPressed())
        {
            Dabble.processInput();
            if (GamePad.isTrianglePressed())
            {
                dutyPercent += 10.0;
                if (dutyPercent > 100.0)
                    dutyPercent = 100.0;
                Serial.print("DutyPercent: ");
                Serial.println(dutyPercent);
                delay(200); // simple debounce
            }
            if (GamePad.isCrossPressed())
            {
                dutyPercent -= 10.0;
                if (dutyPercent < 0)
                    dutyPercent = 0;
                Serial.print("DutyPercent: ");
                Serial.println(dutyPercent);
                delay(200);
            }
            double targetHz = percentToHz(dutyPercent);

            setpointL = (dutyPercent < 0) ? -targetHz : targetHz;
            setpointR = (dutyPercent < 0) ? -targetHz : targetHz;

            if (GamePad.isUpPressed())
            {
                motorR.Compute();
                motorL.Compute();

                digitalWrite(ldirPin, HIGH);
                digitalWrite(rdirPin, HIGH);
                analogWrite(rpwmPin, outputR);
                analogWrite(lpwmPin, outputL);

                if ((storeR != outputR) || (storeL != outputL))
                {
                    storeR = outputR;
                    storeL = outputL;
                    Serial.println("inputR, inputL, outputR, outputL, errorR, errorL,");
                    Serial.print(inputR);
                    Serial.print("  ");
                    Serial.print(inputL);
                    Serial.print("  ");
                    Serial.print(outputR * 100.0 / 255.0);
                    Serial.print("  ");
                    Serial.print(outputL * 100.0 / 255.0);
                    Serial.print("  ");
                    Serial.print(100 * (setpointR - inputR) / setpointR);
                    Serial.print("  ");
                    Serial.print(100 * (setpointL - inputL) / setpointL);
                    Serial.println("\n");
                }
            }
            else if (GamePad.isDownPressed())
            {
                motorR.Compute();
                motorL.Compute();

                digitalWrite(ldirPin, LOW);
                digitalWrite(rdirPin, LOW);
                analogWrite(rpwmPin, outputR);
                analogWrite(lpwmPin, outputL);

                if ((storeR != outputR) || (storeL != outputL))
                {
                    storeR = outputR;
                    storeL = outputL;
                    Serial.println("inputR, inputL, outputR, outputL, errorR, errorL,");
                    Serial.print(inputR);
                    Serial.print("  ");
                    Serial.print(inputL);
                    Serial.print("  ");
                    Serial.print(outputR * 100.0 / 255.0);
                    Serial.print("  ");
                    Serial.print(outputL * 100.0 / 255.0);
                    Serial.print("  ");
                    Serial.print(100 * (setpointR - inputR) / setpointR);
                    Serial.print("  ");
                    Serial.print(100 * (setpointL - inputL) / setpointL);
                    Serial.println("\n");
                }
            }
            else if (GamePad.isRightPressed())
            {
                digitalWrite(rdirPin, LOW);
                digitalWrite(ldirPin, HIGH);

                analogWrite(rpwmPin, ((dutyPercent / 100) * 255));
                analogWrite(lpwmPin, ((dutyPercent / 100) * 255));
            }
            else if (GamePad.isLeftPressed())
            {
                digitalWrite(rdirPin, HIGH);
                digitalWrite(ldirPin, LOW);

                analogWrite(rpwmPin, ((dutyPercent / 100) * 255));
                analogWrite(lpwmPin, ((dutyPercent / 100) * 255));
            }
            else
            {
                analogWrite(rpwmPin, 0);
                analogWrite(lpwmPin, 0);
                inputL = 0;
                inputR = 0;
                outputL = 0;
                outputR = 0;
                setpointR = 0;
                setpointL = 0;
            }
        }
    }
    // Grip Mode
    else if (GamePad.isSelectPressed())
    {
        analogWrite(rpwmPin, (0));
        analogWrite(lpwmPin, (0));
        gservo.attach(gservoPin);
        rservo.attach(rservoPin);
        bool Pressed = false;

        while (!GamePad.isStartPressed())
        {
            Dabble.processInput();
            if (GamePad.isDownPressed())
            {
                rcurrentAngle -= 5;
                if (rcurrentAngle < 80)
                {
                    rcurrentAngle = 80;
                }
                rservo.write(rcurrentAngle);
                Serial.println(rcurrentAngle);
                delay(50);
            }
            else if (GamePad.isUpPressed())
            {
                rcurrentAngle += 5;
                if (rcurrentAngle > 180)
                {
                    rcurrentAngle = 180;
                }

                rservo.write(rcurrentAngle);
                Serial.println(rcurrentAngle);
                delay(50);
            }
            else if (GamePad.isRightPressed())
            {
                if (stepCount >= 720)
                {
                    stepCount = 720;
                    return;
                }
                PORTA = 0b10000000;
                delayMicroseconds(stepDelay);

                PORTA = 0b01100000;
                delayMicroseconds(stepDelay);

                PORTA = 0b00110000;
                delayMicroseconds(stepDelay);

                PORTA = 0b10010000;
                delayMicroseconds(stepDelay);

                stepCount++;
                Serial.println(stepCount);

                Pressed = true;
            }
            else if (GamePad.isLeftPressed())
            {
                if (stepCount <= -540)
                {
                    stepCount = -540;
                    return;
                }
                PORTA = 0b10010000;
                delayMicroseconds(stepDelay);

                PORTA = 0b00110000;
                delayMicroseconds(stepDelay);

                PORTA = 0b01100000;
                delayMicroseconds(stepDelay);

                PORTA = 0b10000000;
                delayMicroseconds(stepDelay);

                stepCount--;
                Serial.println(stepCount);
                Pressed = true;
            }
            else if (GamePad.isSquarePressed())
            {
                gcurrentAngle += 2;

                if (gcurrentAngle > 180)
                {
                    gcurrentAngle = 180;
                }
                gservo.write(gcurrentAngle);
                Serial.print(gcurrentAngle);
                Serial.print(", ");
                avalue = analogRead(feedbackPin);
                voltage = (avalue * 5) / 1023.0;
                calcAngle = (voltage / slope) - (intercept / slope);
                error = ((int)calcAngle - gcurrentAngle);
                Serial.print(calcAngle);
                Serial.print(", ");

                Serial.print(error);
                Serial.print(", ");

                Serial.print(", ");
                Serial.println(voltage);
                if (abs(error) > 12 && voltage > 0.57)
                {
                    Grip();
                }
                delay(10);
            }
            else if (GamePad.isCirclePressed())
            {
                gcurrentAngle -= 2;

                if (gcurrentAngle < 0)
                {
                    gcurrentAngle = 0;
                }

                gservo.write((gcurrentAngle));
                Serial.print(gcurrentAngle);
                Serial.print(", ");
                avalue = analogRead(feedbackPin);
                voltage = (avalue * 5) / 1023.0;
                calcAngle = (voltage / slope) - (intercept2 / slope);
                error = ((int)calcAngle - gcurrentAngle);
                Serial.print(calcAngle);
                Serial.print(", ");

                Serial.print(error);
                Serial.print(", ");

                Serial.println(voltage);

                delay(10);
            }
            else if (GamePad.isTrianglePressed())
            {
                analogWrite(lpwmPin, 0);
                analogWrite(rpwmPin, 0);

                Serial.println("Triangle Pressed!");
                if (!SD.begin(cardSelect))
                {
                    Serial.println("SD read failed :(");
                    return;
                }
                File file = SD.open("datafile.txt", FILE_READ);

                if (!file)
                {
                    Serial.println("Could not open named file :(");
                    return;
                }
                Serial.println("Reading File:");
                while (file.available())
                {
                    eradist = 0;
                    eladist = 0;
                    dutyPercent = 13;
                    double targetHz = percentToHz(dutyPercent);

                    setpointL = (dutyPercent < 0) ? -targetHz : targetHz;
                    setpointR = (dutyPercent < 0) ? -targetHz : targetHz;
                    String line = file.readStringUntil('\n');
                    line.trim(); // remove spaces and \r
                    if (line.length() == 0)
                        continue; // skip empty lines
                    // find commas
                    int commaIndex = line.indexOf(',');
                    if (commaIndex == -1)
                        continue; // broken line

                    // Extract command and value
                    String command = line.substring(0, commaIndex);
                    command.trim();

                    String valueStr = line.substring(commaIndex + 1);
                    valueStr.trim();

                    // Convert number
                    float value = valueStr.toFloat();

                    Serial.println(command);
                    Serial.println(value);

                    if (command[0] == 'F')
                    {
                        // Forward
                        revPer = value / 8.60796;
                        actualCount = revPer * countsPerRev;

                        while (eladist <= actualCount || eradist <= actualCount)
                        {
                            motorR.Compute();
                            motorL.Compute();
                            digitalWrite(rdirPin, HIGH);
                            digitalWrite(ldirPin, HIGH);

                            analogWrite(rpwmPin, outputR);
                            analogWrite(lpwmPin, (outputL));
                            Serial.print("ela=");
                            Serial.print(eladist);
                            Serial.print(" era=");
                            Serial.println(eradist);
                        }
                        eladist = 0;
                        eradist = 0;
                        revPer = 0;
                        actualCount = 0;
                        actualCount = 0;
                        inputL = 0;
                        inputR = 0;
                        outputL = 0;
                        outputR = 0;
                        setpointR = 0;
                        setpointL = 0;
                        analogWrite(rpwmPin, (0));
                        analogWrite(lpwmPin, (0));
                        delay(150);
                    }
                    if (command[0] == 'R')
                    {
                        if (command[1] == 'I')
                        {
                            // RIGHT
                            double turnRdistance = (value * PI / 180.0) * 5.75;          // theta in radians x radius from one wheel to the other
                            double numRcounts = countsPerRev * turnRdistance / (6 * PI); // need counts per revolution

                            while (eladist <= numRcounts || eradist <= numRcounts)
                            {
                                digitalWrite(rdirPin, LOW);
                                digitalWrite(ldirPin, HIGH);

                                analogWrite(rpwmPin, (0.15 * 255));
                                analogWrite(lpwmPin, (0.15 * 255));
                                Serial.print("numRcounts = ");
                                Serial.print(numRcounts);
                                Serial.print(" ela=");
                                Serial.print(eladist);
                                Serial.print(" era=");
                                Serial.println(eradist);
                            }
                            eladist = 0;
                            eradist = 0;
                            turnRdistance = 0;
                            numRcounts = 0;
                            

                            analogWrite(rpwmPin, (0));
                            analogWrite(lpwmPin, (0));
                            delay(150);
                        }
                        if (command[1] == 'E')
                        {
                            // Reverse
                            revPer = value / 8.60796;
                            actualCount = revPer * countsPerRev;

                            while (eladist <= actualCount || eradist <= actualCount)
                            {
                                motorR.Compute();
                                motorL.Compute();
                                digitalWrite(rdirPin, LOW);
                                digitalWrite(ldirPin, LOW);

                                analogWrite(rpwmPin, outputR);
                                analogWrite(lpwmPin, outputL);
                                Serial.print("ela=");
                                Serial.print(eladist);
                                Serial.print(" era=");
                                Serial.println(eradist);

                                // Use encoders to find number of inches
                            }
                            eladist = 0;
                            eradist = 0;
                            revPer = 0;
                            actualCount = 0;
                            actualCount = 0;
                            inputL = 0;
                            inputR = 0;
                            outputL = 0;
                            outputR = 0;
                            setpointR = 0;
                            setpointL = 0;
                            analogWrite(rpwmPin, (0));
                            analogWrite(lpwmPin, (0));
                            delay(150);
                        }
                    }
                    if (command[0] == 'P')
                    {
                        // PAUSE
                        analogWrite(rpwmPin, 0);
                        analogWrite(lpwmPin, 0);
                        delay(value);
                    }
                    if (command[0] == 'L')
                    {
                        // LEFT
                        double turnRdistance = (value * PI / 180.0) * 5.75;           // theta in radians x radius from one wheel to the other
                        double numRcounts = countsPerRev * turnRdistance / (6 * PI); // need counts per revolution

                        while (eladist <= numRcounts || eradist <= numRcounts)
                        {
                            digitalWrite(rdirPin, HIGH);
                            digitalWrite(ldirPin, LOW);

                            analogWrite(rpwmPin, (0.15 * 255));
                            analogWrite(lpwmPin, (0.15 * 255));
                            Serial.print("ela=");
                            Serial.print(eladist);
                            Serial.print(" era=");
                            Serial.println(eradist);
                        }
                        eladist = 0;
                        eradist = 0;
                        turnRdistance = 0;
                        numRcounts = 0;

                        analogWrite(rpwmPin, (0));
                        analogWrite(lpwmPin, (0));
                        delay(150);
                    }
                }
                analogWrite(rpwmPin, (0));
                analogWrite(lpwmPin, (0));
                gservo.attach(gservoPin);
                rservo.attach(rservoPin);

                file.close();
                dutyPercent = 50;
            }

            // Serial.print('storedCount: ');
            // Serial.print(storedCount);
            // Serial.print(" stepCount: ");
            // Serial.println(stepCount);
            // Serial.println("End of loop");
            else if (Pressed == true)
            {
                Serial.println("trying to save");
                double storedCount;
                EEPROM.get(eepromAddress, storedCount);
                if (storedCount != stepCount)
                {
                    EEPROM.put(eepromAddress, stepCount);
                    Serial.println("SAVED");
                }
                Pressed = false;
            }
        }
    }
}
double percentToHz(float percent)
{
    float p = abs(percent) / 100.0;
    return (double)(p * MAX_ENCODER_HZ);
}

//
//  Move right motor forward.
//

void initMotors()
{
    pinMode(ldirPin, OUTPUT);
    pinMode(rdirPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    pinMode(rpwmPin, OUTPUT);
}
void initEncoders()
{
    pinMode(elaPin, INPUT_PULLUP);
    pinMode(eraPin, INPUT_PULLUP);
    cli();
    PCICR |= 0b00000101;  // Enables Interrupts on PCIINT 0 to 7 and 16 to 23
    PCMSK0 |= 0b01000000; // PCINT6 (Pin 12)
    PCMSK2 |= 0b00000010; // PCINT17 (Pin A9)
    sei();
}

ISR(PCINT0_vect)
{
    // count sufficient interrupts to get accurate timing
    // inputL is the encoder frequency in Hz
    unsigned long t = millis();
    eladist++;
    elacount++;
    if (elacount == INT_COUNT)
    {
        inputL = (float)INT_COUNT * 1000 / (float)(t - startTimeL);
        startTimeL = t;
        elacount = 0;
    }
}

ISR(PCINT2_vect)
{
    // count sufficient interrupts to get accurate timing
    // inputR is the encoder frequency in Hz
    unsigned long t = millis();
    eradist++;
    eracount++;
    if (eracount == INT_COUNT)
    {
        inputR = (float)INT_COUNT * 1000 / (float)(t - startTimeR);
        startTimeR = t;
        eracount = 0;
    }
}
void initPWM()
{
    startTimeR = millis();
    startTimeL = millis();
    motorR.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorL.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorR.SetSampleTime(SAMPLE_TIME);
    motorL.SetSampleTime(SAMPLE_TIME);
    motorR.SetMode(AUTOMATIC);
    motorL.SetMode(AUTOMATIC);
    dutyPercent = 50;
}
void initServos_Stepper()
{
    gservo.attach(gservoPin);
    rservo.attach(rservoPin);
    eservo.attach(eservoPin);
    gservo.write(gcurrentAngle); // 0 degrees is fully open;
    rservo.write(rcurrentAngle); // 90 degrees is straight up;
    eservo.write(ecurrentAngle);
    delay(500);

    DDRA &= 0b11111111;
    PORTA = 0b00000000;

    stepDelay = 2000;
}
void Grip()
{
    grip++;
    Serial.println("Gripped!");
    while (grip == 1)
    {
        gservo.write(((int)calcAngle + 10));
        gcurrentAngle = ((int)calcAngle + 10);
        calcAngle = ((int)calcAngle + 10);
        Serial.println(gcurrentAngle);
        Serial.println("raising arm!");
        delay(500);
        /*
        while(rcurrentAngle > 100)
        {
          rcurrentAngle -= 2;
          rservo.write(rcurrentAngle);
          //Serial.println(rcurrentAngle);
          delay(50);
        }
        */

        rservo.write(120);
        rcurrentAngle = 120;

        delay(500);
        grip = 0;
    }
}

void stepHome()
{
    EEPROM.get(eepromAddress, stepCount);
    Serial.println(stepCount);
    if (stepCount > 0)
    {
        for (int i = 0; i < abs(.75 * stepCount); i++)
        {
            PORTA = 0b10010000;
            delayMicroseconds(stepDelay);

            PORTA = 0b00110000;
            delayMicroseconds(stepDelay);

            PORTA = 0b01100000;
            delayMicroseconds(stepDelay);

            PORTA = 0b10000000;
            delayMicroseconds(stepDelay);
        }
    }
    else if (stepCount < 0)
    {
        for (int i = 0; i < abs(stepCount); i++)
        {
            PORTA = 0b10000000;
            delayMicroseconds(stepDelay);

            PORTA = 0b01100000;
            delayMicroseconds(stepDelay);

            PORTA = 0b00110000;
            delayMicroseconds(stepDelay);

            PORTA = 0b10010000;
            delayMicroseconds(stepDelay);
        }
    }
    stepCount = 0;
    EEPROM.put(eepromAddress, stepCount);
}
