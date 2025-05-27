#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <math.h>

#define SERVO_FREQ 60 // Hz

struct Point
{
  float x;
  float y;
  float z;
};

struct Leg
{
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  float x;
  float y;
  float z;
  int legID;
  bool nextState;
  int state;
};

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

struct Leg leg = {0, 0, 0, 0, 0, 0, 0, false, 0};
float x0;
float y0;
float z0;
float maxTibiaLimit0 = 800;
float maxMicroTibia0 = 2000;
float minMicroTibia0 = 300;
float maxMicroFemur0 = 2000;
float minMicroFemur0 = 300;
float  maxMicroCoxa0 = 2000;
float  minMicroCoxa0 = 300;
struct Leg leg1 = {0, 0, 0, 0, 0, 0, 1, false, 0};
float x1;
float y1;
float z1;
float maxTibiaLimit1 = 800;
float maxMicroTibia1 = 2000;
float minMicroTibia1 = 300;
float maxMicroFemur1 = 2000;
float minMicroFemur1 = 300;
float  maxMicroCoxa1 = 2000;
float  minMicroCoxa1 = 300;
struct Leg leg2 = {0, 0, 0, 0, 0, 0, 2, false, 0};
float x2;
float y2;
float z2;
float maxTibiaLimit2 = 800;
float maxMicroTibia2 = 2000;
float minMicroTibia2 = 300;
float maxMicroFemur2 = 2000;
float minMicroFemur2 = 300;
float  maxMicroCoxa2 = 2000;
float  minMicroCoxa2 = 300;
struct Leg leg3 = {0, 0, 0, 0, 0, 0, 3, false, 0};
float x3;
float y3;
float z3;
float maxTibiaLimit3 = 800;
float maxMicroTibia3 = 2000;
float minMicroTibia3 = 300;
float maxMicroFemur3 = 2000;
float minMicroFemur3 = 300;
float  maxMicroCoxa3 = 2000;
float  minMicroCoxa3 = 300;
struct Leg leg4 = {0, 0, 0, 0, 0, 0, 4, false, 0};
float x4;
float y4;
float z4;
float maxTibiaLimit4 = 800;
float maxMicroTibia4 = 2000;
float minMicroTibia4 = 300;
float maxMicroFemur4 = 2000;
float minMicroFemur4 = 300;
float  maxMicroCoxa4 = 2000;
float  minMicroCoxa4 = 300;
struct Leg leg5 = {0, 0, 0, 0, 0, 0, 5, false, 0};
float x5;
float y5;
float z5;
float maxTibiaLimit5 = 800;
float maxMicroTibia5 = 2000;
float minMicroTibia5 = 300;
float maxMicroFemur5 = 2000;
float minMicroFemur5 = 300;
float  maxMicroCoxa5 = 2000;
float  minMicroCoxa5 = 300;

float speed = 1.7;

bool isFirstMove = true;
int state = 0;
float xOffset = 150;
float yOffset = 120;
float zOffset = 0;
float x = 0;
float y = 0;
float z = 0;
float d1 = 130;
float d2 = 200;

long currentMillis;
long previousMillis;

void setup() {
  Serial.begin(9600);
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);


  delay(10);
}

void loop() {
  static int state;
  if (state == 0)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10)
    {
      previousMillis = currentMillis;
      bool xReached = Interpolate(x0, x0, 0, 0.05 * speed, 1, 0);
      bool yReached = Interpolate(y0, y0, 0, 0.05 * speed, 1, 0);
      bool zReached = Interpolate(z0, -60, 60, 0.005 * speed, 0, 0);
      bool nextState = xReached && yReached && zReached;
      if (nextState)
      {
        state = 1;
      }
    }
  }
  else if (state == 1)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10)
    {
      previousMillis = currentMillis;
      bool xReached = Interpolate(x0, 0, 0, 0.005 * speed, 0, 0);
      static int yState;

      bool yReached;
      if(yState == 0)
      {
        bool nextYState = Interpolate(y0, 0, -80, 0.01 * speed, 0, 0);
        if (nextYState)
        {
          yState = 1;
        }
      }
      else if (yState == 1)
      {
        bool nextYState = Interpolate(y0, -80, 0, 0.01 * speed, 0, 0);
        if (nextYState)
        {
          yState = 0;
          yReached = true;
        }
      }

      bool zReached = Interpolate(z0, 60, -60, 0.005 * speed, 0, 0);
      bool nextState = xReached && yReached && zReached;
      if (nextState)
      {
        yState = 0;
        state = 0;
      }
    }
  }

  leg = IK(x0 + xOffset, y0 + yOffset, z0 + zOffset);
  servoDriver.writeMicroseconds(0, ConvertToMicrosecondsTibia(90, 0)); // leg.tibiaAngle
  servoDriver.writeMicroseconds(0, ConvertToMicrosecondsFemur(90, 0)); // leg.femurAngle
  servoDriver.writeMicroseconds(0, ConvertToMicrosecondsCoxa(90, 0));  // leg.coxaAngle

  leg1 = IK(x1 + xOffset, y1 + yOffset, z1 + zOffset);
  servoDriver.writeMicroseconds(3, ConvertToMicrosecondsTibia(leg.tibiaAngle, 1));
  servoDriver.writeMicroseconds(4, ConvertToMicrosecondsFemur(90 + leg.femurAngle, 1));
  servoDriver.writeMicroseconds(5, ConvertToMicrosecondsCoxa(90 + leg.coxaAngle, 1));
}

Leg IK(float x, float y, float z)
{
  Serial.print(x - xOffset);
  Serial.print(", ");
  Serial.print(y - yOffset);
  Serial.print(", ");
  Serial.println(z - zOffset);

  Leg leg;
  leg.coxaAngle = atan2(z, x) * 180/PI;
  float distance = sqrt(x * x + z * z);
  float phi1 = atan2(y, distance);
  float L = sqrt(distance * distance + y * y);
  if (L < d2 - d1 || L > d1 + d2 || isnan(L))
  {
    Serial.println("Error: Not possible (x, y, z)");
    return;
  }
  float phi2 = acos((L * L + d1 * d1 - d2 * d2) / (2 * L * d1));
  leg.femurAngle = (phi1 - phi2) * 180/PI;
  leg.tibiaAngle = (acos((d2 * d2 + d1 * d1 - L * L) / (2 * d2 * d1))) * 180/PI;
  return leg;
}

bool Interpolate(float& value, float origin, float targetValue, float Lerp, float Range, int curve)
{
  float difference = (targetValue - origin);
  bool targetReached;
  if (difference > 0)
  {
    targetReached = (value >= targetValue + Range) || (value >= targetValue - Range);
  }
  else if (difference < 0)
  {
    targetReached = (value <= targetValue + Range) || (value <= targetValue - Range);
  }
  else
  {
    return true;
  }

  if (targetReached)
  {
    value = targetValue;
    return true;
  }
  else
  {
    float normalizedValue = map(value, origin, targetValue, 0, 1) + Lerp;
    switch (curve)
    {
      case 0:
      break;

      case 1:
      normalizedValue = StartSmooth(normalizedValue);
      break;

      case 2:
      normalizedValue = EndSmooth(normalizedValue);
      break;
    }
    value += difference * normalizedValue;
    return false;
  }
}

void StepTo(Point targetPoint, float height, float stepAmmount, Leg& legToMove)
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) 
  {
    previousMillis = currentMillis;
    Serial.print("WTFMAN");
    float upperLimit = 2 * height;

    static float t;
  	float tStep = upperLimit / stepAmmount;

    Point nextPoint = StepCurve(t, targetPoint.x, targetPoint.y, height);

    legToMove = IK(nextPoint.x + x + xOffset, nextPoint.y + y + yOffset, nextPoint.z + z + zOffset);
    servoDriver.writeMicroseconds(10, ConvertToMicrosecondsTibia(legToMove.tibiaAngle, 0));
    servoDriver.writeMicroseconds(9, ConvertToMicrosecondsFemur(90 +legToMove.femurAngle, 0));
    servoDriver.writeMicroseconds(8, ConvertToMicrosecondsCoxa(90 + legToMove.coxaAngle, 0));

    t += tStep;
    if (t >= upperLimit)
    {
      legToMove.nextState = true;
      t = 0;
    }
  }
}

Point RotatePoint(float x, float y, float z, float xOrigin, float yOrigin, float zOrigin, char axis, float angle)
{
  angle = angle * PI/180;
  Point translatedPoint;
  Point pointF;
  if (axis == 'Z')
  {
    translatedPoint.x = (x - xOrigin);
    translatedPoint.y = (y - yOrigin);
    pointF.x = (translatedPoint.x * cos(angle)) - (translatedPoint.y * sin(angle)) + xOrigin;
    pointF.y = (translatedPoint.y * cos(angle)) + (translatedPoint.x * sin(angle)) + yOrigin;
    pointF.z = z;
    return pointF;
  }
  else if (axis == 'X')
  {
    translatedPoint.x = (z - zOrigin);
    translatedPoint.y = (y - yOrigin);
    pointF.x = x;
    pointF.y = (translatedPoint.y * cos(angle)) + (translatedPoint.x * sin(angle)) + yOrigin;
    pointF.z = (translatedPoint.x * cos(angle)) - (translatedPoint.y * sin(angle)) + zOrigin;
    return pointF;
  }
  else if (axis == 'Y')
  {
    translatedPoint.x = (z - zOrigin);
    translatedPoint.y = (x - xOrigin);
    pointF.x = (translatedPoint.y * cos(angle)) + (translatedPoint.x * sin(angle)) + xOrigin;
    pointF.y = y;
    pointF.z = (translatedPoint.x * cos(angle)) - (translatedPoint.y * sin(angle)) + zOrigin;
    return pointF;
  }
}

float lerp(float a, float b, float t)
{
  return (b - a) * constrain(t, 0, 1);
}

int ConvertToMicrosecondsCoxa(float degrees, int whatLeg)
{
  switch (whatLeg)
  {
    case 0:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa0, maxMicroCoxa0);
    case 1:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa1, maxMicroCoxa1);
    case 2:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa2, maxMicroCoxa2);
    case 3:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa3, maxMicroCoxa3);
    case 4:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa4, maxMicroCoxa4);
    case 5:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroCoxa5, maxMicroCoxa5);
  }
}

int ConvertToMicrosecondsFemur(float degrees, int whatLeg)
{
  switch (whatLeg)
  {
    case 0:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur0, maxMicroFemur0);
    case 1:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur1, maxMicroFemur1);
    case 2:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur2, maxMicroFemur2);
    case 3:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur3, maxMicroFemur3);
    case 4:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur4, maxMicroFemur4);
    case 5:
    degrees = constrain(degrees, 0, 180);
    return map(degrees, 0, 180, minMicroFemur5, maxMicroFemur5);
  }
}

int ConvertToMicrosecondsTibia(float degrees, int whatLeg)
{
  int microseconds;
  switch (whatLeg)
  {
    case 0:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia0, maxMicroTibia0);
    if (microseconds < maxTibiaLimit0)
    {
      return maxTibiaLimit0;
    }
    else
    {
      return microseconds;
    }
    case 1:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia1, maxMicroTibia1);
    if (microseconds < maxTibiaLimit1)
    {
      return maxTibiaLimit1;
    }
    else
    {
      return microseconds;
    }
    case 2:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia2, maxMicroTibia2);
    if (microseconds < maxTibiaLimit2)
    {
      return maxTibiaLimit2;
    }
    else
    {
      return microseconds;
    }
    case 3:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia3, maxMicroTibia3);
    if (microseconds < maxTibiaLimit3)
    {
      return maxTibiaLimit3;
    }
    else
    {
      return microseconds;
    }
    case 4:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia4, maxMicroTibia4);
    if (microseconds < maxTibiaLimit4)
    {
      return maxTibiaLimit4;
    }
    else
    {
      return microseconds;
    }
    case 5:
    degrees = constrain(degrees, 0, 180);
    microseconds = map(degrees, 0, 180, minMicroTibia5, maxMicroTibia5);
    if (microseconds < maxTibiaLimit5)
    {
      return maxTibiaLimit5;
    }
    else
    {
      return microseconds;
    }
  }
}

float StartSmooth(float t)
{
  return t * t;
}

float EndSmooth(float t)
{
  return sqrt(t);
}

Point StepCurve(float t, float x, float y, float h)
{
  Point funcPoint;
  funcPoint.z = (x / (2 * h)) * t;
  funcPoint.y = sqrt(h * h - ((t - h) * (t = h)));
  funcPoint.x = (y / (2 * h)) * t;
  return funcPoint;
}

Point SemiCircle(float r, float t)
{
  Point funcPoint;
  funcPoint.z = t;
  funcPoint.y = sqrt((r * r) - ((t - r) * (t - r)));
  funcPoint.x = 0;
  return funcPoint;
}

Point Swoop(float t)
{
  Point funcPoint;
  funcPoint.z = t;
  funcPoint.y = ((t - 50) * (t - 50) * (t - 50) + (50 * ((t - 50) * (t - 50)))) / 500;
  funcPoint.x = 0;
  return funcPoint;
}