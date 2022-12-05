#include <stdlib.h>
#include <soc/rtc_wdt.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include "definition_utils.h"
#define CONFIG_DISABLE_HAL_LOCKS
#define EEPROM_SIZE 16
#include <HardwareSerial.h>
#include "calc_utils.h"
#include "controllerFunction.h"

const int en = 21;
bool ignoreLimits = false;
const int xS = 4, xD = 16, xL = 34;
const float A = 3203.18697341;
int xSp = 10;
double xLm;
axis X = {xS, xD, xL, 10, 10, 45, A, (1 / A), 0, 0, 0};
fTB oX, sXm, sXd, sXw;
float cmdX, xI, nX, iPosX, routineX;
long storeI;
volatile bool xLim = false;

const int yS = 18, yD = 19, yL = 39;
const float B = 3201.59756096;
int ySp = 10;
double yLm;
axis Y = {yS, yD, yL, 10, 10, 45, B, (1 / B), 0, 0, 0};
fTB oY, sYm, sYd, sYw;
float  cmdY, yI, nY, iPosY, routineY;
long storeO;
volatile bool yLim = false;

const int zS = 17, zD = 5, zL = 36;
const float C = 3201.95086891;
int zSp = 15;
double zLm;
volatile bool zLim = false;
//Z axis drw speed is used as drilling speed
axis Z = {zS, zD, zL, 15, 15, 45, C, (1 / C), 0, 0, 0};
unsigned long focusHeight = 52513, drawHeight = 0, cutHeight = 0, plungeDepth = 0 , prePlungeHeight = 0 , preDotHeight = 0,
              dotDepth = 0, millStartHeight = 0 , millRunHeight = 0 , preDrillHeight = 0 , postDrillHeight = 0;
iTB toolT;
//toolT. 0: No tool. 1: Mill Bit. 2: Drill Bit. 3: V Bit
double spinOffX = -.70, spinOffY = 42.149899195;
fTB toolD;
fTB sZm, sZd, sZw;
float cmdZ;
long storeR;

const int spn = 26;
const int spnEn = 14;
int spindleSpeed = 80;
int cutPower = 80;
bool spindleOn = false;
int rampVal = 0;
tool spindle = tool(spn, spnEn, 8000, 6, true, cutPower);
double spindleBitLength;

const int lzr = 23;
int lzrPwr = 30;
int drawPower = 30;
tool laser(lzr, 5000, 8, false, lzrPwr);


const int vac = 33;
tool vacuum(vac, true);

const int probePin = 32;
toolProbe probe(probePin, true);


bool drawNodes = false, ack = false, taskRunning = false, stepsDone = false, fire = false, readingNodes = false, preDot = false;
bool mir = false, iPosSet = false, widenNodes = false, millNodes = false, isDrilling = false;
String s, q, waiting;
char h;
//-1: Not running routine. 0: Routine started. 1: Pads. 2: Traces. 3: Drills. 4: Vias.
//0: Standard Draw Mode. 1: Cut Mode. 2: Raster Mode.
int routineMode = -1, operationMode = 0;
volatile double circlePart = 0;
volatile int fillPart = 0;
volatile bool xToLim = false, yToLim = false, zToLim = false, relCircle = false, probing = false;
float rd, theta, tW;
double tWid[150], traceWidth = 0;

vector2 nodes[150];
vector2 v1, v2, v3, v4, p1, p2;
TaskHandle_t circleHandle = NULL;
float DtoR = ((2 * PI) / 360);

uTb xPos, yPos, zPos;
fTB xSM, ySM, zSM;
iTB LZPW, SPPW;
byte xByte, yByte, zByte;

struct dataOut datO;

float platformH = 35.456663;

float xGap = 283.6230, yGap = 185.3385, zGap = 34.909194, probeComp = .8012;
//focal height above material
float focalBelow = 19.906641, focalAbove = 15;

bool debugPS4 = false;
bool controlPS4 = false;

unsigned long xC, yC, zC;

void IRAM_ATTR xLimit()
{
  xLim = true;
}

void IRAM_ATTR yLimit()
{
  yLim = true;
}

void IRAM_ATTR zLimit()
{
  zLim = true;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(74880);
  Serial.println();
  vacuum.toolInit();
  spindle.toolInit();
  laser.toolInit();
  probe.initToolProbe();
  pinMode(en, OUTPUT);
  digitalWrite(en, HIGH);
  X.initAxis();
  Y.initAxis();
  Z.initAxis();

  EEPROM.begin(EEPROM_SIZE);

  for (int n = 0; n < 4; n++)
  {
    oX.b[n] = EEPROM.read(n);
    oY.b[n] = EEPROM.read(n + 4);
    toolD.b[n] = EEPROM.read(n + 8);

    if (n < 2)
    {
      toolT.b[n] = EEPROM.read(n + 12);
    }
  }

  if (toolT.i == 0 || toolD.f == 0)
  {
    toolT.i = 0;
    toolD.f = 0;
  }
  ack = false;
  routineMode = -1;
  xSM.f = X.A;
  ySM.f = Y.A;
  zSM.f = Z.A;
  attachInterrupt(xL, xLimit, RISING);
  attachInterrupt(yL, yLimit, RISING);
  attachInterrupt(zL, zLimit, RISING);
}

void loop() {

  if (!taskRunning && !readingNodes)
  {
    if (ack)
    {
      ack = false;
      Serial.println("@K$");
    }

    if (Serial.available())
    {
      serialEvents();
    }

    if (X.stepRead() == HIGH)
    {
      X.stepLow();
    }

    if (Y.stepRead() == HIGH)
    {
      Y.stepLow();
    }

    if (Z.stepRead() == HIGH)
    {
      Z.stepLow();
    }

  }
  else if (readingNodes && taskRunning)
  {
    serialNodes();
  }

  if (debugPS4)
  {
    debugController();
  }
  else if (controlPS4)
  {
    if (controllerInput())
    {
      if (i != 0 || o != 0 || r != 0)
      {
        opStep();
      }
      else
      {
        homingRun(true);
      }
    }
  }
}

void limitMotion(int i)
{
  // 0 = x-, 1 = x+, 2 = y-, 3 = y+, 4 = z-

  esp_task_wdt_init(720, false);

  if (i == 0)
  {
    X.dirLow();
    while (digitalRead(xL) == LOW)
    {
      X.stepHigh();
      delayMicroseconds(xSp);
      X.stepLow();
      delayMicroseconds(xSp);
    }

    X.dirHigh();
    while (digitalRead(xL) == HIGH)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 4);
      X.stepLow();
      delayMicroseconds(xSp * 4);
    }

    X.dirLow();
    while (digitalRead(xL) == LOW)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 16);
      X.stepLow();
      delayMicroseconds(xSp * 16);
    }

    X.dirHigh();
    while (digitalRead(xL) == HIGH)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 16);
      X.stepLow();
      delayMicroseconds(xSp * 16);
    }
    X.absPos = 0;
  }
  else if (i == 1)
  {
    X.dirHigh();
    while (digitalRead(xL) == LOW)
    {
      X.stepHigh();
      delayMicroseconds(xSp);
      X.stepLow();
      delayMicroseconds(xSp);
    }

    X.dirLow();
    while (digitalRead(xL) == HIGH)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 4);
      X.stepLow();
      delayMicroseconds(xSp * 4);
    }

    X.dirHigh();
    while (digitalRead(xL) == LOW)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 4);
      X.stepLow();
      delayMicroseconds(xSp * 4);
    }

    X.dirLow();
    while (digitalRead(xL) == HIGH)
    {
      X.stepHigh();
      delayMicroseconds(xSp * 4);
      X.stepLow();
      delayMicroseconds(xSp * 4);
    }
  }
  else if (i == 2)
  {
    Y.dirLow();
    while (digitalRead(yL) == LOW)
    {
      Y.stepHigh();
      delayMicroseconds(ySp);
      Y.stepLow();
      delayMicroseconds(ySp);
    }

    Y.dirHigh();
    while (digitalRead(yL) == HIGH)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 4);
      Y.stepLow();
      delayMicroseconds(ySp * 4);
    }

    Y.dirLow();
    while (digitalRead(yL) == LOW)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 16);
      Y.stepLow();
      delayMicroseconds(ySp * 16);
    }

    Y.dirHigh();
    while (digitalRead(yL) == HIGH)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 16);
      Y.stepLow();
      delayMicroseconds(ySp * 16);
    }
    Y.absPos = 0;
  }
  else if (i == 3)
  {
    Y.dirHigh();
    while (digitalRead(yL) == LOW)
    {
      Y.stepHigh();
      delayMicroseconds(ySp);
      Y.stepLow();
      delayMicroseconds(ySp);
    }

    Y.dirLow();
    while (digitalRead(yL) == HIGH)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 4);
      Y.stepLow();
      delayMicroseconds(ySp * 4);
    }

    Y.dirHigh();
    while (digitalRead(yL) == LOW)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 4);
      Y.stepLow();
      delayMicroseconds(ySp * 4);
    }

    Y.dirLow();
    while (digitalRead(yL) == HIGH)
    {
      Y.stepHigh();
      delayMicroseconds(ySp * 4);
      Y.stepLow();
      delayMicroseconds(ySp * 4);
    }
  }
  else if (i == 4)
  {
    Z.dirLow();
    while (digitalRead(zL) == LOW)
    {
      Z.stepHigh();
      delayMicroseconds(zSp);
      Z.stepLow();
      delayMicroseconds(zSp);
    }

    Z.dirHigh();
    while (digitalRead(zL) == HIGH)
    {
      Z.stepHigh();
      delayMicroseconds(zSp * 2);
      Z.stepLow();
      delayMicroseconds(zSp * 2);
    }

    Z.dirLow();
    while (digitalRead(zL) == LOW)
    {
      Z.stepHigh();
      delayMicroseconds(zSp * 4);
      Z.stepLow();
      delayMicroseconds(zSp * 4);
    }

    Z.dirHigh();
    while (digitalRead(zL) == HIGH)
    {
      Z.stepHigh();
      delayMicroseconds(zSp * 8);
      Z.stepLow();
      delayMicroseconds(zSp * 8);
    }
    Z.absPos = 0;
  }
}

void serialNodes()
{
  while (readingNodes)
  {
    if (Serial.available())
    {
      h = Serial.read();

      s += h;

      if (h == '/')
      {
        if (s.indexOf("eT/") > 0 || s == "eT/")
        {
          readingNodes = false;
          drawNodes = true;
          stepsDone = false;
          taskRunning = true;
          xTaskCreate(nodeLogic, "Node Logic", 10000, NULL, 4, NULL);
        }
        else
        {
          Serial.println("@K$");
        }
      }
    }
  }
}

void serialEvents()
{
  if (Serial.available())
  {
    h = Serial.read();

    if (h == '/')
    {
      if (s != "")
      {
        cX = -100000;
        cY = -100000;
        cZ = -100000;
        cW = -100000;
        cH = -100000;
        cR = -100000;
        cU = -100000;
        //Serial.println("Found Command");
        processCommand(s);
        s = "";
      }
    }
    else
    {
      if (h != '\n' && h != '\r' && h != ' ')
      {
        s += h;
      }
    }
  }
}

void homingRun(bool enSet)
{
  esp_task_wdt_init(720, false);
  digitalWrite(en, LOW);
  fire = false;
  laser.Off();
  spindle.Off();
  vacuum.Off();
  xSp = X.mov;
  ySp = Y.mov;
  zSp = Z.mov;
  bool zLimVerified = false;
  if (digitalRead(zL) == HIGH)
  {
    i = 0;
    o = 0;
    r = 5 * Z.A;
    opStep();
  }
  limitMotion(4);
  vTaskDelay(1);
  //Serial.println("Zlim verified");
  i = 5000;
  o = 5000;
  r = 0;
  opStep();
  vTaskDelay(1);
  //Serial.println("Y and X out from limit");
  limitMotion(0);
  vTaskDelay(1);
  //Serial.println("X returned to limit");
  limitMotion(2);
  vTaskDelay(1);
  //Serial.println("Y returned to limit");
  //Serial.println("Read Union Values. oX:" + String(oX.f, DEC) + ", oY:" + String(oY.f, DEC) + ", Proof X:" + String(cX, DEC) + ", Y:" + String(cY, DEC));
  i = oX.f * A;
  o = oY.f * B;
  r = 0;
  fire = false;
  stepsDone = false;
  vTaskDelay(1);
  //Serial.println("Move to initial offset point. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC));
  opStep();

  while (!stepsDone)
  {
    vTaskDelay(10);
  }

  X.absPos = 0;
  Y.absPos = 0;
  Z.absPos = 0;
  X.relPos = 0;
  Y.relPos = 0;
  Z.relPos = 0;
  laser.Off();
  spindle.Off();
  vacuum.Off();
  if (enSet)
  {
    stepsDone = false;
    taskRunning = false;
    digitalWrite(en, HIGH);
  }
}

void processCommand(String u)
{
  //Check for Simple Commands First
  //Serial.println("Processing Command: " + u);
  if (u.indexOf('/') > 0)
  {
    u = u.substring(0, u.indexOf('/') - 1);
  }

  if (u[0] == '?')
  {
    ack = true;
    return;
  }
  else if (u == "CONTROLLER")
  {
    if (!checkController())
    {
      connectController();
    }
    else
    {
      Serial.println("Controller Connected. Ready For Manual Control");
      controlPS4 = !controlPS4;
    }
    ack = true;
    return;
  }
  else if (u == "DEBUGCONTROLLER")
  {
    debugPS4 = !debugPS4;
    ack = true;
    return;
  }
  else if (u == "DRAW")
  {
    lzrPwr = drawPower;
    focusHeight = drawHeight;
    operationMode = 0;
    ack = true;
    return;
  }
  else if (u == "CUT")
  {
    lzrPwr = cutPower;
    focusHeight = cutHeight;
    operationMode = 1;
    ack = true;
    return;
  }
  else if (u == "START")
  {
    //Serial.println("Routine Start Command Received");
    routineMode = 0;
    ack = true;
    return;
  }
  else if (u == "PADS")
  {
    routineMode = 1;
    ack = true;
    //Serial.println("Routine Pads Mode Command Received. rX:" + String(routineX, DEC) + ", rY:" + String(routineY, DEC) + ", mode:" + String(routineMode, DEC));
    return;
  }
  else if (u == "TRACES")
  {
    routineMode = 2;
    //Serial.println("Routine Traces Command Received");
    homingRun(false);
    //Serial.println("Homed.");
    if (mir)
    {
      mir = false;
      cX = routineX;
      cY = routineY;
      //iPosX = bW;
      //Serial.println("CX:" + String(cX, DEC) + ", CY:" + String(cY, DEC));
      destinationMath(false);
      opStep();
      mir = true;
    }
    else
    {
      cX = routineX;
      cY = routineY;
      destinationMath(false);
      opStep();
    }
    X.absPos = 0;
    Y.absPos = 0;
    X.relPos = 0;
    Y.relPos = 0;
    X.relZero = 0;
    Y.relZero = 0;
    //Serial.println("Moved to routine X, Y");
    ack = true;
    return;
  }
  else if (u == "DRILLS" || u == "DRILLING")
  {
    isDrilling = true;
    ack = true;
    return;
  }
  else if (u == "VIAS")
  {
    routineMode = 4;
    //Serial.println("Routine Vias Command Received");
    homingRun(false);
    //Serial.println("Homed.");
    if (mir)
    {
      mir = false;
      cX = routineX;
      cY = routineY;
      destinationMath(false);
      opStep();
      mir = true;
    }
    else
    {
      cX = routineX;
      cY = routineY;
      destinationMath(false);
      opStep();
    }
    X.absPos = 0;
    Y.absPos = 0;
    X.relPos = 0;
    Y.relPos = 0;
    X.relZero = 0;
    Y.relZero = 0;
    //Serial.println("Moved to routine X, Y");
    ack = true;
    return;
  }
  else if (u == "STATES")
  {
    Serial.println("States:");
    Serial.println("X: L:" + String(digitalRead(xL), DEC) + ", ST:" + String(digitalRead(xS), DEC) + ", DR:" + String(digitalRead(xD), DEC));
    Serial.println("Y: L:" + String(digitalRead(yL), DEC) + ", ST:" + String(digitalRead(yS), DEC) + ", DR:" + String(digitalRead(yD), DEC));
    Serial.println("Z: L:" + String(digitalRead(zL), DEC) + ", ST:" + String(digitalRead(zS), DEC) + ", DR:" + String(digitalRead(zD), DEC));
    Serial.println("EN:" + String(digitalRead(en), DEC) + ", LZ:" + String(digitalRead(lzr), DEC));
    Serial.println("SPN:" + String(digitalRead(spn), DEC));
    ack = true;
    return;
  }
  else if (u.indexOf("GAP") >= 0)
  {
    if (u == "GAPX")
    {
      measureGap(0);
      ack = true;
      return;
    }
    else if (u == "GAPY")
    {
      measureGap(1);
      ack = true;
      return;
    }
  }
  else if (u == "ALLPINS")
  {
    for (int n = 2; n < 40; n++)
    {
      Serial.println("Pin #:" + String(n, DEC) + ", STATE:" + String(digitalRead(n), DEC));
    }
    ack = true;
    return;
  }
  else if (u == "MIR" || u == "mir")
  {
    mir = true;
    //iPosX = X.absPos / X.A;
    //iPosY = Y.absPos / Y.A;
    //float nIX = 2 * (49.65 - iPosX);
    //iPosX = nIX;
    ack = true;
    //Serial.println("Mirroring ON!");
    return;
  }
  else if (u == "POSASBINARY")
  {
    Serial.println("X as binary: " + String(X.absPos, BIN));
    return;
  }
  else if (u == "LEFTSHIFTPOS")
  {
    unsigned long lPos = X.absPos;
    const unsigned long lsh = 1;
    lPos = lPos << 1;
    Serial.println("X as binary left shifted: " + String(lPos, BIN));
    return;
  }
  else if (u == "RIGHTSHIFTPOS")
  {
    unsigned long lPos = X.absPos;
    const unsigned long lsh = 1;
    lPos = lPos >> 1;
    Serial.println("X as binary right shifted: " + String(lPos, BIN));
    return;
  }
  //Home
  else if (u == "O" || u == "o")
  {
    mir = false;
    taskRunning = true;
    ack = true;
    xSp = X.mov;
    ySp = Y.mov;
    zSp = Z.mov;
    iPosX = 0;
    iPosY = 0;
    iPosSet = false;
    homingRun(true);
    return;
  }
  //Set/Clear Relative Zero
  else if (u == "ZR" || u == "zr")
  {
    if (X.relZero != 0 || Y.relZero != 0 || Z.relZero != 0)
    {
      X.relZero = 0;
      Y.relZero = 0;
      Y.relZero = 0;
      X.relPos = X.absPos;
      Y.relPos = Y.absPos;
      Z.relPos = Z.absPos;
    }
    else
    {
      X.relZero = X.absPos;
      Y.relZero = Y.absPos;
      Z.relZero = Z.absPos;
      X.relPos = 0;
      Y.relPos = 0;
      Z.relPos = 0;
    }
    ack = true;
    return;
  }
  //Set Absolute Zero
  else if (u == "ZA" || u == "za")
  {
    digitalWrite(en, LOW);
    X.absPos = 0;
    Y.absPos = 0;
    X.relPos = 0;
    Y.relPos = 0;
    X.relZero = 0;
    Y.relZero = 0;
    ack = true;
    return;
  }
  else if (u == "EN" || u == "en")
  {
    if (digitalRead(en) == HIGH)
    {
      digitalWrite(en, LOW);
    }
    else
    {
      digitalWrite(en, HIGH);
    }
    ack = true;
    return;
  }
  //Go To Limits
  else if (u.indexOf('+') >= 0 || (u.indexOf('-') >= 0 && u.indexOf('l') >= 0))
  {
    u.toLowerCase();
    if (u == "xl+" || u == "xl-")
    {
      xToLim = true;
      taskRunning = true;
      if (u == "xl+")
      {
        limitMotion(1);
      }
      else
      {
        limitMotion(0);
      }

      ack = true;
      return;
    }
    else if (u == "yl+" || u == "yl-")
    {
      yToLim = true;
      taskRunning = true;
      if (u == "yl+")
      {
        limitMotion(3);
      }
      else
      {
        limitMotion(2);
      }

      ack = true;
      return;
    }
    else if (u == "zl+" || u == "zl-")
    {
      zToLim = true;
      taskRunning = true;
      if (u == "zl+")
      {

      }
      else
      {
        limitMotion(4);
      }

      ack = true;
      return;
    }
  }
  //Manually Fire Laser
  else if (u == "LZR" || u == "lzr")
  {

    if (drawNodes)
    {
      drawNodes = false;
    }
    else
    {
      drawNodes = true;
    }

    if (laser.on)
    {
      laser.Off();
    }
    else
    {
      movetoHeight(focusHeight);
      laser.On();
    }
    ack = true;
    return;
  }
  //Manually Fire Spindle
  else if (u.indexOf("SPN") >= 0 || u.indexOf("spn") >= 0)
  {
    int aVal = 0;
    for (int n = 0; n < u.length(); n++)
    {
      if (isDigit(u[n]))
      {
        u = u.substring(n);
        break;
      }
    }

    aVal = u.toInt();

    if (aVal > 0)
    {
      spindleOn = true;
      //Serial.println("Sent Spindle On. Value:" + String(aVal, DEC));
      spindle.On(aVal);
    }
    else
    {
      vacuum.Off();
      spindle.Off();
      spindleOn = false;
      //Serial.println("Sent Spindle Off");
    }
    rampVal = aVal;

    ack = true;
    return;
  }
  //Manually Fire Spindle
  else if (u.indexOf("LAZ") >= 0 || u.indexOf("laz") >= 0)
  {
    int aVal = 0;
    for (int n = 0; n < u.length(); n++)
    {
      if (isDigit(u[n]))
      {
        u = u.substring(n);
        break;
      }
    }

    aVal = u.toInt();

    if (aVal > 0)
    {
      //Serial.println("Sent Lazer On. Value:" + String(aVal, DEC));
      laser.On(aVal);
    }
    else
    {
      laser.Off();
      //Serial.println("Sent lazer Off");
    }

    ack = true;
    return;
  }
  else if (u == "seeDrill")
  {
    i = X.A * spinOffX;
    o = Y.A * spinOffY;
    r = 0;
    opStep();

    movetoHeight(prePlungeHeight);

    ack = true;
    return;
  }
  else if (u == "T" || u == "t")
  {
    //Output Absolute Position Steps
    if (u == "T")
    {
      q = "T" + String(X.absPos, DEC) + "," + String(Y.absPos, DEC) + "," + String(Z.absPos, DEC) + "/";
    }
    else
      //Output Relative Position Steps
    {
      q = "t" + String(X.relPos, DEC) + "," + String(Y.relPos, DEC) + "," + String(Z.relPos, DEC) + "/";
    }
    Serial.println(q);
    return;
  }
  else if (u == "I" || u == "i")
  {
    //Output Absolute Position MM
    if (u == "I")
    {
      q = "I" + String(X.absPos * X.iA, DEC) + "," + String(Y.absPos * Y.iA, DEC) + "," + String(Z.absPos * Z.iA, DEC) + "/";
    }
    else
      //Output Relative Position MM
    {
      q = "i" + String(X.relPos * X.iA, DEC) + "," + String(Y.relPos * Y.iA, DEC) + "," + String(Z.relPos * Z.iA, DEC) + "/";
    }
    Serial.println(q);
    return;
  }
  else if (u.indexOf("rT") >= 0 || u.indexOf("rM") >= 0)
  {
    if (u == "rT")
    {
      movetoHeight(focusHeight);
      widenNodes = true;
      taskRunning = true;
      readingNodes = true;
      drawNodes = false;
      Serial.println("@K$");
    }
    else
    {
      movetoHeight(focusHeight);
      drawNodes = true;
      ack = true;
      parseValues(u);
      if (cR != -100000)
      {
        traceWidth = cR;
      }
    }
    return;
  }
  else if (u == "eT")
  {
    drawNodes = false;
    ledcWrite(8, 0);
    ack = true;
    return;
  }
  else if (u == "eM")
  {
    drawNodes = false;
    digitalWrite(spn, LOW);
    ack = true;
    return;
  }
  else if (u == "eO")
  {
    //End of Module
    modRot = -100000;
    padRot = -100000;
    X.relZero = 0;
    Y.relZero = 0;
    Y.relZero = 0;
    X.relPos = X.absPos;
    Y.relPos = Y.absPos;
    Z.relPos = Z.absPos;
    ack = true;
    return;
  }
  else if (u == "END" || u == "end")
  {
    ack = true;
    focusHeight = drawHeight;
    xSp = X.mov;
    ySp = Y.mov;
    routineMode = -1;
    operationMode = 0;
    routineX = 0;
    routineY = 0;
    iPosX = 0;
    iPosY = 0;

    if (isDrilling)
    {
      spindleOn = false;

      spindle.Off();
      vacuum.Off();
      isDrilling = false;
    }
    iPosSet = false;
    fire = false;
    stepsDone = false;
    digitalWrite(22, LOW);
    ledcWrite(8, 0);
    homingRun(true);
    digitalWrite(en, HIGH);
    vTaskDelay(30);
    taskRunning = false;
    return;
  }
  else if (u == "getOffset")
  {
    Serial.println("X:" + String(oX.f, DEC) + ", Y:" + String(oY.f, DEC));
    return;
  }
  else if (u == "loadTool")
  {
    for (int n = 0; n < 4; n++)
    {
      toolD.b[n] = EEPROM.read(n + 8);

      if (n < 2)
      {
        toolT.b[n] = EEPROM.read(n + 12);
      }
    }

    if (toolT.i == 0 || toolD.f == 0)
    {
      toolT.i = 0;
      toolD.f = 0;
    }

    //Serial.println("Loaded tool. Type:" + String(toolT.i, DEC) + ", Diameter:" + String(toolD.f, DEC));
    s = "";
    ack = true;
  }
  else if (u == "getTool")
  {
    //Serial.println("Tool Type:" + String(toolT.i, DEC) + ", Diameter:" + String(toolD.f, DEC));
    s = "";
    ack = true;
  }
  else
  {
    //Check for Medium
    char cmd = u[0];
    parseValues(u);
    if (cmd == 'm' || cmd == 'M' && u.indexOf("MAP") < 0)
    {
      if (routineMode == 0)
      {
        if (!mir)
        {
          routineX = cX;
          routineY = cY;
        }
      }

      if (cmd == 'M')
      {
        destinationMath(false);
        //Serial.println("Absolute Movement Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }
      else
      {
        destinationMath(true);
        //Serial.println("Relative Movement Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }
      xSp = X.mov;
      ySp = Y.mov;
      fire = false;
      taskRunning = true;
      stepsDone = false;
      opStep();
      while (i != 0 || o != 0 || r != 0)
      {
        vTaskDelay(10);
      }
      taskRunning = false;
      ack = true;
      return;
    }
    else if (u.indexOf("Cut") >= 0)
    {
      //Serial.println("cut command");
      if (cX != -100000 && cY != -100000)
      {
        if (cW != -100000 && cH != -100000)
        {
          Serial.println("border cut");
          taskRunning = true;
          ack = true;
          xTaskCreate(borderCut, "Border Cut", 10000, NULL, 4, NULL);
        }
        else
        {
          Serial.println("point to point cut");
          taskRunning = true;
          ack = true;
          xTaskCreate(pointCut, "Point Cut", 10000, NULL, 4, NULL);
        }
      }
      return;
    }
    else if (u.indexOf("traces") >= 0)
    {
      if (cW != -100000)
      {
        tW = cW;
      }
      ack = true;
      return;
    }
    else if (cmd == 'd' || cmd == 'D')
    {
      if (cmd == 'D')
      {
        destinationMath(false);
        //Serial.println("Absolute Draw Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }
      else
      {
        destinationMath(true);
        //Serial.println("Relative Draw Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }

      movetoHeight(focusHeight);

      xSp = X.drw;
      ySp = Y.drw;
      fire = true;
      taskRunning = true;
      stepsDone = false;
      opStep();
      while (i != 0 || o != 0)
      {
        vTaskDelay(10);
      }
      taskRunning = false;
      ack = true;
      return;
    }
    else if (cmd == 'c' || cmd == 'C')
    {
      if (cmd == 'C')
      {
        relCircle = false;
        destinationMath(false);
        //Serial.println("Absolute Circle Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }
      else
      {
        destinationMath(true);
        relCircle = true;
        //Serial.println("Relative Circle Call. I:" + String(i, DEC) + ", o:" + String(o, DEC) + ", r:" + String(r, DEC));
      }

      if (cR != -100000)
      {
        movetoHeight(focusHeight);

        xSp = X.drw;
        ySp = Y.drw;
        fire = true;
        taskRunning = true;
        xTaskCreate(circularPath, "Circle", 10000, NULL, 4, NULL);
      }
      return;
    }
    else if (cmd == 'B' || cmd == 'b')
    {
      if (mir)
      {
        bW = cW;
        //if (!iPosSet)
        //{
        //i = X.A * (iPosX - bW);
        //o = 0;
        // r = 0;
        //routineX = iPosX - bW;
        //routineY = iPosY;
        //opStep();
        //iPosSet = true;
        //}
        //Serial.println("Mirror. Stored Border Value:" + String(bW, DEC));
      }
      destinationMath(false);

      movetoHeight(focusHeight);

      xSp = X.drw;
      ySp = Y.drw;
      xTaskCreate(drawBorder, "Draw Border", 10000, NULL, 4, NULL);
      ack = true;
      taskRunning = true;
      return;

    }
    else if (u.indexOf("rO") >= 0)
    {
      if (mir)
      {
        //Serial.println("X pre mirror:" + String(cX, DEC));
        cX = bW - cX;
        //Serial.println("X post mirror:" + String(cX, DEC));
      }
      X.relZero = X.A * cX;
      Y.relZero = Y.A * cY;
      modRot = cU;
      ack = true;
      return;
    }
    else if (u.indexOf("offset") >= 0)
    {
      oX.f = cX;
      oY.f = cY;
      //Serial.println("Set Union Values. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC) + ". Proof X:" + String(oX.f, DEC) + ", Y:" + String(oY.f, DEC));
      for (int n = 0; n < 4; n++)
      {
        EEPROM.write(n, oX.b[n]);
        //Serial.println("xByte #" + String(n, DEC) + ":" + oX.b[n]);
        EEPROM.write((n + 4), oY.b[n]);
        //Serial.println("yByte #" + String(n, DEC) + ":" + oY.b[n]);
      }
      EEPROM.commit();
      //Serial.println("Commited to Flash");
      ack = true;
      return;
    }
    else if (u.indexOf("tool") >= 0)
    {
      toolT.i = int(cX);
      toolD.f = cY;
      if (toolT.i == 0 || toolD.f == 0)
      {
        toolT.i = 0;
        toolD.f = 0;
      }
      for (int n = 0; n < 4; n++)
      {
        EEPROM.write((n + 8), toolD.b[n]);

        if (n < 2)
        {
          EEPROM.write((n + 12), toolT.b[n]);
        }
      }
      EEPROM.commit();
      //Serial.println("Stored Tool. Type = :" + String(toolT.i, DEC) + ", Diameter:" + String(toolD.f, DEC));
      ack = true;
      taskRunning = false;
      s = "";

      return;
    }
    else if (u.indexOf("Speed") >= 0 || u.indexOf("speed") >= 0)
    {
      if (u.indexOf("getSpeed") >= 0)
      {
        Serial.println("Move:" + String(X.mov, DEC) + ", " + String(Y.mov, DEC) + ", " + String(Z.mov, DEC));
        Serial.println("Draw:" + String(X.drw, DEC) + ", " + String(Y.drw, DEC) + ", " + String(Z.drw, DEC));
        Serial.println("Mill:" + String(X.mil, DEC) + ", " + String(Y.mil, DEC) + ", " + String(Z.mil, DEC));
      }
      else
      {
        if (u.indexOf('M') > 0)
        {
          for (int n = 0; n < 4; n++)
          {
            if (cX != -100000)
            {
              sXm.f = cX;
              EEPROM.write(n + 10, sXm.b[n]);
              X.mov = sXm.f;
            }

            if (cY != -100000)
            {
              sYm.f = cY;
              for (int n = 0; n < 2; n++)
              {
                EEPROM.write(n + 14, sYm.b[n]);
                Y.mov = sYm.f;
              }
            }

            if (cZ != -100000)
            {
              sZm.f = cZ;
              for (int n = 0; n < 2; n++)
              {
                EEPROM.write(n + 18, sZm.b[n]);
                Z.mov = sZm.f;
              }
            }
          }

          if (cX != -100000)
          {
            X.mov = cX;
          }

          if (cY != -100000)
          {
            Y.mov = cY;
          }

          if (cZ != -100000)
          {
            Z.mov = cZ;
          }
          //Serial.println("Wrote MOVE:" + String(sXm.f, DEC) + ", " + String(sYm.f, DEC) + ", " + String(sZm.f, DEC));
        }
        else if (u.indexOf('D') > 0)
        {
          for (int n = 0; n < 4; n++)
          {
            if (cX != -100000)
            {
              sXd.f = cX;
              EEPROM.write(n + 22, sXd.b[n]);
              X.drw = sXd.f;
            }

            if (cY != -100000)
            {
              sYd.f = cY;
              EEPROM.write(n + 26, sYd.b[n]);
              Y.drw = sYd.f;
            }

            if (cZ != -100000)
            {
              sZd.f = cZ;
              EEPROM.write(n + 30, sZd.b[n]);
              Z.drw = sZd.f;
            }
          }

          if (cX != -100000)
          {
            X.drw = cX;
          }

          if (cY != -100000)
          {
            Y.drw = cY;
          }

          if (cZ != -100000)
          {
            Z.drw = cZ;
          }
          //Serial.println("Wrote DRAW:" + String(sXd.f, DEC) + ", " + String(sYd.f, DEC) + ", " + String(sZd.f, DEC));
        }
        else if (u.indexOf('W') > 0)
        {
          for (int n = 0; n < 4; n++)
          {
            if (cX != -100000)
            {
              sXw.f = cX;
              EEPROM.write(n + 34, sXw.b[n]);
              X.mil = sXw.f;
            }

            if (cY != -100000)
            {
              sYw.f = cY;
              EEPROM.write(n + 38, sYw.b[n]);
              Y.mil = sYw.f;
            }

            if (cZ != -100000)
            {
              sZw.f = cZ;
              EEPROM.write(n + 42, sZw.b[n]);
              Z.mil = sZw.f;
            }
          }
          if (cX != -100000)
          {
            X.mil = cX;
          }

          if (cY != -100000)
          {
            Y.mil = cY;
          }

          if (cZ != -100000)
          {
            Z.mil = cZ;
          }
          //Serial.println("Wrote MILL:" + String(sXw.f, DEC) + ", " + String(sYw.f, DEC) + ", " + String(sZw.f, DEC));
        }

        EEPROM.commit();
        ack = true;
      }
      return;
    }
    else if (cmd == 'W' || cmd == 'w')
    {
      if (cmd == 'W')
      {
        preDot = false;
      }
      else
      {
        preDot = true;
      }

      if (cZ != -100000 && cU != -100000)
      {
        xSp = X.drw;
        ySp = Y.drw;
        fire = false;
        taskRunning = true;
        drilling();
      }
      else
      {
        Serial.println("Drilling command missing parameters");
      }
      ack = true;
      return;
    }
    else
    {
      if (cmd == 'F' || cmd == 'f')
      {
        if (cmd == 'F')
        {
          destinationMath(false);
        }
        else
        {
          destinationMath(true);
        }

        movetoHeight(focusHeight);

        xSp = X.drw;
        ySp = Y.drw;
        taskRunning = true;
        ack = true;
        xTaskCreate(fillMMabsolute, "Fill", 10000, NULL, 4, NULL);
        return;
      }
      else if (cmd == 'V' || cmd == 'v')
      {
        if (cmd == 'V')
        {
          destinationMath(false);
        }
        else
        {
          destinationMath(true);
        }

        if (cR != -100000 && cH != -100000)
        {
          movetoHeight(focusHeight);

          xSp = X.drw;
          ySp = Y.drw;
          fire = true;
          taskRunning = true;
          ack = true;
          xTaskCreate(viaPath, "Via", 10000, NULL, 4, NULL);
        }
        return;
      }
      else if (cmd == 'P' || cmd == 'p')
      {
        //Serial.println("Starting Pad. cX:" + String(cX, DEC) + ", cY:" + String(cY, DEC) + ", cW:" + String(cW, DEC) + ", cH:" + String(cH, DEC) + ", cU:" + String(cU, DEC));
        if (cU != -100000)
        {
          padRot = cU;
        }

        if (cmd == 'P')
        {
          if (cR != -100000)
          {
            padLogic(true, false);
          }
          else
          {
            padLogic(false, false);
          }
        }
        else if (cmd == 'p')
        {
          if (cR != -100000)
          {
            padLogic(true, true);
          }
          else
          {
            padLogic(false, true);
          }
        }

        if (mir)
        {
          mir = false;
          destinationMath(true);
          mir = true;
        }
        else
        {
          destinationMath(true);
        }

        movetoHeight(focusHeight);

        taskRunning = true;
        ack = true;

        xSp = X.drw;
        ySp = Y.drw;
        if (cW != -100000 && cH != -100000)
        {
          //Serial.println("Square Pad. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC) + ", W:" + String(cW, DEC) + ", H:" + String(cH, DEC) + ", ROT:" + String(cU, DEC));
          relCircle = false;
          xTaskCreate(fillMMabsolute, "Square Pad", 10000, NULL, 4, NULL);
          return;
        }
        else if (cR != -100000)
        {
          if (cH == -100000)
          {
            //Serial.println("Circular Pad. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC) + ", R:" + String(cR, DEC));
            relCircle = true;
            xTaskCreate(circularPath, "Circle Pad", 10000, NULL, 4, NULL);
            return;
          }
          else
          {
            cR = cR * .5;
            cH = cH * .5;
            //Serial.println("Via Pad. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC) + ", R:" + String(cR, DEC) + ", H:" + String(cH, DEC));
            relCircle = true;
            xTaskCreate(viaPath, "Via Pad", 10000, NULL, 4, NULL);
            return;
          }
        }
      }
      else if (cmd == 'Q' || cmd == 'q')
      {
        xSp = X.drw;
        ySp = Y.drw;
        fire = false;
        taskRunning = true;
        ack = true;
        xTaskCreate(milling, "Mill", 10000, NULL, 4, NULL);
        return;
      }
      else if (cmd == 'E' || cmd == 'e')
      {
        if (cmd == 'E')
        {
          destinationMath(false);
        }
        else
        {
          destinationMath(true);
        }

        if (cR != -100000)
        {
          taskRunning = true;
          ack = true;
          xTaskCreate(boreCircle, "Bore Circle", 10000, NULL, 4, NULL);
        }
        return;
      }
      else
      {
        //Serial.println("Command unrecognized. U:" + u + ", CMD:" + cmd);
      }
    }
  }
}

void opStep()
{
  esp_task_wdt_init(720, false);
  if (digitalRead(en) == HIGH)
  {
    digitalWrite(en, LOW);
  }
  stepsDone = false;
  long sX = abs(i), sY = abs(o), sZ = abs(r), sT;
  double waitX, waitY, waitZ;
  unsigned long destX, destY, destZ, begX, begY, begZ;
  double decX, decY, decZ;
  unsigned long nX = 0, nY = 0, nZ = 0;
  unsigned long xT = 0, yT = 0, zT = 0, curT;
  long Pxn, Pyn, Pzn;
  long dlx, dly, dlz;
  int drvAxis, xDec, yDec, zDec;
  bool stepX, stepY, stepZ;

  if (i > 0)
  {
    X.dirHigh();
  }
  else
  {
    X.dirLow();
  }

  if (o > 0)
  {
    Y.dirHigh();
  }
  else
  {
    Y.dirLow();
  }

  if (r > 0)
  {
    Z.dirHigh();
  }
  else
  {
    Z.dirLow();
  }

  destX = X.absPos + i;
  destY = Y.absPos + o;
  destZ = Z.absPos + r;
  begX = X.absPos;
  begY = Y.absPos;
  begZ = Z.absPos;

  if (sX > sY && sX >= sZ)
  {
    drvAxis = -1;
    waitX = xSp;
    sT = 2 * waitX * sX;

    if (sY > 0)
    {
      waitY = ((double)sT / (double)(2 * sY));
    }
    else
    {
      waitY = 0;
    }

    if (sZ > 0)
    {
      waitZ = ((double)sT / (double)(2 * sZ));
    }
    else
    {
      waitZ = 0;
    }
  }
  else if (sY > sX && sY >= sZ)
  {
    drvAxis = 0;
    waitY = ySp;
    sT = 2 * waitY * sY;

    if (sX > 0)
    {
      waitX = ((double)sT / (double)(2 * sX));
    }
    else
    {
      waitX = 0;
    }

    if (sZ > 0)
    {
      waitZ = ((double)sT / (double)(2 * sZ));
    }
    else
    {
      waitZ = 0;
    }
  }
  else if (sZ > sX && sZ >= sY)
  {
    drvAxis = 1;
    waitZ = zSp;

    if (sX > 0)
    {
      waitX = ((double)sT / (double)(2 * sX));
    }
    else
    {
      waitX = 0;
    }

    if (sY > 0)
    {
      waitY = ((double)sT / (double)(2 * sY));
    }
    else
    {
      waitY = 0;
    }
  }

  //Serial.println("TOTAL X:" + String(sX, DEC) + ", Y:" + String(sY, DEC) + ", Z:" + String(sZ, DEC));

  if (i > 0)
  {
    xDec = 1;
  }
  else if (i < 0)
  {
    xDec = -1;
  }
  else
  {
    xDec = 0;
  }

  if (o > 0)
  {
    yDec = 1;
  }
  else if (o < 0)
  {
    yDec = -1;
  }
  else
  {
    yDec = 0;
  }

  if (r > 0)
  {
    zDec = 1;
  }
  else if (r < 0)
  {
    zDec = -1;
  }
  else
  {
    zDec = 0;
  }

  if (fire)
  {
    laser.On();
  }

  //X driving
  if (drvAxis < 0)
  {
    //Serial.println("X driving");
    dlx = sX;
    dly = sY;
    dlz = sZ;
    Pyn = (2 * dly) - dlx;
    Pzn = (2 * dlz) - dlx;
    stepX = false;
    stepY = false;
    stepZ = false;
    while (nX < sX)
    {
      curT = micros();
      if (!stepX)
      {
        if (Pyn >= 0)
        {
          stepY = true;
          Pyn -= 2 * dlx;
        }

        if (Pzn >= 0)
        {
          stepZ = true;
          Pzn -= 2 * dlx;
        }

        Pyn += 2 * dly;
        Pzn += 2 * dlz;
        stepX = true;
        nX++;
        //Serial.println("Step X");
      }
      else
      {
        if (curT - xT >= waitX)
        {
          if (X.stepRead() == LOW)
          {
            X.stepHigh();
          }
          else
          {
            X.stepLow();
            X.absPos += decX;
            stepX = false;
            //Serial.println("X Stepped. NX: " + String(nX, DEC));
          }

          if (stepY)
          {
            if (Y.stepRead() == LOW)
            {
              Y.stepHigh();
            }
            else
            {
              Y.stepLow();
              Y.absPos += decY;
              stepY = false;
            }
          }

          if (stepZ)
          {
            if (Z.stepRead() == LOW)
            {
              Z.stepHigh();
            }
            else
            {
              Z.stepLow();
              Z.absPos += decZ;
              stepZ = false;
            }
          }
          xT = curT;
        }
      }
    }
  }
  else if (drvAxis == 0)
  {
    //Serial.println("Y Driving");
    dlx = sX;
    dly = sY;
    dlz = sZ;
    Pxn = (2 * dlx) - dly;
    Pzn = (2 * dlz) - dly;
    stepX = false;
    stepY = false;
    stepZ = false;
    while (nY < sY)
    {
      curT = micros();
      if (!stepY)
      {
        if (Pxn >= 0)
        {
          stepX = true;
          Pxn -= 2 * dly;
        }

        if (Pzn >= 0)
        {
          stepZ = true;
          Pzn -= 2 * dly;
        }

        Pxn += 2 * dlx;
        Pzn += 2 * dlz;
        stepY = true;
        nY++;
        //Serial.println("Step X");
      }
      else
      {
        if (curT - yT >= waitY)
        {
          if (Y.stepRead() == LOW)
          {
            Y.stepHigh();
          }
          else
          {
            Y.stepLow();
            Y.absPos += decY;
            stepY = false;
            //Serial.println("X Stepped. NX: " + String(nX, DEC));
          }

          if (stepX)
          {
            if (X.stepRead() == LOW)
            {
              X.stepHigh();
            }
            else
            {
              X.stepLow();
              X.absPos += decX;
              stepX = false;
            }
          }

          if (stepZ)
          {
            if (Z.stepRead() == LOW)
            {
              Z.stepHigh();
            }
            else
            {
              Z.stepLow();
              Z.absPos += decZ;
              stepZ = false;
            }
          }
          yT = curT;
        }
      }
    }
  }
  else
  {
    //Serial.println("Z Driving");
    dlx = sX;
    dly = sY;
    dlz = sZ;
    Pyn = (2 * dly) - dlz;
    Pxn = (2 * dlx) - dlz;
    stepX = false;
    stepY = false;
    stepZ = false;
    while (nZ < sZ)
    {
      curT = micros();
      if (!stepZ)
      {
        if (Pyn >= 0)
        {
          stepY = true;
          Pyn -= 2 * dlz;
        }

        if (Pxn >= 0)
        {
          stepX = true;
          Pxn -= 2 * dlz;
        }

        Pyn += 2 * dly;
        Pxn += 2 * dlx;
        stepZ = true;
        nZ++;
        //Serial.println("Step X");
      }
      else
      {
        if (curT - zT >= waitZ)
        {
          if (Z.stepRead() == LOW)
          {
            Z.stepHigh();
          }
          else
          {
            Z.stepLow();
            Z.absPos += decZ;
            stepZ = false;
            //Serial.println("X Stepped. NX: " + String(nX, DEC));
          }

          if (stepY)
          {
            if (Y.stepRead() == LOW)
            {
              Y.stepHigh();
            }
            else
            {
              Y.stepLow();
              Y.absPos += decY;
              stepY = false;
            }
          }

          if (stepX)
          {
            if (X.stepRead() == LOW)
            {
              X.stepHigh();
            }
            else
            {
              X.stepLow();
              X.absPos += decX;
              stepX = false;
            }
          }
          zT = curT;
        }
      }
    }
  }

  //Serial.println("Time Elapsed:" + String(micros() - startedTime, DEC) + ", Delta:" + String(((long)sT - ((long)micros() - (long)startedTime)), DEC));

  stepsDone = true;
  i = 0;
  o = 0;
  r = 0;

  X.absPos = destX;
  X.relPos = destX;
  Y.absPos = destY;
  Y.relPos = destY;
  Z.absPos = destZ;
  Z.relPos = destZ;

  if (fire && !drawNodes)
  {
    laser.Off();
  }
}

void fillMMabsolute(void * param)
{
  esp_task_wdt_init(720, false);
  unsigned long drwInc, iX, iY;
  bool mainD = true;

  if (fW <= 0 || fH <= 0)
  {
    //Serial.println("Ended fill because dimensions <= 0");
    taskRunning = false;
    vTaskDelete(NULL);
  }

  if (operationMode == 0)
  {
    if (cW > .25 && cH > .25)
    {
      cW = cW - .1f;
      fW = round(X.A * cW);
      cH = cH - .1f;
      fH = round(Y.A * cH);
      //Serial.println("Called Fill. W:" + String(cW, DEC) + ", H:" + String(cH, DEC) + ".....fW:" + String(fW, DEC) + ", fH:" + String(fH, DEC));

      fillPart = 0;
      //Serial.println("called fill. step 0");
      fire = false;
      xSp = X.mov;
      ySp = Y.mov;
      if (i != 0 || o != 0)
      {
        opStep();
      }

      xSp = X.drw;
      ySp = Y.drw;
      for (;;)
      {
        i = fW;
        o = 0;
        r = 0;
        fire = true;
        opStep();

        i = 0;
        o = fH;
        r = 0;
        fire = true;
        opStep();

        i = -1 * fW;
        o = 0;
        r = 0;
        fire = true;
        opStep();

        i = 0;
        o = -1 * fH;
        r = 0;
        fire = true;
        opStep();

        delay(50);
        iX = fW + X.absPos;
        iY = fH + Y.absPos;

        if (cH > cW || cH == cW)
        {
          drwInc = X.A * .05;
          while (X.absPos <= iX)
          {
            if (mainD)
            {
              i = 0;
              o = fH;
              r = 0;
              mainD = false;
            }
            else
            {
              i = drwInc;
              o = -1 * fH;
              r = 0;
              mainD = true;
            }
            fire = true;
            opStep();
          }

          i = 0;
          o = fH;
          r = 0;
          fire = true;
          opStep();
          //Serial.println("Fill ended with X equivalent. ABS:" + String(X.absPos, DEC) + ", iX:" + String(iX, DEC));
        }
        else
        {
          drwInc = Y.A * .05;
          while (Y.absPos <= iY)
          {
            if (mainD)
            {
              o = 0;
              i = fW;
              r = 0;
              mainD = false;
            }
            else
            {
              o = drwInc;
              i = -1 * fW;
              r = 0;
              mainD = true;
            }
            fire = true;
            opStep();
          }

          o = 0;
          i = fW;
          r = 0;
          fire = true;
          opStep();
          //Serial.println("Fill ended with Y equivalent. ABS:" + String(Y.absPos, DEC) + ", iY:" + String(iY, DEC));
        }
        fire = false;
        fillPart = 0;
        taskRunning = false;
        vTaskDelete(NULL);
      }
    }
    else
    {
      //Serial.println("called small fill. W:" + String(cW, DEC) + ", H:" + String(cH, DEC));
      fillPart = 0;
      fire = false;

      if (i != 0 || o != 0)
      {
        xSp = X.mov;
        ySp = Y.mov;
        opStep();
      }

      fire = false;
      opStep();


      if (cW > cH)
      {
        fH = 0;
      }
      else
      {
        fW = 0;
      }
      //Serial.println("called small fill. W:" + String(fW, DEC) + ", H:" + String(fH, DEC));

      xSp = X.drw + 5;
      ySp = Y.drw + 5;
      while (fillPart < 6)
      {
        vTaskDelay(50);
        //Serial.println("called small fill. step " + String(fillPart, DEC));
        if (fillPart < 5)
        {
          stepsDone = false;
          if (mainD)
          {
            i = fW;
            o = fH;
          }
          else
          {
            i = -1 * fW;
            o = -1 * fH;
            fillPart++;
          }
          mainD = !mainD;
          fire = true;
          opStep();
        }
        else
        {
          //Serial.println("ended small fill");

          taskRunning = false;
          stepsDone = false;
          vTaskDelete(NULL);
        }
      }

      //Serial.println("ended small fill");

      xSp = X.drw;
      ySp = Y.drw;
      taskRunning = false;
      vTaskDelete(NULL);
    }
  }
  else if (operationMode == 1)
  {
    int cnt = 0;

    cW = cW - .1;
    cH = cH - .1;
    fW = round(X.A * cW);
    fH = round(Y.A * cH);

    fire = false;
    xSp = X.mov;
    ySp = Y.mov;
    if (i != 0 || o != 0)
    {
      opStep();
    }

    xSp = X.drw;
    ySp = Y.drw;


    while (cnt < 2)
    {
      i = fW;
      o = 0;
      r = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH;
      r = 0;
      fire = true;
      opStep();

      i = -1 * fW;
      o = 0;
      r = 0;
      fire = true;
      opStep();

      i = 0;
      o = -1 * fH;
      r = 0;
      fire = true;
      opStep();
      cnt++;
    }

    fire = false;
    fillPart = 0;
    taskRunning = false;
    vTaskDelete(NULL);
  }
}

void drawBorder(void * param)
{
  esp_task_wdt_init(720, false);
  laser.runVal += 15;
  if (operationMode == 0)
  {
    for (int n = 0; n < 4; n++)
    {
      i = fW;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH;
      fire = true;
      opStep();

      i = fW * -1;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH * -1;
      fire = true;
      opStep();
    }
  }
  else if (operationMode == 1)
  {
    for (int n = 0; n < 4; n++)
    {
      i = fW;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH;
      fire = true;
      opStep();

      i = fW * -1;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH * -1;
      fire = true;
      opStep();

      i = fW;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH;
      fire = true;
      opStep();

      i = fW * -1;
      o = 0;
      fire = true;
      opStep();

      i = 0;
      o = fH * -1;
      fire = true;
      opStep();
    }
  }

  fire = false;
  taskRunning = false;
  stepsDone = false;
  laser.runVal -= 15;
  vTaskDelete(NULL);
}

void circularPath(void * param)
{
  esp_task_wdt_init(720, false);
  //Serial.println("called circle");
  long  sX, drwInc;
  unsigned long rds, startX, endX;
  bool mainD = false;
  rd = (cR / 2);
  i = round(((cX - rd) * X.A) - X.absPos);
  o = round((cY * Y.A) - Y.absPos);
  rd -= .01;
  r = 0;
  circlePart = 0;
  fire = false;
  stepsDone = false;
  //Serial.println("Via Path. Movement to initial point is X:" + String(i, DEC) + ", Y:" + String(o, DEC) + ", R:" + String(cR, DEC) + ", H:" + String(cH, DEC));

  xSp = X.mov;
  ySp = Y.mov;
  if (i != 0 || o != 0)
  {
    stepsDone = false;
    opStep();
  }
  else
  {
    stepsDone = true;
  }

  xSp = X.drw + 8;
  ySp = Y.drw + 8;
  xI = cX;
  yI = cY;
  if (operationMode == 0)
  {
    while (rd >= .01)
    {
      if (circlePart <= 2 * PI)
      {
        if (stepsDone)
        {
          //Serial.println("increment circle");
          stepsDone = false;
          circlePart += PI / 360;
          nX = rd * cos(circlePart);
          nX = xI + nX;
          nY = rd * sin(circlePart);
          nY = yI + nY;

          if (nX * X.A != X.relPos)
          {
            i = (nX * A) - X.relPos;
          }
          else
          {
            i = 0;
          }

          if (nY * Y.A != Y.relPos)
          {
            o = (nY * Y.A) - Y.relPos;
          }
          else
          {
            o = 0;
          }

          if (i != 0 || o != 0)
          {
            fire = true;
            opStep();
            //opStep();
          }
          else
          {
            stepsDone = true;
          }
        }
        else
        {
          vTaskDelay(10);
        }
      }
      else
      {
        if (circlePart != 18000)
        {
          if (stepsDone)
          {
            //Serial.println("move inward");
            stepsDone = false;
            i = X.A * .0125 * -1;
            o = 0;
            fire = true;
            opStep();
            circlePart = 18000;
          }
          else
          {
            vTaskDelay(10);
          }
        }
        else
        {
          if (stepsDone)
          {
            //Serial.println("reduce radius");
            circlePart = 0;
            if (circleHandle != NULL)
            {
              rd -= .025;
            }
            else
            {
              rd -= .025;
            }
          }
          else
          {
            vTaskDelay(10);
          }
        }
      }
    }
  }
  else if (operationMode == 1)
  {
    rd = cR - .1;
    i = X.A * -1 * .1;
    o = 0;
    r = 0;

    xSp = X.mov;
    ySp = Y.mov;
    fire = false;
    if (i != 0 || o != 0)
    {
      stepsDone = false;
      opStep();
    }
    xI = cX;
    yI = cY;

    int cnt = 0;
    while (cnt < 4)
    {
      while (circlePart < 2 * PI)
      {
        circlePart += PI / 180;
        nX = rd * cos(circlePart);
        nX = xI + nX;
        nY = rd * sin(circlePart);
        nY = yI + nY;

        if (nX * X.A != X.relPos)
        {
          i = (nX * X.A) - X.relPos;
        }
        else
        {
          i = 0;
        }

        if (nY * Y.A != Y.relPos)
        {
          o = (nY * Y.A) - Y.relPos;
        }
        else
        {
          o = 0;
        }

        if (i != 0 || o != 0)
        {
          fire = true;
          opStep();
        }
      }
      cnt++;
      circlePart = 0;
    }
  }
  if (circleHandle != NULL)
  {
    //Serial.println("End of circle path with non null handle");
    circleHandle = NULL;
    vTaskDelete(NULL);
  }
  else
  {
    //Serial.println("End of circle path with null handle");
    ack = true;
    taskRunning = false;
    vTaskDelete(NULL);
  }
}

void viaPath(void * param)
{
  esp_task_wdt_init(720, false);
  //Serial.println("called circle");
  long  sX, drwInc;
  unsigned long rds, startX, endX;
  bool mainD = false;
  i += fR;
  rd = cR;
  circlePart = 0;
  fire = false;
  stepsDone = false;
  //Serial.println("Via Path. Movement to initial point is X:" + String(i, DEC) + ", Y:" + String(o, DEC) + ", R:" + String(cR, DEC) + ", H:" + String(cH, DEC));

  xSp = X.mov;
  ySp = Y.mov;
  if (i != 0 || o != 0)
  {
    stepsDone = false;
    opStep();
  }
  else
  {
    stepsDone = true;
  }

  xSp = X.drw + (40.0 * (cH / rd));
  ySp = Y.drw + (40.0 * (cH / rd));
  xI = cX;
  yI = cY;
  if (operationMode == 0)
  {
    while (rd >= cH)
    {
      if (stepsDone)
      {
        if (circlePart < 2 * PI)
        {
          //Serial.println("increment circle");
          stepsDone = false;
          circlePart += PI / 360;
          nX = rd * cos(circlePart);
          nX = xI + nX;
          nY = rd * sin(circlePart);
          nY = yI + nY;

          if (nX * X.A != X.relPos)
          {
            i = (nX * A) - X.relPos;
          }
          else
          {
            i = 0;
          }

          if (nY * Y.A != Y.relPos)
          {
            o = (nY * Y.A) - Y.relPos;
          }
          else
          {
            o = 0;
          }

          if (i != 0 || o != 0)
          {
            fire = true;
            opStep();
          }
          else
          {
            stepsDone = true;
          }
        }
        else if (circlePart != 18000 && circlePart >= 2 * PI)
        {
          //Serial.println("move inward");
          stepsDone = false;
          i = X.A * .05 * -1;
          o = 0;
          fire = true;
          opStep();
          circlePart = 18000;
        }
        else
        {
          circlePart = 0;
          if (circleHandle != NULL)
          {
            rd -= .01;
          }
          else
          {
            rd -= .01;
          }
        }
      }
      else
      {
        vTaskDelay(10);
      }
    }
  }
  else if (operationMode == 1)
  {
    rd = cR - .1;
    i = X.A * -1 * .1;
    o = 0;
    r = 0;

    xSp = X.mov;
    ySp = Y.mov;
    fire = false;
    if (i != 0 || o != 0)
    {
      stepsDone = false;
      opStep();
    }
    xI = cX;
    yI = cY;

    int cnt = 0;
    while (cnt < 4)
    {
      while (circlePart < 2 * PI)
      {
        circlePart += PI / 180;
        nX = rd * cos(circlePart);
        nX = xI + nX;
        nY = rd * sin(circlePart);
        nY = yI + nY;

        if (nX * X.A != X.relPos)
        {
          i = (nX * X.A) - X.relPos;
        }
        else
        {
          i = 0;
        }

        if (nY * Y.A != Y.relPos)
        {
          o = (nY * Y.A) - Y.relPos;
        }
        else
        {
          o = 0;
        }

        if (i != 0 || o != 0)
        {
          fire = true;
          opStep();
        }
      }
      cnt++;
      circlePart = 0;
    }
  }
  if (circleHandle != NULL)
  {
    //Serial.println("End of circle path with non null handle");
    circleHandle = NULL;
    vTaskDelete(NULL);
  }
  else
  {
    //Serial.println("End of circle path with null handle");
    ack = true;
    taskRunning = false;
    vTaskDelete(NULL);
  }
}

void drilling()
{
  esp_task_wdt_init(720, false);
  digitalWrite(en, LOW);
  //limitMotion(4);
  float xBu = cX, yBu = cY, zBu = cZ, uBu = cU;
  xSp = X.mov;
  ySp = Y.mov;
  zSp = Z.mov;
  cX = xBu;
  cY = yBu;
  cZ = -100000;

  if (!mir)
  {
    if (cX != -100000)
    {
      cX += spinOffX;
    }
    else
    {
      cX = spinOffX;
    }

    if (cY != -100000)
    {
      cY += spinOffY;
    }
    else
    {
      cY = spinOffY;
    }
    destinationMath(false);
  }
  else
  {
    if (cX == -100000)
    {
      cX = 0;
    }

    if (cY == -100000)
    {
      cY = 0;
    }

    destinationMath(false);
    opStep();

    cX = spinOffX;
    cY = spinOffY;
    destinationMath(true);
  }

  r = 0;
  opStep();
  //Serial.println("Move Spindle To Location");
  //delay(1000);


  r = (zBu * Z.A) - Z.absPos;
  o = 0;
  i = 0;
  opStep();
  //Serial.println("Move Tool To Height");
  //delay(1000);
  if (isDrilling && !spindleOn)
  {
    spindleOn = true;
    spindle.On();
    vacuum.On();
  }
  //Serial.println("Start Spindle");
  delay(100);

  unsigned long uc = uBu * Z.A;
  Z.dirHigh();
  for (int n = 0; n <= uc; n++)
  {
    Z.stepHigh();
    delayMicroseconds(zSp * 32);
    Z.stepLow();
    delayMicroseconds(zSp * 32);
    Z.absPos++;
    Z.relPos++;
  }
  //Serial.println("Plunge specified distance");
  //delay(1000);

  Z.dirLow();
  uc += (10.0) * Z.A;
  for (int n = 0; n <= uc; n++)
  {
    Z.stepHigh();
    delayMicroseconds(zSp);
    Z.stepLow();
    delayMicroseconds(zSp);
    Z.absPos--;
    Z.relPos--;
  }
  //Serial.println("Return specified distance + 10mm for safety");
  //delay(1000);

  //Serial.println("Spindle Off");
  delay(100);

  taskRunning = false;
  preDot = false;
  //Serial.println("End of task");
}

void milling(void * param)
{
  //0) Check Safe Height
  //Serial.println("Checking If Currently In Milling Position");
  if (Z.absPos < plungeDepth)
  {
    //Serial.println("Checking Mill Height Safety");
    movetoHeight(focusHeight);
    delay(2000);

    //2) Move To Position
    //Serial.println("Moving To Position With Mill Offset");

    if (!mir)
    {
      if (cX != -100000)
      {
        cX += spinOffX;
      }
      else
      {
        cX = spinOffX;
      }

      if (cY != -100000)
      {
        cY += spinOffY;
      }
      else
      {
        cY = spinOffY;
      }

      destinationMath(false);
    }
    else
    {
      if (cX == -100000)
      {
        cX = 0;
      }

      if (cY == -100000)
      {
        cY = 0;
      }

      destinationMath(false);
      opStep();

      cX = spinOffX;
      cY = spinOffY;
      destinationMath(true);
    }

    opStep();

    //Serial.println("Engaging Spindle");
    spindleOn = true;
    spindle.On(false);
    vacuum.On();
    delay(2000);

    //Serial.println("Plunging 7mm");
    r = Z.A * 9.0;
    i = 0;
    o = 0;
    opStep();
    delay(2000);

    //Serial.println("Moving To Plunge Depth");
    i = 0;
    o = 0;
    xSp = X.mil;
    ySp = Y.mil;
    zSp = Z.mil;
    while (Z.absPos < plungeDepth)
    {
      r = Z.A * .05;
      opStep();
      delay(10);
    }
    delay(2000);
  }
  else
  {
    xSp = X.mil;
    ySp = Y.mil;
    zSp = Z.mil;
    if (!mir)
    {
      if (cX != -100000)
      {
        cX += spinOffX;
      }
      else
      {
        cX = spinOffX;
      }

      if (cY != -100000)
      {
        cY += spinOffY;
      }
      else
      {
        cY = spinOffY;
      }

      destinationMath(false);
    }
    else
    {
      if (cX == -100000)
      {
        cX = 0;
      }

      if (cY == -100000)
      {
        cY = 0;
      }

      destinationMath(false);
      opStep();

      cX = spinOffX;
      cY = spinOffY;
      destinationMath(true);
    }

    //Serial.println("Moving To Next Location");
    opStep();
    delay(2000);
  }

  //Serial.println("Milling Operation Complete");
  taskRunning = false;
  ack = true;
  vTaskDelete(NULL);
}

void nodeLogic(void * param)
{
  esp_task_wdt_init(720, false);
  String part = "";
  float xM, yM, xI, yI, xO, yO;
  int nodeCount = 1, idx = 0;

  //count nodes in sequence
  for (int n = 0; n < s.length(); n++)
  {
    if (s[n] == 'D')
    {
      nodeCount++;
    }
  }

  idx = 0;


  //parse nodes from sequence
  char cnv[40];
  String g = s.substring(s.indexOf("MX"));
  g = g.substring(0, g.indexOf("/"));
  part = g.substring(g.indexOf("X") + 1);
  part = part.substring(0, part.indexOf("Y"));
  part.toCharArray(cnv, sizeof(cnv));
  xM = atof(cnv);
  g = g.substring(g.indexOf("Y") + 1);
  g.toCharArray(cnv, sizeof(cnv));
  yM = atof(cnv);
  if (mir)
  {
    xM = bW - xM;
  }
  xI = xM;
  yI = yM;
  //Serial.println("String:" + s);
  if (s.indexOf("R") >= 0)
  {
    part = s.substring(s.indexOf("R") + 1);
    part = part.substring(0, part.indexOf("/"));
    part.toCharArray(cnv, sizeof(cnv));
    traceWidth = atof(cnv);
  }
  s = s.substring(s.indexOf("/") + 1);
  nodes[idx].x = xM;
  nodes[idx].y = yM;
  idx++;

  while (idx < nodeCount)
  {
    g = s.substring(s.indexOf("DX"));
    g = g.substring(0, g.indexOf("/"));
    part = g.substring(g.indexOf("X") + 1);
    part = part.substring(0, part.indexOf("Y"));
    part.toCharArray(cnv, sizeof(cnv));
    xM = atof(cnv);
    g = g.substring(g.indexOf("Y") + 1);
    g.toCharArray(cnv, sizeof(cnv));
    yM = atof(cnv);
    s = s.substring(s.indexOf("/") + 1);
    if (mir)
    {
      xM = bW - xM;
    }
    nodes[idx].x = xM;
    nodes[idx].y = yM;
    idx++;
  }
  idx = 1;
  fire = false;

  xSp = X.drw;
  ySp = Y.drw;
  //Drawing nodes
  float powerScale = traceWidth / .05;
  laser.runVal += powerScale;
  cX = xI;
  cY = yI;
  destinationMath(false);
  opStep();

  while (idx < nodeCount)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx++;
  }
  idx = nodeCount - 1;

  while (idx >= 0)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx--;
  }
  idx = 0;

  while (idx < nodeCount)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx++;
  }
  idx = nodeCount - 1;

  while (idx >= 0)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx--;
  }
  idx = 0;

  while (idx < nodeCount)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx++;
  }
  idx = nodeCount - 1;

  while (idx >= 0)
  {
    cX = nodes[idx].x;
    cY = nodes[idx].y;
    destinationMath(false);
    fire = true;
    opStep();
    idx--;
  }
  idx = 0;

  laser.runVal -= powerScale;
  ledcWrite(8, 0);
  s = "";
  ack = true;
  taskRunning = false;
  drawNodes = false;
  readingNodes = false;
  vTaskDelete(NULL);
}

void nodeLogic1(void * param)
{
  esp_task_wdt_init(720, false);
  String part = "";
  float xM, yM, xI, yI, xO, yO;
  int nodeCount = 1, idx = 0;

  //count nodes in sequence
  for (int n = 0; n < s.length(); n++)
  {
    if (s[n] == 'D')
    {
      nodeCount++;
    }
  }

  idx = 0;


  //parse nodes from sequence
  char cnv[40];
  String g = s.substring(s.indexOf("MX"));
  g = g.substring(0, g.indexOf("/"));
  part = g.substring(g.indexOf("X") + 1);
  part = part.substring(0, part.indexOf("Y"));
  part.toCharArray(cnv, sizeof(cnv));
  xM = atof(cnv);
  g = g.substring(g.indexOf("Y") + 1);
  g.toCharArray(cnv, sizeof(cnv));
  yM = atof(cnv);
  if (mir)
  {
    xM = bW - xM;
  }
  xI = xM;
  yI = yM;
  //Serial.println("String:" + s);
  if (s.indexOf("R") >= 0)
  {
    part = s.substring(s.indexOf("R") + 1);
    part = part.substring(0, part.indexOf("/"));
    part.toCharArray(cnv, sizeof(cnv));
    traceWidth = atof(cnv);
  }
  s = s.substring(s.indexOf("/") + 1);
  nodes[idx].x = xM;
  nodes[idx].y = yM;
  idx++;

  while (idx < nodeCount)
  {
    g = s.substring(s.indexOf("DX"));
    g = g.substring(0, g.indexOf("/"));
    part = g.substring(g.indexOf("X") + 1);
    part = part.substring(0, part.indexOf("Y"));
    part.toCharArray(cnv, sizeof(cnv));
    xM = atof(cnv);
    g = g.substring(g.indexOf("Y") + 1);
    g.toCharArray(cnv, sizeof(cnv));
    yM = atof(cnv);
    s = s.substring(s.indexOf("/") + 1);
    if (mir)
    {
      xM = bW - xM;
    }
    nodes[idx].x = xM;
    nodes[idx].y = yM;
    idx++;
  }
  idx = 1;
  fire = false;

  xSp = X.drw;
  ySp = Y.drw;
  float divider = .05;
  float dv;
  float phi, psi;
  float dy, dx, dly, dlx, px, py, plx, ply;
  //Drawing nodes
  while (idx < nodeCount)
  {
    //horizontal check
    bool horiz = false;
    bool vert = false;
    bool revDir = false;
    xO = nodes[idx].x;
    yO = nodes[idx].y;
    //Serial.println("Xi:" + String(xI, DEC) + ", Yi:" + String(yI, DEC) + ", Xo:" + String(xO, DEC) + ", Yo:" + String(yO, DEC) + ", Trace Width:" + String(traceWidth, DEC));
    //delay(1000);

    if (abs(yO - yI) == 0)
    {
      horiz = true;
      //Serial.println("Horizontal");
    }
    else if (abs(xO - xI) == 0)
    {
      vert = true;
      //Serial.println("Vertical");
    }

    dv = traceWidth / divider;

    if (!horiz && !vert)
    {
      phi = atan((yO - yI) / (xO - xI));
      psi = PI - phi;
      dy = (traceWidth / 2) * sin(psi);
      dx = (traceWidth / 2) * cos(psi);
      dlx = divider * sin(psi);
      dly = divider * cos(psi);
      //Serial.println("Phi:" + String(phi, DEC) + ", Psi:" + String(psi, DEC));
    }
    else if (horiz)
    {
      dx = 0;
      dy = traceWidth / 2;
      dlx = 0;
      dly = divider;
    }
    else if (vert)
    {
      dx = traceWidth / 2;
      dy = 0;
      dlx = divider;
      dly = 0;
    }
    //Serial.println("dx:" + String(dx, DEC) + ", dy:" + String(dy, DEC) + ", dvX:" + String(dlx, DEC) + ", dvY:" + String(dly, DEC) + ", DV:" + String(dv, DEC));
    if (idx == 1)
    {
      xI -= dx;
      yI -= dy;
    }
    else
    {
      xI -= px;
      yI -= py;
    }
    xO -= dx;
    yO -= dy;


    fire = false;
    i = round((xI * X.A) - X.absPos);
    o = round((yI * Y.A) - Y.absPos);
    opStep();
    fire = true;
    i = round((xO * X.A) - X.absPos);
    o = round((yO * Y.A) - Y.absPos);
    opStep();

    for (int n = 0; n <= dv; n++)
    {
      if (idx == 1)
      {
        xI += dlx;
        yI += dly;
      }
      else
      {
        xI += plx;
        yI += ply;
      }
      xO += dlx;
      yO += dly;
      fire = true;
      i = round((xI * X.A) - X.absPos);
      o = round((yI * Y.A) - Y.absPos);
      opStep();
      fire = true;
      i = round((xO * X.A) - X.absPos);
      o = round((yO * Y.A) - Y.absPos);
      opStep();
    }

    xI = nodes[idx].x;
    yI = nodes[idx].y;
    px = dx;
    py = dy;
    plx = dlx;
    ply = dly;

    idx++;
  }

  ledcWrite(8, 0);
  s = "";
  ack = true;
  taskRunning = false;
  drawNodes = false;
  readingNodes = false;
  vTaskDelete(NULL);
}

void boreCircle(void * param)
{
  esp_task_wdt_init(720, false);
  if (toolD.f <= 0)
  {
    //Serial.println("Tool diameter not specified. Aborting");
    s = "";
    readingNodes = false;
    ack = false;
    taskRunning = false;
    vTaskDelete(NULL);
  }
  double rInner = cR - (toolD.f / 2);
  cX += spinOffX;
  cY += spinOffY;
  destinationMath(false);
  //Serial.println("Bore. toolD:" + String(toolD.f, DEC) + ", rI:" + String(rInner, DEC));
  xSp = X.mov;
  ySp = Y.mov;
  zSp = Z.mov;

  i += (A * rInner);
  r = 0;
  opStep();

  movetoHeight(millRunHeight);
  //4 engage spindle
  spindleOn = true;
  spindle.On(false);
  vacuum.On();


  circlePart = 0;
  rd = 1.000;
  xSp = X.mil;
  ySp = Y.mil;

  xI = cX;
  yI = cY;
  fire = false;
  //6 begin cycle ->> draw circle
  //Serial.println("cycle start c1");
  //delay(2000);
  zSp = Z.mil + 10;


  while (Z.absPos < millRunHeight + 1994)
  {
    if (circlePart <= 2 * PI)
    {
      circlePart += PI / 45;
      nX = rd * cos(circlePart);
      nX = xI + nX;
      nY = rd * sin(circlePart);
      nY = yI + nY;

      if (nX * X.A != X.relPos)
      {
        i = (nX * A) - X.relPos;
      }
      else
      {
        i = 0;
      }

      if (nY * Y.A != Y.relPos)
      {
        o = (nY * Y.A) - Y.relPos;
      }
      else
      {
        o = 0;
      }
      if (i != 0 || o != 0)
      {
        fire = false;
        opStep();
      }
      else
      {
        stepsDone = true;
      }
    }
    else
    {
      circlePart = 0;
      r = Z.A * .1;
    }

    //repeat until at depth
  }

  //8 retract
  //Serial.println("cycle c1 finished");
  //delay(2000);
  r = millStartHeight - Z.absPos;
  opStep();

  //9 disengage
  spindleOn = false;
  vacuum.Off();
  spindle.Off();
  xSp = X.mov;
  ySp = Y.mov;
  zSp = Z.mov;

  s = "";
  readingNodes = false;
  ack = true;
  taskRunning = false;
  vTaskDelete(NULL);
}

void pointCut(void * param)
{
  esp_task_wdt_init(720, false);
  destinationMath(false);
  opStep();
  i = spinOffX * A;
  o = spinOffY * B;
  r = 0;
  opStep();

  Z.dirHigh();
  while (Z.absPos < preDotHeight)
  {
    Z.stepHigh();
    delayMicroseconds(Z.mov );
    Z.stepLow();
    delayMicroseconds(Z.mov);
    Z.absPos++;
  }

  for (int n = rampVal; n <= spindleSpeed; n++)
  {
    ledcWrite(6, n);
    delayMicroseconds(40);
  }
  rampVal = spindleSpeed;
  zSp = Z.mil;
  xSp = X.mil;
  ySp = Y.mil;

  while (Z.absPos < dotDepth)
  {
    Z.stepHigh();
    delayMicroseconds(Z.mov * 2);
    Z.stepLow();
    delayMicroseconds(Z.mov * 2);
    Z.absPos++;
  }
  long iX = round(((cW - cX) * X.A));
  long iY = round(((cH - cY) * Y.A));
  xSp = X.mil * 3;
  ySp = Y.mil * 3;
  i = iX;
  o = iY;
  r = 0;
  opStep();

  Z.dirLow();
  while (Z.absPos > preDotHeight)
  {
    Z.stepHigh();
    delayMicroseconds(Z.mov);
    Z.stepLow();
    delayMicroseconds(Z.mov);
    Z.absPos--;
  }

  for (int n = spindleSpeed; n >= 0; n--)
  {
    ledcWrite(6, n);
    delayMicroseconds(40);
  }
  rampVal = 0;
  taskRunning = false;
  vTaskDelete(NULL);
}

void borderCut(void * param)
{
  esp_task_wdt_init(720, false);
  double cIX = cX, cIY = cY, cBW = cW, cBH = cH;
  //Serial.println("CX:" + String(cX, DEC) + ", CY:" + String(cY, DEC) + ", CW:" + String(cW, DEC) + ", CH:" + String(cH, DEC));

  homingRun(false);
  //Serial.println("Homed for border cut");

  i = (spinOffX) * X.A;
  o =  (spinOffY) * Y.A;
  r = 0;
  opStep();
  //Serial.println("Spindle tool to corner 1");
  spindle.On();
  vacuum.On();

  //delay(1000);
  zSp = zSp * 16;
  xSp = xSp * 2;
  ySp = ySp * 2;

  for (int n = 0; n < 14; n++)
  {
    i = 0;
    o = 0;
    r = .15 * Z.A;
    opStep();

    //vector3 rslt;
    //rslt = vectorSubtract(planePoints[1], planePoints[0]);

    i = cBW * X.A;
    o = 0;
    r = 0;
    opStep();

    //rslt = vectorSubtract(planePoints[2], planePoints[1]);

    i = 0;
    o = cBH * Y.A;
    r = 0;
    opStep();

    //rslt = vectorSubtract(planePoints[3], planePoints[2]);

    i = -1 * cBW * X.A;
    o = 0;
    r = 0;
    opStep();

    //rslt = vectorSubtract(planePoints[0], planePoints[3]);

    i = 0;
    o = -1 * cBH * Y.A;
    r = 0;
    opStep();
  }

  i = 0;
  o = 0;
  r = -10 * Z.A;
  spindle.Off();
  opStep();

  taskRunning = false;
  vTaskDelete(NULL);
}

void movetoHeight(signed long targetHeight)
{
  if (Z.absPos != targetHeight)
  {
    r = targetHeight - Z.absPos;
    fire = false;
    storeI = i;
    storeO = o;
    i = 0;
    o = 0;
    opStep();

    if (storeI != 0)
    {
      i = storeI;
      storeI = 0;
    }

    if (storeO != 0)
    {
      o = storeO;
      storeO = 0;
    }
    r = 0;
  }
}

void measureGap(int i)
{
  //0 = x, 1 = y, 2 = z;
  esp_task_wdt_init(720, false);

  //1) move Z to upper limit
  digitalWrite(en, LOW);
  limitMotion(4);

  if (i == 0)
  {
    limitMotion(0);

    X.dirHigh();
    for (int n = 0; n <= 5000; n++)
    {
      X.stepHigh();
      delayMicroseconds(xSp);
      X.stepLow();
      delayMicroseconds(xSp);
      X.absPos++;
    }
    while (digitalRead(xL) == LOW)
    {
      X.stepHigh();
      delayMicroseconds(xSp);
      X.stepLow();
      delayMicroseconds(xSp);
      X.absPos++;
    }

    Serial.println("X Gap Measured: " + String(X.absPos, DEC) + ", " + String((X.absPos / A), DEC));
  }
  else if (i == 1)
  {
    limitMotion(2);

    Y.dirHigh();
    for (int n = 0; n <= 5000; n++)
    {
      Y.stepHigh();
      delayMicroseconds(ySp);
      Y.stepLow();
      delayMicroseconds(ySp);
      Y.absPos++;
    }
    while (digitalRead(yL) == LOW)
    {
      Y.stepHigh();
      delayMicroseconds(ySp);
      Y.stepLow();
      delayMicroseconds(ySp);
      Y.absPos++;
    }

    Serial.println("Y Gap Measured: " + String(Y.absPos, DEC) + ", " + String((Y.absPos / B), DEC));
  }
}
