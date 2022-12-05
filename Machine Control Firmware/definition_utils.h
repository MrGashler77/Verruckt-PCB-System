#include <ESP32Servo.h>

extern double xLm, yLm, zLm;
extern bool fire;
extern bool stepsDone;
extern bool drawNodes;
extern int xSp, ySp, zSp;
extern int lzrPwr;

struct axis
{
  const int stp;
  const int dir;
  const int lim;
  float mov;
  float drw;
  float mil;
  double A;
  double iA;
  unsigned long relPos;
  unsigned long absPos;
  unsigned long relZero;

  void initAxis(void)
  {
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(lim, INPUT);
    stepLow();
    dirLow();
  }

  uint8_t stepRead(void)
  {
    if (stp < 32) {
      return (GPIO.in >> stp) & 0x1;
    } else if (stp < 40) {
      return (GPIO.in1.val >> (stp - 32)) & 0x1;
    }
    return 0;
  }

  uint8_t dirRead(void)
  {
    if (dir < 32) {
      return (GPIO.in >> dir) & 0x1;
    } else if (dir < 40) {
      return (GPIO.in1.val >> (dir - 32)) & 0x1;
    }
    return 0;
  }

  uint8_t limRead(void)
  {
    if (lim < 32) {
      return (GPIO.in >> lim) & 0x1;
    } else if (lim < 40) {
      return (GPIO.in1.val >> (lim - 32)) & 0x1;
    }
    return 0;
  }

  void stepHigh()
  {
    if (stp < 32)
    {
      GPIO.out_w1ts = ((uint32_t)1 << stp);
    }
    else
    {
      GPIO.out1_w1ts.val = ((uint32_t)1 << (stp - 32));
    }
  }

  void stepLow()
  {
    if (stp < 32)
    {
      GPIO.out_w1tc = ((uint32_t)1 << stp);
    }
    else
    {
      GPIO.out1_w1tc.val = ((uint32_t)1 << (stp - 32));
    }
  }

  void dirHigh()
  {
    if (dir < 32)
    {
      GPIO.out_w1ts = ((uint32_t)1 << dir);
    }
    else
    {
      GPIO.out1_w1ts.val = ((uint32_t)1 << (dir - 32));
    }
  }

  void dirLow()
  {
    if (dir < 32)
    {
      GPIO.out_w1tc = ((uint32_t)1 << dir);
    }
    else
    {
      GPIO.out1_w1tc.val = ((uint32_t)1 << (dir - 32));
    }
  }
};

struct vector3
{
  double x;
  double y;
  double z;

  vector3(double i, double o, double r)
  {
    x = i;
    y = o;
    z = r;
  }

  vector3()
  {
    x = 0;
    y = 0;
    z = 0;
  }
};

struct vectorI3
{
  long x;
  long y;
  long z;
};

struct vector2
{
  double x;
  double y;
};

struct vectorI2
{
  long x;
  long y;
};

struct heightPlane
{
  vector3 h;
  vector3 i;
  vector2 lowBound;
  vector2 highBound;

  bool pointInBounds(vector3 v)
  {
    if (v.x >= lowBound.x && v.x <= highBound.x && v.y >= lowBound.y && v.y <= highBound.y)
    {
      return true;
    }
    else
    {
      false;
    }
  }

  double lerpedHeight(vector3 v)
  {
    return i.z + (((h.y * (v.y - i.y)) - (h.x * (v.x - i.x))) / h.z);
  }
};

typedef union floatToByte
{
  float f;
  byte b[4];
} fTB;

typedef union intToByte
{
  int i;
  byte b[4];
} iTB;

typedef union ulongTob
{
  int u;
  byte b[4];
} uTb;

class axisV2class
{
  public:
    int stp;
    int dir;
    int lim;
    int chan;
    double res;
    double duty;
    double mov;
    double drw;
    double mil;
    double A;
    double iA;
    unsigned long relPos;
    unsigned long absPos;
    unsigned long relZero;

    axisV2class(int s, int d, int l, int c, int m, int dr, int ml, double as)
    {
      stp = s;
      dir = d;
      lim = l;
      chan = c;
      mov = (double)(1000000 / m);
      drw = (double)(1000000 / dr);
      mil = (double)(1000000 / ml);
      A = as;
      iA = 1 / A;
      relPos = 0;
      absPos = 0;
      relZero = 0;
    }

    void initAxis(void)
    {
      pinMode(stp, OUTPUT);
      pinMode(dir, OUTPUT);
      pinMode(lim, INPUT_PULLUP);
      digitalWrite(stp, LOW);
      dirLow();
    }

    uint8_t dirRead(void)
    {
      uint32_t input = GPIO_REG_READ(GPIO_OUT_REG);

      return (input >> dir) & 1U;
    }

    uint8_t limRead(void)
    {
      uint32_t input = GPIO_REG_READ(GPIO_IN_REG);

      return (input >> lim) & 1U;
    }

    void startAxis(double sp)
    {
      if (sp > 0)
      {
        if (sp > 40000000)
        {
          sp = 40000000;
        }
        res = (log(80000000 / sp) / log(2));
        duty = pow(2, res) / 2;
        ledcSetup(chan, sp, res);
        ledcAttachPin(stp, chan);
        ledcWrite(chan, duty);
      }
    }

    void stopAxis()
    {
      ledcWrite(chan, 0);
    }

    void dirHigh()
    {
      GPIO.out_w1ts = ((uint32_t)1 << dir);
    }

    void dirLow()
    {
      GPIO.out_w1tc = ((uint32_t)1 << dir);
    }
};

class motionAxes
{
  public:
    axisV2class *axes[3] = {NULL, NULL, NULL};

    motionAxes(axisV2class a, axisV2class b, axisV2class c)
    {
      axes[0] =  &a;
      axes[1] =  &b;
      axes[2] =  &c;
    }

    void initializeAxes()
    {
      for (int n = 0; n < 3; n++)
      {
        axes[n]->initAxis();
      }
    }

    void moveAxes(long i, long o, long r)
    {
      stepsDone = false;
      double xT = 0, yT = 0, zT = 0, waitX, waitY, waitZ, sT;
      unsigned long curT, startT;
      bool xr = false, yr = false, zr = false;
      xLm = 1000000 / (double)(2 * xSp);
      yLm = 1000000 / (double)(2 * ySp);
      zLm = 1000000 / (double)(2 * zSp);
      if (i > 0)
      {
        axes[0]->dirHigh();
      }
      else
      {
        axes[0]->dirLow();
      }

      if (o > 0)
      {
        axes[1]->dirHigh();
      }
      else
      {
        axes[1]->dirLow();
      }

      if (r > 0)
      {
        axes[2]->dirHigh();
      }
      else
      {
        axes[2]->dirLow();
      }

      if (abs(i) >= abs(o))
      {
        if (abs(i) > 0)
        {
          sT = ((double)(abs(i) / xLm)) * 1000000;
          xr = true;
          waitX = xLm;
          xT = sT;
        }
        else
        {
          waitX = 0;
          xT = 0;
        }

        if (abs(o) > 0)
        {
          waitY = (abs(o) * xLm) / abs(i);
          yr = true;
          yT = xT;
        }
        else
        {
          waitY = 0;
          yT = 0;
        }
      }
      else if (abs(o) > abs(i))
      {
        sT = ((double)(abs(o) / yLm)) * 1000000;
        yr = true;
        waitY = yLm;
        yT = sT;

        if (abs(i) > 0)
        {
          waitX = (abs(i) * yLm) / abs(o);
          xr = true;
          xT = sT;
        }
        else
        {
          waitX = 0;
          xT = 0;
        }
      }

      if (abs(r) > 0)
      {
        waitZ = zLm;
        zr = true;
        zT = (abs(r) / zLm) * 1000000;
      }
      else
      {
        waitZ = 0;
        zT = 0;
      }


      if (fire)
      {
        ledcWrite(8, lzrPwr);
      }

      axes[0]->startAxis(waitX);
      axes[1]->startAxis(waitY);
      axes[2]->startAxis(waitZ);
      startT = micros();
      while (xr || yr || zr)
      {
        curT = micros();
        if (xr && (curT - startT >= xT))
        {
          axes[0]->stopAxis();
          xr = false;
        }

        if (yr && (curT - startT >= yT))
        {
          axes[1]->stopAxis();
          yr = false;
        }

        if (zr && (curT - startT >= zT))
        {
          axes[2]->stopAxis();
          zr = false;
        }
      }

      stepsDone = true;
      axes[0]->relPos += i;
      axes[0]->absPos += i;
      axes[1]->relPos += o;
      axes[1]->absPos += o;
      axes[2]->relPos += r;
      axes[2]->absPos += r;

      if (fire && !drawNodes)
      {
        ledcWrite(8, 0);
      }
    }
};

class tool
{
  public:
    int pin;
    int en;
    int pwmFreq;
    int pwmChan;
    bool ramping;
    bool fluxed;
    int runVal;
    int curVal;
    bool inverted;
    bool on;

    tool(int p, int f, int c, bool r, int v)
    {
      pin = p;
      pwmFreq = f;
      pwmChan = c;
      ramping = r;
      fluxed = false;
      runVal = v;
      curVal = 0;
      inverted = false;
      on = false;
    }

    tool (int p, int e, int f, int c, int r, int v)
    {
      pin = p;
      en = e;
      pwmFreq = f;
      pwmChan = c;
      ramping = r;
      fluxed = false;
      runVal = v;
      curVal = 0;
      inverted = false;
      on = false;
    }

    tool(int p, bool b)
    {
      pin = p;
      pwmFreq = 0;
      pwmChan = 0;
      ramping = false;
      fluxed = true;
      runVal = 0;
      curVal = 0;
      inverted = b;
      on = false;
    }

    void toolInit()
    {
      pinMode(pin, OUTPUT);

      if (en != 0)
      {
        //Serial.println("Enable Pin: " + String(en, DEC));
        pinMode(en, OUTPUT);
        //Serial.println("Initiated Spindle Tool");
      }

      if (!fluxed)
      {
        ledcSetup(pwmChan, pwmFreq, 8);
        ledcAttachPin(pin, pwmChan);
        ledcWrite(pwmChan, 0);
      }
      else
      {
        if (!inverted)
        {
          digitalWrite(pin, LOW);

          if (en != 0)
          {
            digitalWrite(en, LOW);
          }
        }
        else
        {
          digitalWrite(pin, HIGH);
          if (en != 0)
          {
            digitalWrite(en, HIGH);
          }
        }
      }

      curVal = 0;
    }

    void On()
    {
      if (en != 0)
      {
        if (!inverted)
        {
          digitalWrite(en, HIGH);
          //Serial.println("Set EN High");
        }
        else
        {
          digitalWrite(en, LOW);
        }
      }

      if (ramping)
      {
        if (curVal < runVal)
        {
          for (int n = curVal; n <= runVal; n++)
          {
            ledcWrite(pwmChan, n);
            delay(5);
          }
          curVal = runVal;
        }
      }
      else
      {
        if (!fluxed)
        {
          ledcWrite(pwmChan, runVal);
          curVal = runVal;
        }
        else
        {
          if (!inverted)
          {
            digitalWrite(pin, HIGH);
          }
          else
          {
            digitalWrite(pin, LOW);
          }
        }
      }
      on = true;
    }

    void On(int i)
    {
      if (en != 0)
      {
        if (!inverted)
        {
          digitalWrite(en, HIGH);
        }
        else
        {
          digitalWrite(en, LOW);
        }
      }

      if (ramping)
      {
        int diff = i - curVal;
        //Serial.println("DIFF:" + String(diff, DEC) + ", D/D:" + String(diff / abs(diff), DEC));
        if (diff > 0)
        {
          for (int n = curVal; n <= i; n++)
          {
            ledcWrite(pwmChan, n);
            //Serial.println("Loop val:" + String(n, DEC));
            delay(25);
          }
        }
        else if (diff < 0)
        {
          for (int n = curVal; n >= i; n--)
          {
            ledcWrite(pwmChan, n);
            //Serial.println("Loop val:" + String(n, DEC));
            delay(25);
          }
        }
        curVal = i;
        //Serial.println("Tool set to: " + String(i, DEC));
      }
      else
      {
        if (!fluxed)
        {
          ledcWrite(pwmChan, i);
          curVal = i;
          //Serial.println("Tool set to: " + String(i, DEC));
        }
        else
        {
          if (!inverted)
          {
            digitalWrite(pin, HIGH);
            //Serial.println("Tool set to: HIGH");
          }
          else
          {
            digitalWrite(pin, LOW);
            //Serial.println("Tool set to: LOW");
          }
        }
      }
      on = true;
    }

    void Off()
    {
      if (en != 0)
      {
        if (!inverted)
        {
          digitalWrite(en, LOW);
          //Serial.println("Set EN Low");
        }
        else
        {
          digitalWrite(en, HIGH);
        }

        //Serial.println("Enable Pin: " + String(en, DEC));
      }
      if (ramping)
      {
        if (curVal > 0)
        {
          for (int n = curVal; n >= 0; n--)
          {
            ledcWrite(pwmChan, n);
            delay(5);
          }
          curVal = 0;
        }
      }
      else
      {
        if (!fluxed)
        {
          ledcWrite(pwmChan, 0);
          curVal = 0;
        }
        else
        {
          if (!inverted)
          {
            digitalWrite(pin, LOW);
          }
          else
          {
            digitalWrite(pin, HIGH);
          }
        }
      }
      on = false;
    }
};

class toolProbe
{
  public:
    int pin;
    bool inverted;

    toolProbe(int p, bool b)
    {
      pin = p;
      inverted = b;
    }

    void initToolProbe()
    {
      if (!inverted)
      {
        pinMode(pin, INPUT_PULLUP);
      }
      else
      {
        pinMode(pin, INPUT);
      }
    }

    bool getProbe()
    {
      if (!inverted)
      {
        if (digitalRead(pin) == HIGH)
        {
          return false;
        }
        else
        {
          return true;
        }
      }
      else
      {
        if (digitalRead(pin) == HIGH)
        {
          return true;
        }
        else
        {
          return false;
        }
      }
    }
};

struct dataOut
{
  unsigned long xPs;
  float xSM;
  byte xEN;
  byte xDir;
  byte xLim;
  unsigned long yPs;
  float ySM;
  byte yEN;
  byte yDir;
  byte yLim;
  unsigned long zPs;
  float zSM;
  byte zEN;
  byte zDir;
  byte zLim;
  int laserPower;
  int spindlePower;
  byte vacState;
};
