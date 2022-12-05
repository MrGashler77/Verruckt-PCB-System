#include <ps4.h>
#include <PS4Controller.h>
#include <ps4_int.h>

bool antiLatch = false;
extern long i, o, r;
extern const float A, B, C;
extern tool spindle;
extern tool laser;
bool lDB = false;
bool sDB = false;
bool oDB = false;
int mlt = 1;

bool checkController()
{
  return PS4.isConnected();
}

void connectController()
{
  PS4.begin("00:1a:7d:da:71:13");
}

void debugController()
{
  // Below has all accessible outputs from the controller
  if (PS4.isConnected()) {
    if (PS4.Right()) Serial.print("Right Button ");
    if (PS4.Down()) Serial.print("Down Button ");
    if (PS4.Up()) Serial.print("Up Button ");
    if (PS4.Left()) Serial.print("Left Button ");
    Serial.println();

    if (PS4.Square()) Serial.print("Square Button ");
    if (PS4.Cross()) Serial.print("Cross Button ");
    if (PS4.Circle()) Serial.print("Circle Button ");
    if (PS4.Triangle()) Serial.print("Triangle Button ");
    Serial.println();

    if (PS4.UpRight()) Serial.print("Up Right ");
    if (PS4.DownRight()) Serial.print("Down Right ");
    if (PS4.UpLeft()) Serial.print("Up Left ");
    if (PS4.DownLeft()) Serial.print("Down Left ");
    Serial.println();

    if (PS4.L1()) Serial.print("L1 Button ");
    if (PS4.R1()) Serial.print("R1 Button");
    Serial.println();

    if (PS4.Share()) Serial.print("Share Button ");
    if (PS4.Options()) Serial.print("Options Button ");
    if (PS4.L3()) Serial.print("L3 Button ");
    if (PS4.R3()) Serial.print("R3 Button ");
    Serial.println();

    if (PS4.PSButton()) Serial.print("PS Button ");
    if (PS4.Touchpad()) Serial.print("Touch Pad Button ");
    Serial.println();

    Serial.printf("L2 button at %d\n", PS4.L2Value());
    Serial.printf("R2 button at %d\n", PS4.R2Value());
    Serial.println();

    Serial.printf("Left Stick x at %d\n", PS4.LStickX());
    Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    Serial.println();
    Serial.printf("Right Stick x at %d\n", PS4.RStickX());
    Serial.printf("Right Stick y at %d\n", PS4.RStickY());
    Serial.println();

    if (PS4.Charging()) Serial.println("The controller is charging");
    if (PS4.Audio()) Serial.println("The controller has headphones attached");
    if (PS4.Mic()) Serial.println("The controller has a mic attached");

    Serial.printf("Battery Level : %d\n", PS4.Battery());

    Serial.println();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(1000);
  }
}

bool controllerInput()
{
  if (PS4.isConnected())
  {
    if (PS4.L3())
    {
      if (!lDB)
      {
        lDB = true;
      }
      else
      {
        if (laser.on)
        {
          laser.Off();
        }
        else
        {
          laser.On();
        }
        lDB = false;
      }
      return false;
    }
    else if (PS4.R3())
    {
      if (!sDB)
      {
        sDB = true;
      }
      else
      {
        if (spindle.on)
        {
          spindle.Off();
        }
        else
        {
          spindle.On();
        }
        sDB = false;
      }
      return false;
    }
    else if (PS4.Options())
    {
      if (!oDB)
      {
        oDB = true;
      }
      else
      {
        oDB = false;
        i = 0;
        r = 0;
        o = 0;
        spindle.Off();
        spindle.Off();
        return true;
      }
    }
    else
    {
      if (PS4.R1())
      {
        mlt = 100;
      }
      else if (PS4.R2())
      {
        mlt = 10;
      }
      else
      {
        mlt = 1;
      }

      if (PS4.L2())
      {
        r = (C * (mlt * .01));
      }
      else if (PS4.L1())
      {
        r = (-1 * C * (mlt * .01));
      }
      else
      {
        if (PS4.Up() || PS4.UpRight() || PS4.UpLeft())
        {
          o = (B * (mlt * .01));
        }

        if (PS4.Down() || PS4.DownRight() || PS4.DownLeft())
        {
          o = (-1 * B * (mlt * .01));
        }


        if (PS4.Right() || PS4.UpRight() || PS4.DownRight())
        {
          i = (A * (mlt * .01));
        }

        if (PS4.Left() || PS4.UpLeft() || PS4.DownLeft())
        {
          i = (-1 * A * (mlt * .01));
        }
      }

      if (i != 0 || o != 0 || r != 0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}
