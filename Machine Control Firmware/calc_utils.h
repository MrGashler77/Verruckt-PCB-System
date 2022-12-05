extern bool mir;
float cX, cY, cZ, cW, cH, cR, cU, bW, cF, modRot, padRot, cmdW, cmdH;
long i, o, r;
extern axis X, Y, Z;
unsigned long rDx, aDx, rDy, aDy, rDz, aDz, fW, fH, fR;
extern double spinOffX, spinOffY;



double magnitude(vector3 v)
{
  double ou = sq(v.x) + sq(v.y) + sq(v.z);
  ou = sqrt(ou);
  return ou;
}

vector3 getVector3(vector3 p1, vector3 p2)
{
  vector3 ou;
  ou.x = p2.x - p1.x;
  ou.y = p2.y - p1.y;
  ou.z = p2.z - p1.z;
  return ou;
}

vector2 getVector2(vector2 p1, vector2 p2)
{
  vector2 ou;
  ou.x = p2.x - p1.x;
  ou.y = p2.y - p1.y;
  return ou;
}

vector3 getNormal(vector3 v1, vector3 v2)
{
  vector3 ou;
  ou.x = (v2.y * v1.z) - (v2.z * v1.y);
  ou.y = -1 * ((v2.x * v1.z) - (v2.z * v1.x));
  ou.z = (v2.x * v1.y) - (v2.y * v1.x);
  return ou;
}

vector3 crossProduct(vector3 v1, vector3 v2)
{
  vector3 ou;
  ou.x = (v2.y * v1.z) - (v2.z * v1.y);
  ou.y = -1 * ((v2.x * v1.z) - (v2.z * v1.x));
  ou.z = (v2.x * v1.y) - (v2.y * v1.x);
  return ou;
}

double dotProduct(vector3 v1, vector3 v2)
{
  double ou = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
  return ou;
}

double dotProduct(vector2 v1, vector2 v2)
{
  double ou = (v1.x * v2.x) + (v1.y * v2.y);
  return ou;
}

double hMapped(heightPlane h, vector2 v)
{
  double ou = (((h.h.y * (v.y - h.i.y)) - (h.h.x * (v.x - h.i.x))) / h.h.z) + h.i.z;
  return ou;
}

vector3 hMapped(heightPlane h, vector3 v)
{
  vector3 ou;
  ou.x = v.x;
  ou.y = v.y;
  ou.z = (((h.h.y * (v.y - h.i.y)) - (h.h.x * (v.x - h.i.x))) / h.h.z) + h.i.z;
  return ou;
}

vector2 rotationTransformation(float a, float b, float c)
{
  vector2 v;
  Serial.println("Rotation: " + String(c, DEC));

  if (c == 0)
  {
    v.x = a;
    v.y = b;
  }
  else if (c == 90 || c == -270)
  {
    v.x = b;
    v.y = (a * -1);
  }
  else if (c == 180 || c == -180)
  {
    v.x = (a * -1);
    v.y = (b * -1);
  }
  else if (c == 270 || c == -90)
  {
    v.x = (b * -1);
    v.y = a;
  }

  if (mir)
  {
    v.x = v.x * -1;
  }

  return v;
}

vector3 vectorSubtract(vector3 v1, vector3 v2)
{
  vector3 ou;
  ou.x = v1.x - v2.x;
  ou.y = v1.y - v2.y;
  ou.z = v1.z - v2.z;
  return ou;
}

vector3 vectorAdd(vector3 v1, vector3 v2)
{
  vector3 ou;
  ou.x = v1.x + v2.x;
  ou.y = v1.y + v2.y;
  ou.z = v1.z + v2.z;
  return ou;
}

void destinationMath(bool rel)
{
  if (rel)
  {
    //Serial.println("Relative destination call");
  }
  else
  {
    //Serial.println("Absolute destination call");
  }

  if (cX != -100000)
  {

    if (!rel)
    {
      if (mir)
      {
        cX = bW - cX;
      }
      i = round((cX * X.A) - X.absPos);
    }
    else
    {
      if (X.relZero != 0)
      {
        rDx = round((cX * X.A));
        aDx = X.relZero + rDx;
        i = aDx - X.absPos;
      }
      else
      {
        i = round(cX * X.A);
      }
    }
  }
  else
  {
    i = 0;
  }

  if (cY != -100000)
  {
    if (!rel)
    {
      o = round((cY * Y.A) - Y.absPos);
    }
    else
    {
      if (Y.relZero != 0)
      {
        rDy = round(cY * Y.A);
        aDy = Y.relZero + rDy;
        o = aDy - Y.absPos;
      }
      else
      {
        o = round(cY * Y.A);
      }
    }
  }
  else
  {
    o = 0;
  }

  if (cZ != -100000)
  {
    if (!rel)
    {
      r = round((cZ * Z.A) - Z.absPos);
    }
    else
    {
      if (Z.relZero != 0)
      {
        rDz = round(cZ * Z.A);
        aDz = Z.relZero + rDz;
        r = aDz - Z.absPos;
      }
      else
      {
        r = round(cZ * Z.A);
      }
    }
  }
  else
  {
    r = 0;
  }

  if (cW != -100000)
  {
    if (cW > .25)
    {
      fW = round(X.A * (cW - .2));
    }
    else
    {
      fW = round(X.A * cW);
    }
    fW = labs(fW);
  }
  else
  {
    fW = 0;
  }

  if (cH != -100000)
  {
    if (cH > .25)
    {
      fH = round(Y.A * (cH - .2));
    }
    else
    {
      fH = round(Y.A * cH);
    }
    fH = labs(fH);
  }
  else
  {
    fH = 0;
  }

  if (cR != -100000)
  {
    fR = round(X.A * cR);
    fR = labs(fR);
  }
  else
  {
    fR = 0;
  }
}

bool stateToBool(uint8_t p)
{
  uint32_t input = GPIO_REG_READ(GPIO_OUT_REG);

  if ((input >> p) & 1U == HIGH)
  {
    return true;
  }
  else
  {
    return false;
  }

}

void padLogic(bool circ, bool conn)
{
  float xN, yN, wNm, hNm;
  //Serial.println("Pad Logic. Initial values are: x:" + String(cX, DEC) + ", y:" + String(cY, DEC) + ", w:" + String(cW, DEC) + ", h:" + String(cH, DEC) + ", Module Rotation:" + String(modRot, DEC) + ", Pad Rotation:" + String(padRot, DEC));
  bool xP = false, yP = false;
  struct vector2 newLoc = rotationTransformation(cX, cY, modRot);
  xN = newLoc.x;
  yN = newLoc.y;
  if (xN > 0)
  {
    xP = true;
  }
  else
  {
    xP = false;
  }

  if (yN > 0)
  {
    yP = true;
  }
  else
  {
    yP = false;
  }
  //Serial.println("Modified Pad Center Location. X:" + String(xN, DEC) + ", Y:" + String(yN, DEC));

  if (cW != -100000 && cH != -100000)
  {
    newLoc = rotationTransformation(cW, cH, padRot);
    wNm = abs(newLoc.x);
    hNm = abs(newLoc.y);
    //Serial.println("Modified Pad Dimensions. W:" + String(wNm, DEC) + ", H:" + String(hNm, DEC));


    if (cW > .01 && cH > .01)
    {
      cX = (xN - (wNm / 2.0)) + .1;
      cY = (yN - (hNm / 2.0)) + .1;
      cW = wNm;
      cH = hNm;
    }
    else
    {
      if (!conn)
      {
        cX = (xN - (wNm / 2)) + .1;
        cY = (yN - (hNm / 2)) + .1;
        cW = wNm;
        cH = hNm;
      }
      else
      {
        cX = xN;
        cY = yN;
        cW = wNm / 2;
        cH = hNm / 2;
        if (cW > cH && xP)
        {
          cX -= cW;
          cW += .1;
        }
        else
        {
          cX = xN;
        }

        if (cH > cW && yP)
        {
          cY -= cH;
          cH += .1;
        }
        else
        {
          cY = yN;
        }
      }
      //Serial.println("Connected Pad Processed. X:" + String(cX, DEC) + ", Y:" + String(cY, DEC) + ", W:" + String(cW, DEC) + ", H:" + String(cH, DEC));
    }
  }
  else if (cR != -100000)
  {
    //Serial.println("RelX:" + String((X.relZero / X.A), DEC) + ", RelY:" + String((Y.relZero / Y.A), DEC));
    cX = ((float)X.relZero / X.A) + xN;
    cY = ((float)Y.relZero / Y.A) + yN;
  }

  //Serial.println("Pad Logic. Results are: x:" + String(cX, DEC) + ", y:" + String(cY, DEC) + ", w:" + String(cW, DEC) + ", h:" + String(cH, DEC));
}

void parseValues(String g)
{
  String j = "";
  if (g.indexOf('X') >= 0)
  {
    for (int n = g.indexOf('X') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.' || g[n] == '-')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cX = atof(cnv);
        break;
      }
    }

    if (cX == 0 || cX == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cX = atof(cnv);
    }
  }
  else
  {
    cX = -100000;
  }

  if (g.indexOf('Y') >= 0)
  {
    j = "";
    for (int n = g.indexOf('Y') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.' || g[n] == '-')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cY = atof(cnv);
        break;
      }
    }

    if (cY == 0 || cY == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cY = atof(cnv);

    }
  }
  else
  {
    cY = -100000;
  }

  if (g.indexOf('Z') >= 0)
  {
    j = "";
    for (int n = g.indexOf('Z') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.' || g[n] == '-')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cZ = atof(cnv);
        break;
      }
    }

    if (cZ == 0 || cZ == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cZ = atof(cnv);
    }
  }
  else
  {
    cZ = -100000;
  }

  if (g.indexOf('W') >= 0)
  {
    j = "";
    for (int n = g.indexOf('W') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cW = atof(cnv);
        break;
      }
    }

    if (cW == 0 || cW == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cW = atof(cnv);
      //Serial.println("AGAIN j:" + j + ", cW:" + String(cW, DEC));
    }
  }
  else
  {
    cW = -100000;
  }

  if (g.indexOf('H') >= 0)
  {
    j = "";
    for (int n = g.indexOf('H') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cH = atof(cnv);
        break;
      }
    }

    if (cH == 0 || cH == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cH = atof(cnv);
      //Serial.println("AGAIN j:" + j + ", cH:" + String(cH, DEC));
    }
  }
  else
  {
    cH = -100000;
  }

  if (g.indexOf('R') >= 0)
  {
    j = "";
    for (int n = g.indexOf('R') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cR = atof(cnv);
        break;
      }
    }

    if (cR == 0 || cR == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cR = atof(cnv);
      //Serial.println("AGAIN j:" + j + ", cR:" + String(cR, DEC));
    }
  }
  else
  {
    cR = -100000;
  }

  if (g.indexOf('U') >= 0)
  {
    j = "";
    for (int n = g.indexOf('U') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.' || g[n] == '-')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cU = atof(cnv);
        //Serial.println("AGAIN j:" + j + ", cU:" + String(cU, DEC));
        break;
      }
    }

    if (cU == 0 || cU == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cU = atof(cnv);
      //Serial.println("AGAIN j:" + j + ", cU:" + String(cU, DEC));
    }
  }
  else
  {
    cU = -100000;
  }

  if (g.indexOf('F') >= 0)
  {
    j = "";
    for (int n = g.indexOf('F') + 1; n < g.length(); n++)
    {
      if (isDigit(g[n]) || g[n] == '.' || g[n] == '-')
      {
        j += g[n];
      }
      else
      {
        char cnv[40];
        j.toCharArray(cnv, sizeof(cnv));
        cF = atof(cnv);
        //Serial.println("AGAIN j:" + j + ", cU:" + String(cU, DEC));
        break;
      }
    }

    if (cF == 0 || cF == -100000)
    {
      char cnv[40];
      j.toCharArray(cnv, sizeof(cnv));
      cF = atof(cnv);
      //Serial.println("AGAIN j:" + j + ", cU:" + String(cU, DEC));
    }
  }
  else
  {
    cF = -100000;
  }
}
