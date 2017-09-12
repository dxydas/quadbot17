#include <avr/pgmspace.h>
#include <math.h>
#include <ax12.h>


// Target arrays - Split into quadrants
const float UpDownQs[2][4][25] PROGMEM = {
  {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.8, 0.6, 0.4, 0.2, 0}
  },
  {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0.077, 0.154, 0.231, 0.308, 0.385, 0.462, 0.538, 0.615, 0.692, 0.769, 0.846, 0.923, 1, 1, 1, 1, 1, 1, 0.923, 0.846, 0.769, 0.692, 0.615, 0.538},
    {0.4615384615, 0.3846153846, 0.3076923077, 0.2307692308, 0.1538461538, 0.0769230769, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  }
};
const float FwdBackQs[2][4][25] PROGMEM = {
  {
    {1, 1, 1, 1, 1, 1, 0.986, 0.971, 0.957, 0.942, 0.928, 0.913, 0.899, 0.884, 0.87, 0.855, 0.841, 0.826, 0.812, 0.797, 0.783, 0.768, 0.754, 0.739, 0.725},
    {0.71, 0.696, 0.681, 0.667, 0.652, 0.638, 0.623, 0.609, 0.594, 0.58, 0.565, 0.551, 0.536, 0.522, 0.507, 0.493, 0.478, 0.464, 0.449, 0.435, 0.42, 0.406, 0.391, 0.377, 0.362},
    {0.348, 0.333, 0.319, 0.304, 0.29, 0.275, 0.261, 0.246, 0.232, 0.217, 0.203, 0.188, 0.174, 0.159, 0.145, 0.13, 0.116, 0.101, 0.087, 0.072, 0.058, 0.043, 0.029, 0.014, 0},
    {0, 0, 0, 0, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1}
  },
  {
    {0.710, 0.696, 0.681, 0.667, 0.652, 0.638, 0.623, 0.609, 0.594, 0.580, 0.565, 0.551, 0.536, 0.522, 0.507, 0.493, 0.478, 0.464, 0.449, 0.435, 0.420, 0.406, 0.391, 0.377, 0.362},
    {0.348, 0.333, 0.319, 0.304, 0.290, 0.275, 0.261, 0.246, 0.232, 0.217, 0.203, 0.188, 0.174, 0.159, 0.145, 0.130, 0.116, 0.101, 0.087, 0.072, 0.058, 0.043, 0.029, 0.014, 0},
    {0, 0, 0, 0, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1},
    {1, 1, 1, 1, 1, 1, 0.986, 0.971, 0.957, 0.942, 0.928, 0.913, 0.899, 0.884, 0.870, 0.855, 0.841, 0.826, 0.812, 0.797, 0.783, 0.768, 0.754, 0.739, 0.725}
  }
};





//float UpDownAll[100];
//float FwdBackAll[100];

int g;
int G = 2;   // Num. of gaits
int Q = 4;   // Num. of quadrants
int N = 25;  // Num. of points
float amplAdjust = 50;

float target[3];
float targetHome[3] = {232.2870506213818, 0.0, -2.7432364936986176};
float joints[5] = {0, 0, 0, 0, 0};
float xAdjustInLegBase = -20;
float zAdjustInLegBase = -20;


void setup()
{
  Serial.begin(38400);
}


void loop()
{
  // Creep gait
  g = 0;
  for (int i = 0; i < Q; ++i)
    for (int j = 0; j < N; ++j)
      {
        // Front Left
        setTarget(g, i, j);

        // Front Right
        setTarget(g, (i+2)%Q, j);

        // Rear Left
        setTarget(g, (i+1)%Q, j);

        // Rear Right
        setTarget(g, (i+3)%Q, j);
     }

  // Walk gait
  g = 1;
  for (int i = 0; i < Q; ++i)
    for (int j = 0; j < N; ++j)
      {
        // Front Left
        setTarget(g, i, j);

        // Front Right
        setTarget(g, (i+2)%Q, j);

        // Rear Left
        setTarget(g, (i+2)%Q, j);

        // Rear Right
        setTarget(g, i, j);
     }

/*
  for (int i = 0; i < 100; ++i)
  {
    Serial.print(UpDownAll[i]);
    Serial.print(" ");
  }
  Serial.println("");

  for (int i = 0; i < 100; ++i)
  {
    Serial.print(FwdBackAll[i]);
    Serial.print(" ");
  }
  Serial.println("");
*/
}


void setTarget(int g, int i, int j)
{
  float UpDown = amplAdjust * pgm_read_float_near(&(UpDownQs[g][i][j]));
  float FwdBack = amplAdjust * pgm_read_float_near(&(FwdBackQs[g][i][j]));

  target[0] = targetHome[0] - UpDown + xAdjustInLegBase;
  target[1] = targetHome[1];
  target[2] = targetHome[2] + FwdBack + zAdjustInLegBase;

  runIK(target, joints);
/*
  Serial.println("target:");
  for (int i = 0; i < 3; ++i)
  {
    Serial.print(target[i]);
    Serial.print(" ");
  }
  Serial.println("");

  Serial.println("joints:");
  for (int i = 0; i < 5; ++i)
  {
    Serial.print(joints[i]);
    Serial.print(" ");
  }
  Serial.println("");
*/
}


// IK - get joint angles from foot target
void runIK(const float targetInLegBase[], float angles[])
{
  float num, den;
  float a0Rads, c0, a2p, a3p, a4p, a5p;
  float j4Height, j2j4DistSquared, j2j4Dist;
  float phi, psi, omega;
  float footOffset = 33.596;
  float a[6] = {0, 0, 29.05, 76.919, 72.96, 45.032};  // Link lengths "a-1"

  // Solve Joint 1
  num = targetInLegBase[1];
  den = abs(targetInLegBase[0]) - footOffset;
  a0Rads = atan2(num, den);
  angles[0] = a0Rads * 180.0 / M_PI;

  // Lengths projected onto z-plane
  c0 = cos(a0Rads);
  a2p = a[2]*c0;
  a3p = a[3]*c0;
  a4p = a[4]*c0;
  a5p = a[5]*c0;

  j4Height = abs(targetInLegBase[0]) - a2p - a5p - footOffset;

  j2j4DistSquared = pow(j4Height, 2) + pow(targetInLegBase[2], 2);
  j2j4Dist = sqrt(j2j4DistSquared);

  // Solve Joint 2
  num = targetInLegBase[2];
  den = j4Height;
  psi = atan2(num, den) * 180.0 / M_PI;

  num = pow(a3p, 2) + j2j4DistSquared - pow(a4p, 2);
  den = 2.0*a3p*j2j4Dist;
  if (abs(num) <= abs(den))
  {
    phi = acos(num/den) * 180.0 / M_PI;
    angles[1] = - (phi - psi);
  }

  // Solve Joint 3
  num = pow(a3p, 2) + pow(a4p, 2) - j2j4DistSquared;
  den = 2.0*a3p*a4p;
  if (abs(num) <= abs(den))
    angles[2] = 180.0 - acos(num/den) * 180.0 / M_PI;

  // Solve Joint 4
  num = pow(a4p, 2) + j2j4DistSquared - pow(a3p, 2);
  den = 2.0*a4p*j2j4Dist;
  if (abs(num) <= abs(den))
  {
    omega = acos(num/den) * 180.0 / M_PI;
    angles[3] = - (psi + omega);
  }

  // Solve Joint 5
  angles[4] = - angles[0];
}


