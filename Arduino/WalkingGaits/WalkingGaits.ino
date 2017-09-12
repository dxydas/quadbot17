#include <avr/pgmspace.h>
#include <math.h>
#include <ax12.h>


// Target arrays - Split into quadrants
<<<<<<< HEAD
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
=======
const float UpDownQs[4][25] PROGMEM = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0.2,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.8,0.6,0.4,0.2,0}
};
const float FwdBackQs[4][25] PROGMEM = {
  {1,1,1,1,1,1,0.9855072464,0.9710144928,0.9565217391,0.9420289855,0.9275362319,0.9130434783,0.8985507246,0.884057971,0.8695652174,0.8550724638,0.8405797101,0.8260869565,0.8115942029,0.7971014493,0.7826086957,0.768115942,0.7536231884,0.7391304348,0.7246376812},
  {0.7101449275,0.6956521739,0.6811594203,0.6666666667,0.652173913,0.6376811594,0.6231884058,0.6086956522,0.5942028986,0.5797101449,0.5652173913,0.5507246377,0.5362318841,0.5217391304,0.5072463768,0.4927536232,0.4782608696,0.4637681159,0.4492753623,0.4347826087,0.4202898551,0.4057971014,0.3913043478,0.3768115942,0.3623188406},
  {0.347826087,0.3333333333,0.3188405797,0.3043478261,0.2898550725,0.2753623188,0.2608695652,0.2463768116,0.231884058,0.2173913043,0.2028985507,0.1884057971,0.1739130435,0.1594202899,0.1449275362,0.1304347826,0.115942029,0.1014492754,0.0869565217,0.0724637681,0.0579710145,0.0434782609,0.0289855072,0.0144927536,0},
  {0,0,0,0,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1}
};
>>>>>>> 4d24b1ff0c1d53eeefe66d86b40296d188b75787


void setup()
{
  Serial.begin(38400);
}

<<<<<<< HEAD

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
=======
  float UpDownAll[100];
  float FwdBackAll[100];

  int Q = 4;
  int N = 25;
  float amplAdjust = 50;
  int offset;
  for (int i = 0; i < Q; ++i)
    for (int j = 0; j < N; ++j)
      {
        // FL
        //offset = i*N;
        //UpDownAll[j+offset] = amplAdjust * pgm_read_float_near(&(UpDownQs[i][j]));
        //FwdBackAll[j+offset] = amplAdjust * pgm_read_float_near(&(FwdBackQs[i][j]));
        offset = i;
        UpDownAll[j+i*N] = amplAdjust * pgm_read_float_near(&(UpDownQs[offset][j]));
        FwdBackAll[j+i*N] = amplAdjust * pgm_read_float_near(&(FwdBackQs[offset][j]));

        // FR
        //offset = (i+2)%Q*N;
        //UpDownAll[j+offset] = amplAdjust * pgm_read_float_near(&(UpDownQs[i][j]));
        //FwdBackAll[j+offset] = amplAdjust * pgm_read_float_near(&(FwdBackQs[i][j]));
        offset = (i+2)%Q;
//        UpDownAll[j+i*N] = amplAdjust * pgm_read_float_near(&(UpDownQs[offset][j]));
//        FwdBackAll[j+i*N] = amplAdjust * pgm_read_float_near(&(FwdBackQs[offset][j]));

        // RL
        //offset = (i+3)%Q*N;
        //UpDownAll[j+offset] = amplAdjust * pgm_read_float_near(&(UpDownQs[i][j]));
        //FwdBackAll[j+offset] = amplAdjust * pgm_read_float_near(&(FwdBackQs[i][j]));
        offset = (i+1)%Q;
//        UpDownAll[j+i*N] = amplAdjust * pgm_read_float_near(&(UpDownQs[offset][j]));
//        FwdBackAll[j+i*N] = amplAdjust * pgm_read_float_near(&(FwdBackQs[offset][j]));

        // RR
        //offset = (i+1)%Q*N;
        //UpDownAll[j+offset] = amplAdjust * pgm_read_float_near(&(UpDownQs[i][j]));
        //FwdBackAll[j+offset] = amplAdjust * pgm_read_float_near(&(FwdBackQs[i][j]));
        offset = (i+3)%Q;
//        UpDownAll[j+i*N] = amplAdjust * pgm_read_float_near(&(UpDownQs[offset][j]));
//        FwdBackAll[j+i*N] = amplAdjust * pgm_read_float_near(&(FwdBackQs[offset][j]));
     }
>>>>>>> 4d24b1ff0c1d53eeefe66d86b40296d188b75787

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

  float target[3];
  float targetHome[3] = {232.2870506213818, 0.0, -2.7432364936986176};
  float joints[5] = {0, 0, 0, 0, 0};
  float xAdjustInLegBase = -20;
  float zAdjustInLegBase = -20;

<<<<<<< HEAD
void setTarget(int g, int i, int j)
{
  float UpDown = amplAdjust * pgm_read_float_near(&(UpDownQs[g][i][j]));
  float FwdBack = amplAdjust * pgm_read_float_near(&(FwdBackQs[g][i][j]));

  target[0] = targetHome[0] - UpDown + xAdjustInLegBase;
  target[1] = targetHome[1];
  target[2] = targetHome[2] + FwdBack + zAdjustInLegBase;

  runIK(target, joints);
/*
=======
  target[0] = targetHome[0] - UpDownAll[0] + xAdjustInLegBase;
  target[1] = targetHome[1];
  target[2] = targetHome[2] + FwdBackAll[0] + zAdjustInLegBase;

  runIK(target, joints);

>>>>>>> 4d24b1ff0c1d53eeefe66d86b40296d188b75787
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
<<<<<<< HEAD
*/
}


=======

}


void loop()
{


}


>>>>>>> 4d24b1ff0c1d53eeefe66d86b40296d188b75787
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


