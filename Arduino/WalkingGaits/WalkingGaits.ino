#include <math.h>
#include <ax12.h>




// max up down range

// max fwd back range


void setup()
{
  Serial.begin(38400);


// Target arrays - Split into quadrants

  
  float UpDownQs[4][25] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0.2,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.8,0.6,0.4,0.2,0}
  };

  float FwdBackQs[4][25] = {
    {1,1,1,1,1,1,0.9855072464,0.9710144928,0.9565217391,0.9420289855,0.9275362319,0.9130434783,0.8985507246,0.884057971,0.8695652174,0.8550724638,0.8405797101,0.8260869565,0.8115942029,0.7971014493,0.7826086957,0.768115942,0.7536231884,0.7391304348,0.7246376812},
    {0.7101449275,0.6956521739,0.6811594203,0.6666666667,0.652173913,0.6376811594,0.6231884058,0.6086956522,0.5942028986,0.5797101449,0.5652173913,0.5507246377,0.5362318841,0.5217391304,0.5072463768,0.4927536232,0.4782608696,0.4637681159,0.4492753623,0.4347826087,0.4202898551,0.4057971014,0.3913043478,0.3768115942,0.3623188406},
    {0.347826087,0.3333333333,0.3188405797,0.3043478261,0.2898550725,0.2753623188,0.2608695652,0.2463768116,0.231884058,0.2173913043,0.2028985507,0.1884057971,0.1739130435,0.1594202899,0.1449275362,0.1304347826,0.115942029,0.1014492754,0.0869565217,0.0724637681,0.0579710145,0.0434782609,0.0289855072,0.0144927536,0},
    {0,0,0,0,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1}
  };

  float UpDownAll[100];
  float FwdBackAll[100];

  int Q = 4;
  int N = 25;
  int offset;
  for (int i = 0; i < Q; ++i)
    for (int j = 0; j < N; ++j)
      {
        // FL
        //offset = i*N;
        //UpDownAll[j+offset] = UpDownQs[i][j];
        //FwdBackAll[j+offset] = FwdBackQs[i][j];
        
        // FR
        //offset = (i+2)%Q*N;
        //UpDownAll[j+offset] = UpDownQs[i][j];
        //FwdBackAll[j+offset] = FwdBackQs[i][j];


        // RL
        //offset = (i+3)%Q*N;
        //UpDownAll[j+offset] = UpDownQs[i][j];
        //FwdBackAll[j+offset] = FwdBackQs[i][j];

        // RR
        //offset = (i+1)%Q*N;
        UpDownAll[j+offset] = UpDownQs[i][j];
        FwdBackAll[j+offset] = FwdBackQs[i][j];
      }

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







  // Forward/backward - X+ is forward
  
  // Up/down - Z+ is up
  
  float target[3] = {232.2870506213818, 0.0, -2.7432364936986176};
  float joints[5] = {0, 0, 0, 0, 0};
  
  runIK(target, joints);
  Serial.println("joints:");
  for (int i = 0; i < 5; ++i)
  {
    Serial.print(joints[i]);
    Serial.print(" ");
  }
  Serial.println("");
  
  
  

}


void loop()
{
  
  // iterate through arrays
  // all legs at each loop
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
    Serial.println("num");
    Serial.println(num);
    Serial.println("den");
    Serial.println(den);

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





