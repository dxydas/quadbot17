#include <ax12.h>






template <size_t n, size_t m, size_t p>
void MatrixMultiply(const float (&A)[n][m], const float (&B)[m][p], float (&C)[n][p])
{
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < p; ++j)
    {
      C[i][j] = 0;
      for (int k = 0; k < m; ++k)
        C[i][j] += A[i][k] * B[k][j];
    }
}





// Target arrays


//size 100?

// build arrays in excel, put arrays into header?

float FLUpDown[100];
float FLFwdBack[100];
float FRUpDown[100];
float FRFwdBack[100];
float RLUpDown[100];
float RLFwdBack[100];
float RRUpDown[100];
float RRFwdBack[100];


// max up down range

// max fwd back range


void setup()
{
  Serial.begin(38400);
  
  // Set up arrays

  // Front left (FL) leg
  
  // Front right (FR) leg

  // Rear left (RL) leg
  
  // Rear right (RR) leg
    
  
  // Forward/backward - X+ is forward
  
  // Up/down - Z+ is up
  
  
  float target[3] = {1, 2, 3};
  float joints[5] = {0, 0, 0, 0, 0};
  
  runIK(target, joints);
  
  
  
  
  



  
  
  // Matrix multiply test
  // A = [ 1 2 3
  //       4 5 6 ]
  // B = [ 1  2  3  4
  //       5  6  7  8
  //       9 10 11 12 ]
  // C = [ 38 44  50  56
  //       83 98 113 128 ]
  int n = 2;
  int m = 3;
  int p = 4;
  float A[2][3];
  float B[3][4];
  float C[2][4];
  int x;

  C[0][0] = 0;

  x = 0;
  Serial.println("A:");
  for (int i = 0; i < n; ++i)
  {
    for (int k = 0; k < m; ++k)
    {
      A[i][k] = ++x;
      Serial.print(A[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  
  x = 0;
  Serial.println("B:");
  for (int k = 0; k < m; ++k)
  {
    for (int j = 0; j < p; ++j)
    {
      B[k][j] = ++x;
      Serial.print(B[k][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }

  MatrixMultiply(A, B, C);
  
  Serial.println("C:");
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < p; ++j)
    {
        Serial.print(C[i][j]);
        Serial.print(" ");
    }
    Serial.println("");
  }



}


void loop()
{
  
  // iterate through arrays
  // all legs at each loop
  
  
}



// IK - get joint angles from foot target
void runIK(const float target[], float joints[])
{
  


}











