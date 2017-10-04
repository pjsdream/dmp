#include <dmp/math/matrix.h>

int main()
{
  printf("Testing matrix\n");

  dmp::Matrix4d m1;
  dmp::Matrix4d m2 = dmp::Matrix4d::Zero();
  dmp::Matrix4d m3 = dmp::Matrix4d::Identity();

  printf("Matrix multiplication\n");
  dmp::Matrix4d m4;
  m4 = m2 * m3;

  printf("Matrix-scalar multiplication\n");
  dmp::Matrix4d m5;
  m5 = m3 * 5.;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
      printf("%lf ", m5(i, j));
    printf("\n");
  }

  return 0;
}
