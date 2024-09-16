//#include <time.h>
//#include <Eigen/Eigen>
#include <cstdlib>
#include <iostream>

using namespace std;
int main() {
  const double PI = 3.141592653589793238463;
  double r1 = 0.0;
  double r2 = 0.0;
  double theta = 0.0;
  double phi = 0.0;

  // Eigen::Vector2d pf;
  // pf_.setZero();

  // providing a seed value
  srand((unsigned)time(NULL));

  // generate random numbers between 0 and 1
  r1 = ((double)rand() / (RAND_MAX));
  r2 = ((double)rand() / (RAND_MAX));
  cout << "r1 = " << r1 << endl;
  cout << "r2 = " << r2 << endl;

  theta = r1 * PI;
  phi = r2 * (2 * PI);
  cout << "theta = " << theta << endl;
  cout << "phi = " << phi << endl;

  return 0;
}