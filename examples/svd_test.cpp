#include<iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <Eigen/QR>    

using namespace std;

int main(int argc, char const *argv[])
{
    /* code */

   Eigen:: MatrixXf m =  Eigen::MatrixXf::Random(3,7);
std::cout << "Here is the matrix m:" << std::endl << m << std::endl;
 Eigen::JacobiSVD< Eigen:: MatrixXf> svd(m,Eigen::ComputeThinU |  Eigen::ComputeThinV);
cout << "Its singular values are:" << endl << svd.singularValues() << endl;
cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
 Eigen::Vector3f rhs(1, 0, 0);
cout << "Now consider this rhs vector:" << endl << rhs << endl;
cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;
    return 0;
}
