#include<iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <Eigen/QR>    

using namespace std;
Eigen::MatrixXf pseudoinverse(Eigen::MatrixXf m)
{
    //Eigen::Matrix<float,2,3> m;
  //  m<<0.68,0.597,-0.211, 0.823,0.566,-0.605;
   // m<<1,2,3,4,5,6;
    cout<<m<<endl;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd =m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
   // Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV); 
    const Eigen::MatrixXf singularValues = svd.singularValues();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());
	singularValuesInv.setZero();
	double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
	 	if (singularValues(i) > pinvtoler)
	 		singularValuesInv(i, i) = 1.0f / singularValues(i);
	 	else
	 		singularValuesInv(i, i) = 0.f;
	 }
    Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
    std::cout << pinvmat << std::endl;
	return pinvmat;
}
int main(int argc, char **argv){
      Eigen::Matrix<double, 6, 1> error;
      for(int i=0; i<6;i++)
      for (int j = 0; j < 1; j++)
      {
          error(i,j)=1;
          std::cout<<error(i,j);
      }
    cout<<endl;
    std::vector<int>ve;
    ve.push_back(1);
    ve.push_back(2);
    cout<<ve.data()<<endl;
    Eigen::Vector4f ve4(1,2,3,4);
    std::cout<<error.cols()<<error.rows()<<endl;
    cout<<ve4<<endl;

    Eigen::MatrixXf m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);


    array<double,4>arr={2,6,7.11,8.23};

    Eigen::Map<const Eigen::Matrix<double, 2, 2>>m2(arr.data());
    cout<<m2<<endl;
    // m2<<1,2,3,4;
    // cout<<m2<<endl;
    //pseudoinverse();
    // Eigen::Matrix<float, 6, 7> name=Eigen::Matrix<float,6,7>::Random();
    // Eigen::Matrix<float,3,7> res;
    // for (int i = 0; i < res.rows(); i++)
    // {
    //     for (int j = 0; j < res.cols(); j++)
    //     {
    //             res(i,j)=name(i,j);
    //     }
    // }
    // cout<<res;

//    Eigen::Matrix<float,2,3> m2;
//    m2<<0.68,0.597,-0.211, 0.823,0.566,-0.605;
//    m2<<1,2,3,4,5,6;
//    cout<<m2;

  //cout<<(pseudoinverse(res));

}

