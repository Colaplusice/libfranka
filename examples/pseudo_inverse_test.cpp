
#include "examples_common.h"
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char const *argv[])
{
    Eigen::Matrix <double,2,3>m;
    m<<2,3,4,
    5,6,7;
    std::cout<<m<<std::endl;
    std::cout<<pseudoinverse(m)<<std::endl;  
    Eigen::Matrix<double,1,3>m1;
    m1<<m;
    
    Eigen::Matrix<double,1,3>m2;
    m2<<3,3,3;

    std::cout<<m1+m2<<std::endl;
    // std::cout<<m.row(0)<<std::endl;
    // std::cout<<m.row(1)<<std::endl;
    // std::cout<<m.row(2)<<std::endl;
    // std::array<double,3>data={1.0,2.3,3.4};

    
    //Eigen::Matrix<double,1,3>::Map(data.data(),m1.rows(),m1.cols())=m1;

        // for (int i = 0; i < data.size(); i++)
        // {
        //     std::cout<<data[i]<<std::endl;
        // }
    return 0;
}
