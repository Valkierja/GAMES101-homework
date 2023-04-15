#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;



int main(int argc, const char** argv)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    std::cout<<model;

    return 0;
}
