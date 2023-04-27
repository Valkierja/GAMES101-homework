#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace Eigen;

constexpr double MY_PI = 3.1415926;
Eigen::Matrix<float, 3, 1> zAxis(0, 0, 1);
#define fDeg2Rad(theta) (theta / 180.0f *MY_PI)

Eigen::Matrix<float, 3, 1> normalize(Eigen::Vector3f input) {
//    if (input() != 0) return Eigen::Vector3f::Zero();
    float mag = sqrt(input(0) * input(0) + input(1) * input(1) + input(2) * input(2));

    input(0) /= mag;
    input(1) /= mag;
    input(2) /= mag;

    std::cout << mag << std::endl;
    std::cout << input << std::endl;
    std::cout << input(0) * input(0) + input(1) * input(1) + input(2) * input(2) << std::endl;

    return input;

}

Eigen::Matrix4f get_rotation(Eigen::Matrix<float, 3, 1> axis, float angle) {
//    Eigen::Matrix<float, 3, 1> normalized_axis = normalize(axis);
//    Eigen::Matrix<float, 1, 3> normalized_axisT = normalized_axis.transpose();
    Eigen::Matrix3f identity = Matrix3f::Identity();
    float rad = fDeg2Rad(angle);
//    Eigen::Matrix3f N;
//    N << 0, -normalized_axis(2), normalized_axis(1),
//            normalized_axis(2), 0, -normalized_axis(0),
//            -normalized_axis(1), normalized_axis(0), 0;
//    Eigen::Matrix3f temp = (cos(rad) * identity) + ((1 - cos(rad)) * normalized_axis * normalized_axisT) + sin(rad) * N;
    Eigen::Matrix4f result;
//    result.conservativeResize(4, 4);
    return result;
}



int main(int argc, const char **argv) {

    std::cout << normalize(zAxis) << std::endl;
    return 0;
}
