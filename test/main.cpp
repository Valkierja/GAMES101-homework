#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix2f normalize(Eigen::Matrix2f input) {


    float mag = sqrt(input(0) * input(0) + input(1) * input(1) + input(2) * input(2));
    auto result = input;
    result(0) /= mag;
    result(1) /= mag;
    result(2) /= mag;

    std::cout << mag << std::endl;
    std::cout << result << std::endl;
    std::cout << result(0) * result(0) + result(1) * result(1) + result(2) * result(2) << std::endl;
    return result;

}

int main(int argc, const char **argv) {
    Eigen::Matrix2f test1;
    test1<<0.81724,3.2141,1.352,1;


    normalize(test1);
    return 0;
}
