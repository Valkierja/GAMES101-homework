//
// Created by cs18 on 4/9/23.
//

#ifndef TRANSFORMATION_MYUTILITY_H
#define TRANSFORMATION_MYUTILITY_H

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>


#define fPoint2D Eigen::Matrix<float, 3, 1>
#define fDeg2Rad(theta) (theta / 180.0f * acos(-1))

Eigen::Matrix3f createRotation(float theta) {
    Eigen::Matrix3f temp;
    temp << cos(fDeg2Rad(theta)), -sin(fDeg2Rad(theta)), 0.0,
            sin(fDeg2Rad(theta)), cos(fDeg2Rad(theta)), 0.0,
            0.0, 0.0, 1.0;
    return temp;
}

Eigen::Matrix3f createScale(float x, float y) {
    Eigen::Matrix3f temp;
    temp << x, 0, 0,
            0, y, 0,
            0, 0, 1;
    return temp;
}

Eigen::Matrix3f createTranslation(float x, float y) {
    Eigen::Matrix3f temp;
    temp << 1, 0, x,
            0, 1, y,
            0, 0, 1;
    return temp;
}


#define fScale2D(x, y) createScale(x,y)
#define fRotate2D(theta) createRotation(theta)

#define fTranslation2D(x, y) createTranslation(x,y)
#endif //TRANSFORMATION_MYUTILITY_H
