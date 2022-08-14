#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;

int main() {
    const float PI = 3.1415926;
    float x, y;
    cout << "输入点的坐标(x y): ";
    cin >> x >> y;
    Eigen::Vector3f p(x, y, 1);
    int degree;
    float nx, ny;
    cout << "输入旋转角度和位移(degree x y): ";
    cin >> degree >> nx >> ny;
    float rad = degree * PI / 180;
    Eigen::Matrix3f m{
        {cos(rad), -sin(rad), nx},
        {sin(rad), cos(rad), ny},
        {0, 0, 1}};
    Eigen::Vector3f res = m * p;
    cout << res << endl;
    return 0;
}