// clang-format off
#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline double DEG2RAD(double deg) { return deg * MY_PI / 180; }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 1.模型变换矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    double rad = DEG2RAD(rotation_angle);
    // Create the model matrix for rotating the triangle around the Z axis. Then return it.
    model << cos(rad), -sin(rad), 0, 0,
             sin(rad), cos(rad), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

    return model;
}

// 2.透视投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // 这里传入的 zNear 和 zFar 为正值，转换一下
    zNear = -zNear;
    zFar = -zFar;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters. Then return it.
    Eigen::Matrix4f persp2orth;
    persp2orth << zNear, 0, 0, 0,
                  0, zNear, 0, 0,
                  0, 0, zNear + zFar, -zNear * zFar,
                  0, 0, 1, 0;

    Eigen::Matrix4f ortho1;
    ortho1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, -(zNear + zFar) / 2,
              0, 0, 0, 1;

    float t = tan(DEG2RAD(eye_fov / 2.0f)) * abs(zNear), b = -t;
    float r = t * aspect_ratio, l = -r;

    Eigen::Matrix4f ortho2;
    ortho2 << 2 / (r - l), 0, 0, 0,
              0, 2 / (t - b), 0, 0,
              0, 0, 2 / (zNear - zFar), 0,
              0, 0, 0, 1;

    projection = ortho2 * ortho1 * persp2orth;
    return projection;
}

// 3.附加：绕任意过原点的轴的旋转变换矩阵
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float rad = DEG2RAD(angle);

    // Rodrigues’ Rotation Formula
    Eigen::Matrix3f m;
    m << 0, -axis.z(), axis.y(),
         axis.z(), 0, -axis.x(),
         -axis.y(), axis.x(), 0;
    Eigen::Matrix3f rotation = cos(rad) * Eigen::Matrix3f::Identity() + (1 - cos(rad)) * axis * axis.transpose() + sin(rad) * m;

    // Matrix3f to Matrix4f
    model.block<3, 3>(0, 0) = rotation;
    return model;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));

        // 绕任意 axis 轴旋转
        Vector3f axis{0, 1, 0};
        r.set_model(get_rotation(axis, angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
