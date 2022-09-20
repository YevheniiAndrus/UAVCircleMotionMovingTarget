#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cmath>
#include "uav.h"

void draw_cartesian(cv::Mat img, int width, int height)
{
    cv::line(img, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(0, 0, 0), 2);
    cv::line(img, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(0, 0, 0), 2);
}

cv::Point2d cartesian_to_opencv(const cv::Point2d &point, int width, int height)
{
    double x = point.x + width / 2;
    double y = height / 2 - point.y;
    return cv::Point2d(x, y);
}

void move_target(cv::Point2d &cartesian_target, const double target_velocity, const double theta)
{
    cartesian_target.x = cartesian_target.x + target_velocity * std::cos(theta);
    cartesian_target.y = cartesian_target.y + target_velocity * std::sin(theta);
}

void render_frame(cv::VideoWriter &video, int frame_width, int frame_height, const UAV &uav, const cv::Point2d &wind,
                  const cv::Point2d &target)
{

    cv::Mat frame = cv::Mat(cv::Size(frame_width, frame_height), CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point2d wind_anchor(-450, 250);
    cv::Point2d w(wind_anchor.x + 15 * wind.x, wind_anchor.y + 15 * wind.y);
    cv::arrowedLine(frame, cartesian_to_opencv(wind_anchor, frame_width, frame_height),
                    cartesian_to_opencv(w, frame_width, frame_height), cv::Scalar(255, 0, 255), 2);
    draw_cartesian(frame, frame_width, frame_height);
    cv::circle(frame, cartesian_to_opencv(cv::Point2d(uav.getXPosition(), uav.getYPosition()), frame_width, frame_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
    cv::circle(frame, cartesian_to_opencv(target, frame_width, frame_height), 2, cv::Scalar(0, 0, 255), 2, cv::LineTypes::FILLED);

    video.write(frame);
}

int main()
{
    int img_width = 1024;
    int img_height = 768;

    // target point
    cv::Point2d target_position(-200, 70);

    // initial point
    cv::Point2d uav_position(-450, 300);

    // simulation
    // setup UAV parameters
    UAV uav(uav_position.x, uav_position.y, 4.0, M_PI_2 - M_PI / 10);
    uav.setTargetPosition(target_position.x, target_position.y);
    uav.setTargetVelocity(1.0);
    uav.setRadius(30.0);
    double target_angle = -M_PI / 8;
    double target_velocity = 1.0;

    // set up different wind directions
    std::vector<cv::Point2d> wind{cv::Point2d(2.0, 1.0),
                                  cv::Point2d(4.0, 1.0),
                                  cv::Point2d(1.5, 7.0),
                                  cv::Point2d(-3.0, -1.2),
                                  cv::Point2d(-4.0, 2.3),
                                  cv::Point2d(3.0, -2.5)};
    int wind_counter = 0;
    cv::Point2d wind_vec;

    cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(img_width, img_height));
    for (int i = 0; i < 600; ++i)
    {
        move_target(target_position, target_velocity, target_angle);

        // change wind direction every 10 seconds
        if (i % 100 == 0)
            wind_vec = wind[wind_counter++];

        uav.setTargetPosition(target_position.x, target_position.y);
        uav.applyWind(wind_vec.x, wind_vec.y);
        uav.move(1.0);

        render_frame(video, img_width, img_height, uav, wind_vec, target_position);
    }

    return 0;
}