#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cmath>

int to_gradus(double radians){
    return radians * 180 / M_PI;
}

double to_radians(int gradus){
    return static_cast<double>(gradus) * M_PI / 180;
}

void update_guidance(const double velocity, const double acceleration, const double phi,
            double& delta_x, double& delta_y, double& delta_phi){

    delta_x = velocity * std::cos(phi);
    delta_y = velocity * std::sin(phi);

    delta_phi = acceleration / velocity;
}

void draw_cartesian(cv::Mat img, int width, int height){
    cv::line(img, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(0, 0, 0), 2);
    cv::line(img, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(0, 0, 0), 2);
}

cv::Point2d cartesian_to_opencv(const cv::Point2d& point, int width, int height){
    double x = point.x + width / 2;
    double y = height / 2 - point.y;
    return cv::Point2d(x, y);
}

void move_target(cv::Point2d& cartesian_target, double theta){
    cartesian_target.x = cartesian_target.x + std::cos(theta);
    cartesian_target.y = cartesian_target.y + std::sin(theta);
}

double bearing_angle(const double phi, const double theta){
    int phi_g = to_gradus(phi) % 360;
    int theta_g = to_gradus(theta) % 360;

    int n = 180 - (phi_g + theta_g);
    return to_radians(n);
}

cv::Point2d build_vector(const double velocity, const double theta){
    return cv::Point2d(velocity * std::sin(theta), velocity * std::cos(theta));
}

double vector_dot(const cv::Point2d& v1, const cv::Point& v2){
    return v1.x * v2.x + v1.y * v2.y;
}

double vector_module(const cv::Point2d& v){
    return std::sqrt(v.x * v.x + v.y * v.y);
}

int main(){
   int img_width = 1024;
    int img_height = 768;
    cv::Mat blank = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(255, 255, 255));
    draw_cartesian(blank, img_width, img_height);

    // target point
    cv::Point2d cartesian_target(-200, 70);
    cv::circle(blank, cartesian_to_opencv(cartesian_target, img_width, img_height), 2, cv::Scalar(0, 0, 255), 2, cv::LineTypes::FILLED);

    // initial point
    cv::Point2d cartesian_uav(-450, 300);
    cv::circle(blank, cartesian_to_opencv(cartesian_uav, img_width, img_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);

    // simulation
    double uav_angle = M_PI/2 - M_PI / 10; // initial azimut
    double target_angle = -M_PI / 8;
    cv::Point vec(cartesian_uav.x - cartesian_target.x, cartesian_uav.y - cartesian_target.y);
    double uav_velocity = 4.0;
    double target_velocity = 1.0;
    double r_ref = 30;
    double K = 1.0;

    std::vector<cv::Point2d> wind{cv::Point2d(2.0, 1.0), 
                                  cv::Point2d(4.0, 1.0),
                                  cv::Point2d(1.5, 7.0),
                                  cv::Point2d(-3.0, -1.2),
                                  cv::Point2d(-4.0, 2.3),
                                  cv::Point2d(3.0, -2.5)};
    int wind_counter = 0;
    cv::Point2d wind_vec;
    cv::Point2d wind_anchor(-450, 250);

    cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(img_width, img_height));
    for(int i = 0; i < 600; ++i){
        move_target(cartesian_target, target_angle);

        double target_to_uav_angle = std::atan2(cartesian_uav.x - cartesian_target.x, cartesian_uav.y - cartesian_target.y);
        double n_angle = bearing_angle(uav_angle, target_to_uav_angle);

        cv::Point2d uav_vec = build_vector(uav_velocity, uav_angle);
        cv::Point2d target_vec = build_vector(target_velocity, target_angle);
        cv::Point2d at_vec = cv::Point2d(uav_vec.x - target_vec.x, uav_vec.y - target_vec.y);

        if(i % 100 == 0)
            wind_vec = wind[wind_counter++];
        
        cv::Point2d air_vec(uav_vec.x - wind_vec.x, uav_vec.y - wind_vec.y);

        double cos_ksi = vector_dot(air_vec, at_vec) / (vector_module(air_vec) * vector_module(at_vec));

        double acceleration = uav_velocity * uav_velocity / r_ref + K * std::sin(n_angle);
        acceleration = acceleration / cos_ksi;

        double delta_x, delta_y, delta_phi;
        update_guidance(uav_velocity, acceleration, uav_angle, delta_x, delta_y, delta_phi);

        cartesian_uav.x = cartesian_uav.x + delta_x;
        cartesian_uav.y = cartesian_uav.y + delta_y;
        uav_angle = uav_angle - delta_phi;

        cv::circle(blank, cartesian_to_opencv(cartesian_uav, img_width, img_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);

        // make frame and put to video sequence
        cv::Mat frame = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Point2d w(wind_anchor.x + 15 * wind_vec.x, wind_anchor.y + 15 * wind_vec.y);
        cv::arrowedLine(frame, cartesian_to_opencv(wind_anchor, img_width, img_height),
                                  cartesian_to_opencv(w, img_width, img_height), cv::Scalar(255, 0, 255), 2);
        draw_cartesian(frame, img_width, img_height);
        cv::circle(frame, cartesian_to_opencv(cartesian_uav, img_width, img_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
        cv::circle(frame, cartesian_to_opencv(cartesian_target, img_width, img_height), 2, cv::Scalar(0, 0, 255), 2, cv::LineTypes::FILLED);

        video.write(frame);
    }

    return 0;
}