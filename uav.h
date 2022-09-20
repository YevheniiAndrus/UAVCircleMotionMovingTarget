#pragma once

class UAV
{
public:
    UAV() = default;
    UAV(double pos_x, double pos_y, double velocity, double angle);

    void setTargetPosition(const double pos_x, const double pos_y);
    void setTargetVelocity(const double target_velocity);
    void setRadius(const double radius);
    void applyWind(const double wind_dir_x, const double wind_dir_y);
    void move(const double K);

    double getXPosition() const;
    double getYPosition() const;

private:
    // relative side bearing angle is an angle between centripetal acceleration
    // which is perpendicular to aircraft velocity and vector from heading from
    // target to aircraft
    // Obtain it by subtracting sum of target to uav angle and uav angle from 180
    double bearing_angle(const double phi, const double theta);

    // calculates coeff which is essentially cosine of angle
    // betwee vectors Va/t and Vair, where
    // Va/t - vector pointing from target to UAV
    // Vair - UAV vector subtract wind vector
    // returned coefficient should be applied inversaly proportional to acceleration
    double wind_effect();

private:
    double m_position_x;
    double m_position_y;
    double m_velocity;
    double m_angle;
    double m_radius;

    double m_target_position_x;
    double m_target_position_y;
    double m_target_velocity;

    double m_wind_dir_x;
    double m_wind_dir_y;

    bool m_wind_applied = false;
};