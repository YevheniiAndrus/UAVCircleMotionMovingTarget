#include "uav.h"
#include <cmath>
#include <utility>
#include <iostream>

namespace
{
    struct Vector
    {
        double x;
        double y;

        Vector operator-(const Vector &vec)
        {
            return Vector{x - vec.x, y - vec.y};
        }

        Vector operator+(const Vector &vec)
        {
            return Vector{x + vec.x, y + vec.y};
        }
    };

    int to_gradus(double radians)
    {
        return radians * 180 / M_PI;
    }

    double to_radians(int gradus)
    {
        return static_cast<double>(gradus) * M_PI / 180;
    }

    Vector build_vector(const double velocity, const double theta)
    {
        return Vector{velocity * std::sin(theta), velocity * std::cos(theta)};
    }
}

UAV::UAV(double pos_x, double pos_y, double velocity, double angle) : m_position_x(pos_x),
                                                                      m_position_y(pos_y),
                                                                      m_velocity(velocity),
                                                                      m_angle(angle) {}

void UAV::setTargetPosition(const double position_x, const double position_y)
{
    m_target_position_x = position_x;
    m_target_position_y = position_y;
}

void UAV::setTargetVelocity(const double velocity)
{
    m_target_velocity = velocity;
}

void UAV::setRadius(const double radius)
{ß
    m_radius = radius;
}

void UAV::applyWind(const double wind_dir_x, const double wind_dir_y)
{
    m_wind_dir_x = wind_dir_x;
    m_wind_dir_y = wind_dir_y;

    m_wind_applied = true;
}

void UAV::move(const double K)
{
    // target to UAV angle
    double target_to_uav_angle = std::atan2(m_position_x - m_target_position_x,
                                            m_position_y - m_target_position_y);

    // bearing angle
    double n_angle = bearing_angle(target_to_uav_angle, m_angle);

    // guidance law
    // acceleration without wind effect
    double acceleration = m_velocity * m_velocity / m_radius + K * std::sin(n_angle);

    // apply wind
    if (m_wind_applied)
        acceleration = acceleration / wind_effect();

    // delta angle
    double delta_angle = acceleration / m_velocity;
    m_angle = m_angle - delta_angle;

    // update UAV position
    m_position_x = m_position_x + m_velocity * std::cos(m_angle);
    m_position_y = m_position_y + m_velocity * std::sin(m_angle);
}

double UAV::wind_effect()
{
    // UAV vector
    Vector uav_vector = build_vector(m_velocity, m_angle);

    // wind vector
    Vector wind_vector{m_wind_dir_x, m_wind_dir_y};

    // target vector
    Vector target_vector = build_vector(m_target_velocity, std::atan2(m_target_position_y, m_target_position_x));

    // target to UAV vector
    Vector ut_vector = uav_vector - target_vector;

    // air vector
    Vector air_vector = uav_vector - wind_vector;

    double cos_ksi = (air_vector.x * ut_vector.x + air_vector.y * ut_vector.y) /
                     (std::sqrt(air_vector.x * air_vector.x + air_vector.y * air_vector.y) *
                      std::sqrt(ut_vector.x * ut_vector.x + ut_vector.y * ut_vector.y));

    return cos_ksi;
}

double UAV::bearing_angle(const double phi, const double theta)
{
    int phi_g = to_gradus(phi) % 360;
    int theta_g = to_gradus(theta) % 360;

    int n = 180 - (phi_g + theta_g);
    return to_radians(n);
}

double UAV::getXPosition() const
{
    return m_position_x;
}

double UAV::getYPosition() const
{
    return m_position_y;
}