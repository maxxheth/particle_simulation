#ifndef PARTICLE_SIMULATION_PARTICLE_HPP
#define PARTICLE_SIMULATION_PARTICLE_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define GRAVITY 9.8

class Particle {
public:
    // Constructor
    Particle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id);

    // Update the particle's position based on its velocity and acceleration
    void Update(double d_time_step);

    // Get the current position of the particle
    void GetPosition(double& d_x, double& d_y) const;

    double x() const { return d_x_; }
    double y() const { return d_y_; }
    int id() const { return id_; }
    void SetPosition(double d_x, double d_y);

    // Set the velocity of the particle
    void SetVelocity(double d_velocity_x, double d_velocity_y);

    // Get the velocity of the particle
    double GetVelocityX() const;
    double GetVelocityY() const;

    // Get the radius of the particle
    double GetRadius() const;

private:
    double d_x_;  // X position
    double d_y_;  // Y position
    double d_vx_; // X velocity
    double d_vy_; // Y velocity
    double d_ax_; // X acceleration
    double d_ay_; // Y acceleration
    double d_radius_ = 2.0; // Example radius
    int id_;
};
#endif // PARTICLE_SIMULATION_PARTICLE_HPP
