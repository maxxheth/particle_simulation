#ifndef PARTICLE_SIMULATION_PARTICLE_HPP
#define PARTICLE_SIMULATION_PARTICLE_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define GRAVITY 30
#define PARTICLE_RADIUS 1.0

class Particle {
public:
    // Constructor
    Particle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id);

    // Update the particle's position based on its velocity and acceleration
    void Update(double d_time_step);

    int id() const { return id_; }

    Eigen::Vector2f pos;
    Eigen::Vector2f vel;
    Eigen::Vector2f acc;

private:

    int id_;
};
#endif // PARTICLE_SIMULATION_PARTICLE_HPP

