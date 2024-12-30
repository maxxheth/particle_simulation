#include "particle_simulation/particle.hpp"


// Constructor
Particle::Particle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id)
    : d_x_(d_x), d_y_(d_y), d_vx_(d_vx), d_vy_(d_vy), d_ax_(d_ax), d_ay_(d_ay), id_(id) {}

// Update the particle's position based on its velocity and acceleration
void Particle::Update(double d_time_step) {
    // Update velocity based on acceleration
    d_vx_ += d_ax_ * d_time_step;
    d_vy_ += d_ay_ * d_time_step;

    // Update position based on velocity
    d_x_ += d_vx_ * d_time_step;
    d_y_ += d_vy_ * d_time_step;
}

// Get the current position of the particle
void Particle::GetPosition(double& d_x, double& d_y) const {
    d_x = d_x_;
    d_y = d_y_;
}

void Particle::SetPosition(double d_x, double d_y) {
    d_x_ = d_x;
    d_y_ = d_y;
}

// Set the velocity of the particle
void Particle::SetVelocity(double d_velocity_x, double d_velocity_y) {
    d_vx_ = d_velocity_x;
    d_vy_ = d_velocity_y;
}