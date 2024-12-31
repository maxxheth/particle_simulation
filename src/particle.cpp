#include "particle_simulation/particle.hpp"


// Constructor
Particle::Particle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id)
    : pos(d_x, d_y), vel(d_vx, d_vy), acc(d_ax, d_ay), id_(id) {}

// Update the particle's position based on its velocity and acceleration
void Particle::Update(double d_time_step) {
    // Update velocity based on acceleration
    vel.x() += acc.x() * d_time_step;
    vel.y() += acc.y() * d_time_step;

    // Update position based on velocity
    pos.x() += vel.x() * d_time_step;
    pos.y() += vel.y() * d_time_step;
}
