#ifndef PARTICLE_SIMULATION_PARTICLE_MANAGER_HPP
#define PARTICLE_SIMULATION_PARTICLE_MANAGER_HPP

#include "particle_simulation/particle.hpp"
#include <vector>
#include <utility>

#define DAMPING_FACTOR 0.9
class ParticleManager {
public:
    void SetBounds(double d_min_x, double d_max_x, double d_min_y, double d_max_y);
    void AddParticle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id);
    void UpdateParticles(double d_time_step);

    // Get all particle positions
    std::vector<std::pair<double, double>> GetParticlePositions() const;

    int GetParticleCount() const;

private:
    std::pair<int, int> GetGridIndex(double x, double y) const;
    void AssignParticlesToGrid();

    std::vector<Particle> particles_; // List of particles
    double grid_cell_size_ = 5.0; // Example cell size
    int grid_width_, grid_height_;
    std::vector<std::vector<std::vector<int>>> grid_; // 3D vector: [grid_y][grid_x][particle_indices]

    double d_min_x_, d_max_x_; // X bounds
    double d_min_y_, d_max_y_; // Y bounds
};
#endif // PARTICLE_SIMULATION_PARTICLE_MANAGER_HPP

