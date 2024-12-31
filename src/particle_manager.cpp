#include "particle_simulation/particle_manager.hpp"

// Set the bounds within which particles can move
void ParticleManager::SetBounds(double d_min_x, double d_max_x, double d_min_y, double d_max_y) {
    d_min_x_ = d_min_x;
    d_max_x_ = d_max_x;
    d_min_y_ = d_min_y;
    d_max_y_ = d_max_y;

    grid_height_ = static_cast<int>((d_max_y_ - d_min_y_) / grid_cell_size_);
    grid_width_ = static_cast<int>((d_max_x_ - d_min_x_) / grid_cell_size_);

    grid_.resize(grid_height_, std::vector<std::vector<int>>(grid_width_));
}

// Add a new particle to the manager
void ParticleManager::AddParticle(double d_x, double d_y, double d_vx, double d_vy, double d_ax, double d_ay, int id) {
    std::cout << "Add Particle: (" << d_x << ", " << d_y << ")" << std::endl;
    particles_.emplace_back(d_x, d_y, d_vx, d_vy, d_ax, d_ay, id);
}

std::pair<int, int> ParticleManager::GetGridIndex(double x, double y) const {
    int grid_x = static_cast<int>((x - d_min_x_) / grid_cell_size_);
    int grid_y = static_cast<int>((y - d_min_y_) / grid_cell_size_);
    return {grid_x, grid_y};
}

void ParticleManager::AssignParticlesToGrid() {
    // Clear the grid
    for (auto& row : grid_) {
        for (auto& cell : row) {
            cell.clear();
        }
    }

    // Assign particles to grid
    for (int i = 0; i < particles_.size(); ++i) {
        const auto& p = particles_[i];
        auto [grid_x, grid_y] = GetGridIndex(p.pos.x(), p.pos.y());
        if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_) {
            grid_[grid_y][grid_x].push_back(i);
        }
    }
}

// Update all particles
void ParticleManager::UpdateParticles(double d_time_step) {
    AssignParticlesToGrid();
    for (auto& p1 : particles_) {
        p1.Update(d_time_step);

        auto [grid_x, grid_y] = GetGridIndex(p1.pos.x(), p1.pos.y());

        // Check this cell and adjacent cells
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int neighbor_x = grid_x + dx;
                int neighbor_y = grid_y + dy;

                if (neighbor_x >= 0 && neighbor_x < grid_width_ && neighbor_y >= 0 && neighbor_y < grid_height_) {
                    for (int j : grid_[neighbor_y][neighbor_x]) {
                        auto& p2 = particles_[j];

                        // Skip self
                        if (p1.id() == p2.id()) continue;

                        double dx = p2.pos.x() - p1.pos.x();
                        double dy = p2.pos.y() - p1.pos.y();
                        double distance = std::sqrt(dx * dx + dy * dy);
                        double min_distance = PARTICLE_RADIUS * 2.0;

                        if (distance < min_distance) {
                            // Handle collision response
                            double nx = dx / distance;
                            double ny = dy / distance;
                            double relative_velocity = (p2.vel.x() - p1.vel.x()) * nx + 
                                                        (p2.vel.y() - p1.vel.y()) * ny;

                            if (relative_velocity < 0) {
                                double impulse = 2 * relative_velocity / (1 + 1) ;  // Assuming equal mass

                                p1.vel = Eigen::Vector2f(p1.vel.x() + impulse * nx,
                                                           p1.vel.y() + impulse * ny);
                                p2.vel = Eigen::Vector2f(p2.vel.x() - impulse * nx,
                                                           p2.vel.y() - impulse * ny);
                            }

                            // Adjust positions to prevent overlap
                            double overlap = min_distance - distance;
                            double correction_factor = 0.5; // Adjust this factor as needed

                            p1.pos = Eigen::Vector2f(p1.pos.x() - overlap * nx * correction_factor,
                                                       p1.pos.y() - overlap * ny * correction_factor);
                            p2.pos = Eigen::Vector2f(p2.pos.x() + overlap * nx * correction_factor,
                                                       p2.pos.y() + overlap * ny * correction_factor);
                        }
                    }
                }
            }
        }
        // Check for boundary collision and reverse velocity if needed
        if (p1.pos.x() < d_min_x_ + PARTICLE_RADIUS) {
            p1.pos.x() = d_min_x_ + PARTICLE_RADIUS;
            p1.vel.x() = -p1.vel.x() * DAMPING_FACTOR;
        }
        if (p1.pos.x() > d_max_x_ - PARTICLE_RADIUS) {
            p1.pos.x() = d_max_x_ - PARTICLE_RADIUS;
            p1.vel.x() = -p1.vel.x() * DAMPING_FACTOR;
        }
        if (p1.pos.y() < d_min_y_ + PARTICLE_RADIUS) {
            p1.pos.y() = d_min_y_ + PARTICLE_RADIUS;
            p1.vel.y() = -p1.vel.y() * DAMPING_FACTOR;
        }
        if (p1.pos.y() > d_max_y_ - PARTICLE_RADIUS) {
            p1.pos.y() = d_max_y_ - PARTICLE_RADIUS;
            p1.vel.y() = -p1.vel.y() * DAMPING_FACTOR;
        }
    }
}

// Get all particle positions
std::vector<std::pair<double, double>> ParticleManager::GetParticlePositions() const {
    std::vector<std::pair<double, double>> positions;
    for (const auto& particle : particles_) {
        positions.emplace_back(particle.pos.x(), particle.pos.y());
    }
    return positions;
}

std::vector<Particle> ParticleManager::GetParticles() const {
    return particles_;
}

int ParticleManager::GetParticleCount() const {
    return particles_.size();
}