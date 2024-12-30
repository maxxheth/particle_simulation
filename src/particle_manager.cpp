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
        auto [grid_x, grid_y] = GetGridIndex(p.x(), p.y());
        std::cout << "Particle " << p.id() << " is in grid cell (" << grid_x << ", " << grid_y << ")" << std::endl;
        if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_) {
            grid_[grid_y][grid_x].push_back(i);
        }
    }
}

// Update all particles
void ParticleManager::UpdateParticles(double d_time_step) {
    AssignParticlesToGrid();
    for (auto& particle : particles_) {
        particle.Update(d_time_step);


        auto [grid_x, grid_y] = GetGridIndex(particle.x(), particle.y());

        // Check this cell and adjacent cells
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int neighbor_x = grid_x + dx;
                int neighbor_y = grid_y + dy;

                if (neighbor_x >= 0 && neighbor_x < grid_width_ && neighbor_y >= 0 && neighbor_y < grid_height_) {
                    for (int j : grid_[neighbor_y][neighbor_x]) {
                        auto& p2 = particles_[j];

                        // Skip self
                        if (particle.id() == p2.id()) continue;

                        double dx = p2.x() - particle.x();
                        double dy = p2.y() - particle.y();
                        double distance = std::sqrt(dx * dx + dy * dy);
                        double min_distance = particle.GetRadius() + p2.GetRadius();

                        if (distance < min_distance) {
                            std::cout << "Collision detected between particle " << particle.id() << " and particle " << p2.id() << std::endl;
                            // Handle collision response
                            double nx = dx / distance;
                            double ny = dy / distance;
                            double relative_velocity = (p2.GetVelocityX() - particle.GetVelocityX()) * nx + (p2.GetVelocityY() - particle.GetVelocityY()) * ny;

                            if (relative_velocity < 0) {
                                double impulse = 2 * relative_velocity / (1 + 1); // Assuming equal mass
                                particle.SetVelocity(particle.GetVelocityX() - impulse * nx,
                                                    particle.GetVelocityY() - impulse * ny);
                                p2.SetVelocity(p2.GetVelocityX() + impulse * nx,  
                                                p2.GetVelocityY() + impulse * ny);
                            }
                        }
                    }
                }
            }
        }


        // Get the updated position
        double d_position_x, d_position_y;
        particle.GetPosition(d_position_x, d_position_y);

        // Check for boundary collision and reverse velocity if needed
        if (d_position_x < d_min_x_) {
            d_position_x = d_min_x_;
            particle.SetVelocity(-particle.GetVelocityX() * DAMPING_FACTOR, particle.GetVelocityY());
        }
        if (d_position_x > d_max_x_) {
            d_position_x = d_max_x_;
            particle.SetVelocity(-particle.GetVelocityX() * DAMPING_FACTOR, particle.GetVelocityY());
        }
        if (d_position_y < d_min_y_) {
            d_position_y = d_min_y_;
            particle.SetVelocity(particle.GetVelocityX(), -particle.GetVelocityY() * DAMPING_FACTOR);
        }
        if (d_position_y > d_max_y_) {
            d_position_y = d_max_y_;
            particle.SetVelocity(particle.GetVelocityX(), -particle.GetVelocityY() * DAMPING_FACTOR);
        }

        // Update the particle's position if it was out of bounds
        particle.SetPosition(d_position_x, d_position_y);
    }
}

// Get all particle positions
std::vector<std::pair<double, double>> ParticleManager::GetParticlePositions() const {
    std::vector<std::pair<double, double>> positions;
    for (const auto& particle : particles_) {
        double d_x, d_y;
        particle.GetPosition(d_x, d_y);
        positions.emplace_back(d_x, d_y);
    }
    return positions;
}

int ParticleManager::GetParticleCount() const {
    return particles_.size();
}