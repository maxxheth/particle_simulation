import { Particle, PARTICLE_RADIUS, Vector2 } from './Particle';

const DAMPING_FACTOR = 0.8;

export class ParticleManager {
    private particles: Particle[] = [];
    private gridCellSize = PARTICLE_RADIUS * 2.0;
    private gridWidth = 0;
    private gridHeight = 0;
    private grid: number[][][] = [];
    private minX = 0;
    private maxX = 0;
    private minY = 0;
    private maxY = 0;

    setBounds(minX: number, maxX: number, minY: number, maxY: number): void {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;

        this.gridHeight = Math.floor((maxY - minY) / this.gridCellSize);
        this.gridWidth = Math.floor((maxX - minX) / this.gridCellSize);

        // Initialize grid
        this.grid = Array(this.gridHeight).fill(null).map(() => 
            Array(this.gridWidth).fill(null).map(() => [])
        );
    }

    addParticle(x: number, y: number, vx: number, vy: number, ax: number, ay: number, id: number): void {
        this.particles.push(new Particle(x, y, vx, vy, ax, ay, id));
    }

    private getGridIndex(x: number, y: number): [number, number] {
        const gridX = Math.floor((x - this.minX) / this.gridCellSize);
        const gridY = Math.floor((y - this.minY) / this.gridCellSize);
        return [gridX, gridY];
    }

    private assignParticlesToGrid(): void {
        // Clear the grid
        for (let y = 0; y < this.gridHeight; y++) {
            for (let x = 0; x < this.gridWidth; x++) {
                this.grid[y][x] = [];
            }
        }

        // Assign particles to grid
        this.particles.forEach((particle, index) => {
            const [gridX, gridY] = this.getGridIndex(particle.pos.x, particle.pos.y);
            if (gridX >= 0 && gridX < this.gridWidth && gridY >= 0 && gridY < this.gridHeight) {
                this.grid[gridY][gridX].push(index);
            }
        });
    }

    updateParticles(timeStep: number): void {
        this.assignParticlesToGrid();

        this.particles.forEach((p1, i) => {
            p1.update(timeStep);

            const [gridX, gridY] = this.getGridIndex(p1.pos.x, p1.pos.y);

            // Check this cell and adjacent cells
            for (let dy = -1; dy <= 1; dy++) {
                for (let dx = -1; dx <= 1; dx++) {
                    const neighborX = gridX + dx;
                    const neighborY = gridY + dy;

                    if (neighborX >= 0 && neighborX < this.gridWidth && 
                        neighborY >= 0 && neighborY < this.gridHeight) {
                        
                        this.grid[neighborY][neighborX].forEach(j => {
                            const p2 = this.particles[j];

                            // Skip self
                            if (p1.id() === p2.id()) return;

                            const dx = p2.pos.x - p1.pos.x;
                            const dy = p2.pos.y - p1.pos.y;
                            const distance = Math.sqrt(dx * dx + dy * dy);
                            const minDistance = PARTICLE_RADIUS * 2.0;

                            if (distance < minDistance) {
                                // Handle collision response
                                const nx = dx / distance;
                                const ny = dy / distance;
                                const relativeVelocity = (p2.vel.x - p1.vel.x) * nx + 
                                                       (p2.vel.y - p1.vel.y) * ny;

                                if (relativeVelocity < 0) {
                                    const restitutionCoefficient = 0.95;
                                    const impulse = (1 + restitutionCoefficient) * relativeVelocity / 2;

                                    p1.vel.x += impulse * nx;
                                    p1.vel.y += impulse * ny;
                                    p2.vel.x -= impulse * nx;
                                    p2.vel.y -= impulse * ny;
                                }

                                // Adjust positions to prevent overlap
                                const overlap = minDistance - distance;
                                const correctionFactor = 0.5;

                                p1.pos.x -= overlap * nx * correctionFactor;
                                p1.pos.y -= overlap * ny * correctionFactor;
                                p2.pos.x += overlap * nx * correctionFactor;
                                p2.pos.y += overlap * ny * correctionFactor;
                            }
                        });
                    }
                }
            }

            // Check for boundary collision and reverse velocity if needed
            if (p1.pos.x < this.minX + PARTICLE_RADIUS) {
                p1.pos.x = this.minX + PARTICLE_RADIUS;
                p1.vel.x = -p1.vel.x * DAMPING_FACTOR;
            }
            if (p1.pos.x > this.maxX - PARTICLE_RADIUS) {
                p1.pos.x = this.maxX - PARTICLE_RADIUS;
                p1.vel.x = -p1.vel.x * DAMPING_FACTOR;
            }
            if (p1.pos.y < this.minY + PARTICLE_RADIUS) {
                p1.pos.y = this.minY + PARTICLE_RADIUS;
                p1.vel.y = -p1.vel.y * DAMPING_FACTOR;
            }
            if (p1.pos.y > this.maxY - PARTICLE_RADIUS) {
                p1.pos.y = this.maxY - PARTICLE_RADIUS;
                p1.vel.y = -p1.vel.y * DAMPING_FACTOR;
            }
        });
    }

    getParticlePositions(): [number, number][] {
        return this.particles.map(particle => [particle.pos.x, particle.pos.y]);
    }

    getParticles(): Particle[] {
        return this.particles;
    }

    getParticleCount(): number {
        return this.particles.length;
    }
} 