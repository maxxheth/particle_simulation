export const GRAVITY = 1000;
export const PARTICLE_RADIUS = 2;
export const FRAME_DT = 1/60;

export class Vector2 {
    constructor(public x: number, public y: number) {}

    norm(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }
}

export class Particle {
    pos: Vector2;
    vel: Vector2;
    acc: Vector2;
    private id_: number;
    createdAt: number;

    constructor(x: number, y: number, vx: number, vy: number, ax: number, ay: number, id: number) {
        this.pos = new Vector2(x, y);
        this.vel = new Vector2(vx, vy);
        this.acc = new Vector2(ax, ay);
        this.id_ = id;
        this.createdAt = performance.now();
    }

    update(timeStep: number): void {
        // Update velocity with gravity and acceleration
        this.vel.x += this.acc.x * timeStep;
        this.vel.y += this.acc.y * timeStep;

        // Update position with velocity
        this.pos.x += this.vel.x * timeStep;
        this.pos.y += this.vel.y * timeStep;
    }

    id(): number {
        return this.id_;
    }

    setId(newId: number): void {
        this.id_ = newId;
    }
} 