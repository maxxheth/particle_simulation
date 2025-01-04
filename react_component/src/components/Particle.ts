export const GRAVITY = 30;
export const PARTICLE_RADIUS = 2;

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

    constructor(x: number, y: number, vx: number, vy: number, ax: number, ay: number, id: number) {
        this.pos = new Vector2(x, y);
        this.vel = new Vector2(vx, vy);
        this.acc = new Vector2(ax, ay);
        this.id_ = id;
    }

    update(timeStep: number): void {
        // Update velocity based on acceleration
        this.vel.x += this.acc.x * timeStep;
        this.vel.y += this.acc.y * timeStep;

        // Update position based on velocity
        this.pos.x += this.vel.x * timeStep;
        this.pos.y += this.vel.y * timeStep;
    }

    id(): number {
        return this.id_;
    }
} 