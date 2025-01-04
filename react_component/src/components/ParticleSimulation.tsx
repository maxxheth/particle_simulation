import React, { useEffect, useRef, useState } from 'react';
import { ParticleManager } from './ParticleManager';
import { GRAVITY, PARTICLE_RADIUS } from './Particle';

interface ParticleSimulationProps {
    width?: number;
    height?: number;
}

export const ParticleSimulation: React.FC<ParticleSimulationProps> = ({ 
    width = 600, 
    height = 600 
}) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const requestIdRef = useRef<number>();
    const particleManagerRef = useRef<ParticleManager>(new ParticleManager());
    const lastGenTimeRef = useRef<number>(0);
    const [isMousePressed, setIsMousePressed] = useState(false);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

    const MIN_GEN_DT = 0.01;
    const FRAME_DT = 0.01;

    useEffect(() => {
        const particleManager = particleManagerRef.current;
        particleManager.setBounds(0, width, 0, height);

        const canvas = canvasRef.current;
        if (!canvas) return;

        let wildcard = 0.01;
        const increment = 0.01;

        const animate = () => {
            const ctx = canvas.getContext('2d');
            if (!ctx) return;

            // Clear canvas
            ctx.fillStyle = 'black';
            ctx.fillRect(0, 0, width, height);

            // Update particles
            wildcard += increment;

            const frameDt = FRAME_DT * wildcard;
            particleManager.updateParticles(frameDt);

            // Draw particles
            particleManager.getParticles().forEach(particle => {
                const velocity = particle.vel.norm();
                let r = 0, g = 0, b = 1; // Default to blue

                if (velocity >= 200.0) {
                    r = 1;
                    b = 0;
                } else if (velocity > 0.0) {
                    r = velocity / 200.0 * wildcard;
                    b = 1 - r;
                }

                ctx.beginPath();
                ctx.fillStyle = `rgb(${r * 255}, ${g * 255}, ${b * 255})`;
                ctx.arc(particle.pos.x, height - particle.pos.y, PARTICLE_RADIUS, 0, Math.PI * 2);
                ctx.fill();
            });

            // Draw particle count
            ctx.fillStyle = 'white';
            ctx.font = '18px Helvetica';
            ctx.fillText(`Particle Count: ${particleManager.getParticleCount()}`, 10, 30);

            requestIdRef.current = requestAnimationFrame(animate);
        };

        animate();

        return () => {
            if (requestIdRef.current) {
                cancelAnimationFrame(requestIdRef.current);
            }
        };
    }, [width, height]);

    const addParticles = (x: number, y: number) => {
        const currentTime = performance.now() / 1000;
        if (currentTime - lastGenTimeRef.current > MIN_GEN_DT) {
            const physicsY = height - y;
            for (let i = 0; i < 10; i++) {
                particleManagerRef.current.addParticle(
                    x - 10.0 + i * 2.0,
                    physicsY,
                    0.0,
                    0.0,
                    0.0,
                    -GRAVITY,
                    particleManagerRef.current.getParticleCount()
                );
            }
            lastGenTimeRef.current = currentTime;
        }
    };

    const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
        setIsMousePressed(true);
        const rect = canvasRef.current?.getBoundingClientRect();
        if (rect) {
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            setMousePos({ x, y });
            addParticles(x, y);
        }
    };

    const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
        if (isMousePressed) {
            const rect = canvasRef.current?.getBoundingClientRect();
            if (rect) {
                const x = e.clientX - rect.left;
                const y = e.clientY - rect.top;
                setMousePos({ x, y });
                addParticles(x, y);
            }
        }
    };

    const handleMouseUp = () => {
        setIsMousePressed(false);
    };

    return (
        <canvas
            ref={canvasRef}
            width={width}
            height={height}
            style={{ border: '1px solid #000' }}
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
        />
    );
}; 