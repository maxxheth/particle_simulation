import React from 'react'
import { ParticleSimulation } from './components/ParticleSimulation'
import './App.css'

// WebM format is fully supported
// const VIDEO_URL = "https://example.com/your-video.webm"

const VIDEO_URL="http://mirrors.creativecommons.org/movingimages/webm/WannaWorkTogether_480p.webm"

function App() {
  return (
    <div className="app-container">
      <h1>Particle Simulation</h1>
      <ParticleSimulation 
        width={800} 
        height={600} 
        videoSrc={VIDEO_URL}
      />
      <p className="instructions">
        Click and drag to create particles that reveal the video underneath
      </p>
    </div>
  )
}

export default App
