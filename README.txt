# 3D Boids Simulation

A 3D interactive simulation of flocking behavior based on the Boids algorithm, developed using **C++** and **OpenGL**. The project models collective motion through three fundamental steering behaviors: **Separation, Cohesion, and Alignment**, allowing a flock of virtual birds to navigate an environment organically.


## Features

- **Flocking Algorithm**: Full implementation of local steering rules to simulate group behavior.
- **Dynamic Interaction**: Real-time control of the flock's target and dynamic addition/removal of members.
- **3D Graphics**: Polyhedral rendering with wing-flap animations and a global lighting system.
- **Visual Enhancements**: 
    - **Shadows**: Simple parallel projection of boids onto the ground plane.
    - **Fog**: Toggleable fog system for depth perception.
    - **Banking**: Dynamic rotation (roll) during turns for increased realism.
- **Multiple Perspectives**: Four camera modes (Tower, Follow, Side, and Aerial).
- **Environment**: Collision avoidance with spatial obstacles and a central tower.

## Controls

| Key | Action |
| :--- | :--- |
| **W / A / S / D** | Move target (Forward, Left, Backward, Right) |
| **Q / E** | Move target (Up / Down) |
| **1, 2, 3, 4** | Switch camera modes |
| **+ / -** | Add / Remove a boid |
| **F** | Toggle Fog |
| **P** | Pause / Resume simulation |
| **S** (while paused) | Step-by-step execution (Debug mode) |
| **O** | Generate random obstacle |
| **ESC** | Exit program |

## Tech Stack

- **Language**: C++17
- **Graphics**: OpenGL / GLUT
- **Build System**: CMake

## Getting Started

### Prerequisites
- CMake
- OpenGL and GLUT development libraries (e.g., `freeglut`)

### Build and Run (Linux/macOS)
```bash
git clone https://github.com/seu_usuario/nome_do_repositorio.git
cd nome_do_repositorio
mkdir build && cd build
cmake ..
make
./boids
```

---
*This project was developed as part of a Computer Graphics course at the Federal University of Minas Gerais (UFMG).*
