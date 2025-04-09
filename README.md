# Tomato Picking Robot Raspberry Part

## Overview

This Repository is a code base for the Tomato Picking Robot i was tasked of creating for my Master's Degree end of cycle project. This robot is a supposed to be a compact yet modular system that can be retrofited for the task you want.

## Mechanical System

### Core Components

- **Base Platform**

  -2DC Motors that enablge the system to move linearly but also rotates around its inertia axis (typically in the middle of the robot).

- **Vertical Movement System**

  - Three-stage cascade lift mechanism
  - Enables precise height adjustment for reaching tomatoes at different levels
  - Designed for stable operation throughout movement range

- **Extension System**

  - Telescopic arm assembly
  - Allow the system to reach on the farthest point of an hydroponic shelf
  -

- **Picking Mechanism**
  - Mechanized gripper system
  - Designed for gentle tomato handling
  - Controlled compression to prevent damage

## Software Architecture

### System Components

1. **Central Processing Unit (Raspberry Pi)**

   - Primary hub for sensor processing and AI operations
   - Handles camera feed analysis
   - Coordinates overall system control
   - Acts as message broker for distributed commands

2. **Motion Control System (Arduino)**
   - Controls mechanical movements
   - Manages motor operations
   - Processes movement commands
   - Can be alternatively implemented with ESP32 for MQTT integration

## Future Development Possibilities

- Implementation of ESP32 for enhanced connectivity
- Advanced computer vision algorithms
- Integration with farming automation systems
- Expansion to handle multiple tomato varieties
- Enhanced navigation algorithms for complex environments

## Getting Started

The project is split into two main repositories:

1. Main repository (current): Contains Raspberry Pi implementation
2. Secondary repository: Houses Arduino/ESP32 control software

Begin by reviewing the mechanical documentation and ensuring all hardware components are properly assembled before implementing the software components.

**Note:** This README focuses on the main project structure. Detailed implementation guides and technical specifications can be found in the respective component directories.
