# Drone Simulation and Control Project

## Overview
This project focuses on developing a drone control system using a Linux-based drone simulator, a Raspberry Pi running TRENTOS, and a network communication system for real-time drone operation. The main goal is to enable a drone to detect objects using LiDAR sensors, navigate to a designated landing target, and execute a safe landing.

## Challenges and Solutions

### Setup Challenges:
- Configuring the **Drone Simulator** on a Linux machine.
- Running the **Server Program** on Linux using Python.
- Establishing a **network connection** between the Raspberry Pi and the Linux computer.
- Developing the **Client Program** on the Raspberry Pi using **C and TRENTOS**.

### TRENTOS Setup Challenges:
- Proper **TRENTOS configuration** for drone operation.
- Implementing **network communication** between the Raspberry Pi and the drone simulator.
- Developing an efficient **drone control logic** to process sensor data and execute commands.

## Drone Control Logic

### Object Detection with LiDAR
#### Hardware:
- **Three LiDAR sensors**: One horizontal and two vertical.
- LiDAR data format: **Point cloud** (each point has x, y, z coordinates).

#### Detection Process:
1. **Horizontal LiDAR Scanning**:
   - Move the drone **slowly upward** while recording horizontal LiDAR scan data.
   - Generate **slices** of the surrounding environment based on LiDAR rotation.
   - Stop the drone when the LiDAR detects **no further obstacles**.

2. **Analyzing the Data**:
   - The **last slice** provides the **highest surrounding objects**, helping identify potential landing spots.
   - **All slices combined** help:
     - Reduce data complexity by selecting only one point per object.
     - Determine object height by comparing different slices.
     - Create a **map** of object locations and heights.

### Navigating to the Landing Target
- Use a **ground-facing distance sensor** to measure altitude.
- The drone **rotates to face** the selected landing target.
- It flies **in a straight line** at controlled speed.
- Once the landing target is detected **directly below**, the drone stops.

### Finding a Safe Landing Position
- Utilize **two vertical LiDAR sensors** positioned **90 degrees apart**.
- The drone performs **two perpendicular scans**:
  1. First scan: Determines an estimated center of the landing zone.
  2. Second scan (after rotating **45 degrees**): Refines the center estimate.
- The drone selects a **safe landing point**, ensuring it is **away from edges**.
- It then **slowly descends and lands** at the calculated position.

## Practical Implementation
The project successfully integrates **hardware and software** to enable precise drone navigation and landing. The combination of **TRENTOS-based control**, **networked communication**, and **LiDAR-based detection** ensures that the drone can autonomously detect its environment, find a landing site, and execute a controlled descent.

---
This README provides an overview of the challenges we tackled, the solutions we implemented, and how the drone's navigation and landing logic was developed. Future improvements could focus on optimizing flight speed, enhancing obstacle detection, and improving landing accuracy.

