# Search and Rescue Robot

This project is part of the Embedded Systems course at the University of Coimbra. It aims to design and implement a search and rescue robot capable of navigating autonomously, detecting obstacles, reading RFID tags, and streaming live video. The robot is designed to aid in search and rescue missions by providing real-time data and visualization to users.

## Project Members
- Miguel Meireles Teixeira
- José Pedro da Cunha Rodrigues
- Ezequiel Juan Armando Alves Flores Rojas

## Head Teacher
- Lino José Forte Marques

## Class
- PL1

## Academic Year
- 2023/2024

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Engineering V Diagram](#system-engineering-v-diagram)
3. [Tasks and Architecture](#tasks-and-architecture)
   - [Requirements Analysis](#requirements-analysis)
   - [System Architecture](#system-architecture)
   - [System Design](#system-design)
   - [Software Architecture](#software-architecture)
   - [Software Design](#software-design)
   - [Unit Tests](#unit-tests)
   - [System Integration](#system-integration)
   - [Integration Testing](#integration-testing)
   - [Acceptance & Use](#acceptance-and-use)
   - [Additional Ideas](#additional-ideas)
4. [Final Project Review](#final-project-review)
   - [Sensors](#sensors)
   - [Odometry](#odometry)
   - [SPI Communication](#spi-communication)
   - [Web Server](#web-server)
   - [Collision Avoidance](#collision-avoidance)
   - [Move To Target](#move-to-target)

## Project Overview

The primary objective of this project is to create a robust, efficient, and reliable search and rescue robot. The robot uses various sensors for navigation, obstacle detection, and target identification. It communicates with a web server for real-time control and monitoring.

## System Engineering V Diagram

![System Engineering V Diagram](path_to_v_diagram_image)

## Tasks and Architecture

### Requirements Analysis

- **Gathering Requirements:** Understand the mission and constraints. Research existing solutions.
- **Defining Functional Requirements:** Specify the robot's functionalities, including movement, obstacle detection, RFID reading, and position reporting.
- **Defining User Interface:** Outline the necessary UI functionalities for real-time monitoring and control.

### System Architecture

- **Selecting Hardware:** Choose sensors, motors, microcontrollers (STM32F411CEU6, ESP32-WROOM-32D), and communication devices.
- **High-Level Design:** Create block diagrams to show hardware interactions.
- **Defining Interfaces:** Specify communication protocols (SPI) between hardware components.

### System Design

- **Detailed Component Layout:** Design the physical layout within the robot’s chassis.
- **Wiring Schematics:** Create detailed wiring diagrams for electrical connections.

### Software Architecture

- **Module Breakdown:** Identify software modules for navigation, RFID detection, and communication.
- **Layered Architecture:** Define a layered architecture using FreeRTOS, semaphores, interrupts, SPI, and timers.

### Software Design

- **Algorithm Design:** Develop algorithms for navigation, obstacle avoidance, and search patterns.
- **Interface Design:** Design interfaces for module interaction and hardware communication.
- **User Interface:** Design a remote monitoring and control interface.

### Unit Tests

- **Test Case Development:** Create test cases for each module function.
- **Documentation:** Record test results and anomalies for further investigation.

### System Integration

- **Physical Assembly:** Assemble hardware components.
- **Functional Verification:** Verify each component's operation.
- **Subsystem Integration:** Integrate and test subsystems incrementally.

### Integration Testing

- **Integration Test Plan:** Develop and execute integration test plans.
- **Performance Testing:** Test robot performance under varying conditions.
- **User Scenario Testing:** Simulate search and rescue scenarios.

### Acceptance and Use

- **User Acceptance Testing (UAT):** Ensure the robot meets expectations and requirements.
- **Field Testing:** Test the robot in real-world conditions.
- **Final Demonstration:** Demonstrate the robot’s capabilities.

### Additional Ideas

- **User Interface:** Design a UI to visualize the robot’s position and movements on a 2D graph.
- **Live Stream Camera:** Integrate a live video stream feature using ESP32-CAM.

## Final Project Review

### Sensors

- **Left LIDAR Sensor:** Detects obstacles on the left.
- **Middle LIDAR Sensor:** Intended for front obstacle detection, but not used due to communication issues.
- **Right LIDAR Sensor:** Detects obstacles on the right.
- **RFID Tag Reader:** Identifies RFID tags representing victims.
- **ADC Gas Sensor:** Detects hazardous gases.

### Odometry

- **Key Components:** Encoders for each wheel, variables for distances, and position tracking.
- **Odometry Calculation:** Steps to calculate the robot’s position and orientation from encoder data.

### SPI Communication

- **Sensor Values:** Read and transmit sensor data between ESP32 and STM.
- **Coordinate Transmission:** Send target coordinates from web server to STM.
- **Real-Time Tracking:** Monitor robot’s position via the web server.

### Web Server

- **User Interface:** Input target coordinates and monitor the robot’s status.
- **Real-Time Updates:** Display real-time tracking information.

### Collision Avoidance

- **Sensor Reading:** Continuous monitoring by LIDAR sensors.
- **Safety Distance Check:** Ensure safe distance from obstacles.
- **Avoidance Maneuver:** Stop and pivot to avoid obstacles.
- **Resume Movement:** Continue towards the target once the path is clear.

### Move To Target

- **Objective:** Navigate to user-provided coordinates.
- **Components and Variables:** Position and velocity variables, control gains.
- **Main Loop Functionality:** Read position, calculate errors, compute control laws, and adjust motor controls.

---

This project showcases the integration of various hardware and software components to achieve a functional search and rescue robot. The detailed tasks and systematic approach ensure thorough development and testing, leading to a reliable and efficient system.
