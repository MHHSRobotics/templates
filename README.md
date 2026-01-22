# FRC Robot Templates

Java robot templates for FRC teams built with WPILib and AdvantageKit.

## Templates

- **kitbot/** - Tank drive robot with intake/launcher mechanism
- **swerve/** - 4-wheel swerve drive with vision support (compatible with Phoenix Tuner X generated `TunerConstants`)

## Features

- **AdvantageKit Integration** - Full logging and replay support with three operating modes: real hardware, simulation, and log replay
- **IO Abstraction Layer** - Hardware interfaces (`MotorIO`, `GyroIO`, `EncoderIO`, `CameraIO`) with swappable implementations for different motor controllers (SparkMAX, TalonFX, TalonFXS)
- **Simulation Support** - Physics-based simulation for off-robot development
- **3D Visualization** - Robot pose publishing for AdvantageScope

## Getting Started

Each template is a standalone Gradle project. From within a template directory:

```bash
./gradlew build           # Build the project
./gradlew deploy          # Deploy to RoboRIO
./gradlew simulateJava    # Run simulation
```

## Acknowledgments

Inspired by [Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage) and their open-source robot code.
