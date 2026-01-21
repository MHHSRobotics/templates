package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

// Simulates a single side of the tank drive (left or right).
// The DCMotorSim is configured with the gear ratio, so it outputs WHEEL (mechanism) units,
// not motor units. This matches the convention used by MotorIO where inputs.position
// and inputs.velocity are in mechanism units (wheel rad, wheel rad/s).
public class DriveWheelSim extends SubsystemBase {
    // Motor model - 2 CIMs per side (front + back motors)
    private static final DCMotor driveGearbox = DCMotor.getCIM(2);

    // Moment of inertia for the wheel (kg*m^2) - estimate for a typical FRC drivetrain wheel
    private static final double wheelMOI = 0.025;

    private MotorIO motor;
    private DCMotorSim wheelMech; // Simulates wheel (mechanism) dynamics, not motor dynamics

    public DriveWheelSim(MotorIO motorIO) {
        this.motor = motorIO;
        // Create a DCMotorSim with the gear ratio - this makes the sim output in wheel units
        // Input: motor voltage (V)
        // Output: wheel position (rad), wheel velocity (rad/s)
        this.wheelMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, wheelMOI, Drive.Constants.gearRatio), driveGearbox);
    }

    // Get the wheel's angular velocity (rad/s)
    public double getAngularVelocityRadPerSec() {
        return wheelMech.getAngularVelocityRadPerSec(); // wheel rad/s
    }

    // Get the wheel's linear velocity (m/s): v = omega * radius
    public double getLinearVelocityMetersPerSec() {
        return wheelMech.getAngularVelocityRadPerSec() * Drive.Constants.wheelRadiusMeters; // m/s
    }

    // Get the wheel's angular position (rad)
    public double getAngularPositionRad() {
        return wheelMech.getAngularPositionRad(); // wheel rad
    }

    @Override
    public void periodic() {
        // Apply voltage from motor controller to simulation (V)
        wheelMech.setInputVoltage(motor.getInputs().appliedVoltage);

        // Update simulation physics
        wheelMech.update(Constants.loopTime);

        // Feed simulated wheel position and velocity back to motor IO
        // These are in mechanism (wheel) units, which MotorIO converts to motor units internally
        motor.setMechPosition(wheelMech.getAngularPositionRad()); // wheel rad
        motor.setMechVelocity(wheelMech.getAngularVelocityRadPerSec()); // wheel rad/s
    }
}
