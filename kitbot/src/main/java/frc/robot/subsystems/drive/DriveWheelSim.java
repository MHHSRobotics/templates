package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

// Simulates a single side of the tank drive (left or right)
public class DriveWheelSim extends SubsystemBase {
    // Motor model - 2 CIMs per side (front + back motors)
    private static final DCMotor driveGearbox = DCMotor.getCIM(2);

    // Moment of inertia for the wheel (kg*m^2) - estimate for a typical FRC drivetrain wheel
    private static final double wheelMOI = 0.025;

    private MotorIO motor;
    private DCMotorSim wheelMech;

    public DriveWheelSim(MotorIO motorIO) {
        this.motor = motorIO;
        this.wheelMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, wheelMOI, Drive.Constants.gearRatio), driveGearbox);
    }

    // Get the wheel's angular velocity in rad/s
    public double getAngularVelocityRadPerSec() {
        return wheelMech.getAngularVelocityRadPerSec();
    }

    // Get the wheel's linear velocity in m/s
    public double getLinearVelocityMetersPerSec() {
        return wheelMech.getAngularVelocityRadPerSec() * Drive.Constants.wheelRadiusMeters;
    }

    // Get the wheel's angular position in radians
    public double getAngularPositionRad() {
        return wheelMech.getAngularPositionRad();
    }

    @Override
    public void periodic() {
        // Apply voltage from motor controller to simulation
        wheelMech.setInputVoltage(motor.getInputs().appliedVoltage);

        // Update simulation physics
        wheelMech.update(Constants.loopTime);

        // Feed simulated position and velocity back to motor
        motor.setMechPosition(wheelMech.getAngularPositionRad());
        motor.setMechVelocity(wheelMech.getAngularVelocityRadPerSec());
    }
}
