package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModule {
    private MotorIO driveMotor;
    private MotorIO angleMotor;
    private EncoderIO angleEncoder;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private SwerveModulePosition lastPosition = new SwerveModulePosition();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    public SwerveModule(
            MotorIO driveMotorIO,
            MotorIO angleMotorIO,
            EncoderIO angleEncoderIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        driveMotor = driveMotorIO;
        angleMotor = angleMotorIO;
        angleEncoder = angleEncoderIO;

        driveMotor.setBraking(true);
        driveMotor.setGains(constants.DriveMotorGains);
        driveMotor.setGearRatio(constants.DriveMotorGearRatio);
        driveMotor.setStatorCurrentLimit(constants.SlipCurrent);
        driveMotor.setInverted(constants.DriveMotorInverted);

        angleEncoder.setInverted(constants.EncoderInverted);

        angleMotor.setBraking(true);
        angleMotor.setGains(constants.SteerMotorGains);
        angleMotor.connectEncoder(angleEncoder, constants.SteerMotorGearRatio);
        angleMotor.setContinuousWrap(true);
        angleMotor.setInverted(constants.SteerMotorInverted);
        angleMotor.setOffset(Units.rotationsToRadians(
                -constants.EncoderOffset)); // Fix encoder zero position (convert from rotations to radians)
    }

    // Sets whether the drive and angle motors should brake
    public void setLocked(boolean locked) {
        driveMotor.setBraking(locked);
        angleMotor.setBraking(locked);
    }

    // Sets whether the swerve module is disabled
    public void setDisabled(boolean disabled) {
        driveMotor.setDisabled(disabled);
        angleMotor.setDisabled(disabled);
    }

    // Tell the drive motor how much power to use (voltage in volts, like 12V battery)
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    // Tell the steering motor how much power to use (voltage in volts)
    public void setAngleVoltage(double voltage) {
        angleMotor.setVoltage(voltage);
    }

    // Tell the wheel how fast to spin (speed in radians per second)
    public void setDriveVelocity(double radPerSec) {
        driveMotor.setVelocityWithVoltage(radPerSec);
    }

    // Tell the wheel which direction to point (angle in radians, like 0 = forward)
    public void setAnglePosition(double position) {
        angleMotor.setGoalWithVoltage(position);
    }

    // Find out which direction the wheel is currently pointing (angle in radians)
    public double getAngle() {
        return angleMotor.getInputs().position;
    }

    // Get target angle of this module
    public double getTargetAngle() {
        return angleMotor.getInputs().setpoint;
    }

    // Find out how far the robot has driven (distance in meters)
    public double getPositionMeters() {
        return getWheelPosition() * constants.WheelRadius;
    }

    // Find out how fast the robot is moving (speed in meters per second)
    public double getVelocityMetersPerSec() {
        return driveMotor.getInputs().velocity * constants.WheelRadius;
    }

    // Get target velocity of this module
    public double getTargetVelocity() {
        return driveMotor.getInputs().setpoint * constants.WheelRadius;
    }

    // Find out how much the wheel has rotated (angle in radians)
    public double getWheelPosition() {
        return driveMotor.getInputs().position;
    }

    // Make the swerve module go a certain speed and direction (state has speed in m/s and angle in radians)
    public void runSetpoint(SwerveModuleState state) {
        // Smart optimization: instead of turning 180° and going forward, just go backward instead
        state.optimize(Rotation2d.fromRadians(getAngle()));

        // Slow down the wheel when it's still turning to the right angle (makes driving smoother)
        state.cosineScale(Rotation2d.fromRadians(getAngle()));

        // Convert robot speed (m/s) to wheel spin speed (rad/s) using wheel size
        setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);

        if (state.speedMetersPerSecond != 0) {
            setAnglePosition(state.angle.getRadians());
        }
    }

    // Special test mode for measuring how the robot moves
    public void runCharacterization(double volts) {
        setDriveVoltage(volts); // Apply test voltage
        setAnglePosition(0); // Keep wheel pointing straight forward
    }

    // Turn off all motors in this swerve module
    public void stop() {
        setDriveVoltage(0);
        setAngleVoltage(0);
    }

    // Get the module's current position: how far it's driven and which way it's pointing
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    // Get the position of the swerve module in the last tick
    public SwerveModulePosition getLastPosition() {
        return lastPosition;
    }

    // Gets the change in position in the last tick
    public SwerveModulePosition getPositionDelta() {
        return new SwerveModulePosition(
                currentPosition.distanceMeters - lastPosition.distanceMeters, currentPosition.angle);
    }

    // Get the module's current state: how fast it's going and which way it's pointing
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRadians(getAngle()));
    }

    // Get the module's target state
    public SwerveModuleState getTargetState() {
        return new SwerveModuleState(getTargetVelocity(), Rotation2d.fromRadians(getTargetAngle()));
    }

    // This runs every robot loop (about 50 times per second) to update sensors and check for problems
    public void periodic() {
        // All updates handle logging and alerts automatically
        driveMotor.update();
        angleMotor.update();
        angleEncoder.update();

        // Update gains
        driveMotor.setkP(Swerve.Constants.drivekP.get());
        driveMotor.setkD(Swerve.Constants.drivekD.get());
        driveMotor.setkS(Swerve.Constants.drivekS.get());
        driveMotor.setkV(Swerve.Constants.drivekV.get());
        driveMotor.setkA(Swerve.Constants.drivekA.get());

        angleMotor.setkP(Swerve.Constants.steerkP.get());
        angleMotor.setkD(Swerve.Constants.steerkD.get());
        angleMotor.setkS(Swerve.Constants.steerkS.get());
        angleMotor.setkV(Swerve.Constants.steerkV.get());
        angleMotor.setkA(Swerve.Constants.steerkA.get());

        // Update last position
        lastPosition = currentPosition;

        // Update current position
        currentPosition = new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRadians(getAngle()));
    }

    public void setDriveKP(double kP) {
        driveMotor.setkP(kP);
    }

    public void setDriveKI(double kI) {
        driveMotor.setkI(kI);
    }

    public void setDriveKD(double kD) {
        driveMotor.setkD(kD);
    }

    public void setDriveKS(double kS) {
        driveMotor.setkS(kS);
    }

    public void setDriveKV(double kV) {
        driveMotor.setkV(kV);
    }

    public void setDriveKA(double kA) {
        driveMotor.setkA(kA);
    }

    public void setAngleKP(double kP) {
        angleMotor.setkP(kP);
    }

    public void setAngleKI(double kI) {
        angleMotor.setkI(kI);
    }

    public void setAngleKD(double kD) {
        angleMotor.setkD(kD);
    }

    public void setAngleKS(double kS) {
        angleMotor.setkS(kS);
    }

    public void setAngleKV(double kV) {
        angleMotor.setkV(kV);
    }

    public void setAngleKA(double kA) {
        angleMotor.setkA(kA);
    }
}
