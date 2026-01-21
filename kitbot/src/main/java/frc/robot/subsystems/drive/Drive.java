package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.GyroIO;
import frc.robot.io.MotorIO;
import frc.robot.util.Field;

// Tank drive subsystem for differential drive robots.
// Uses two motors (left and right) to control robot movement.
// Units:
// - Distances in meters (m)
// - Linear speeds in meters per second (m/s)
// - Angles in radians
// - Angular speeds in radians per second (rad/s)
public class Drive extends SubsystemBase {
    public static class Constants {
        // Deadband for joystick inputs (0 to 1 range)
        public static final double moveDeadband = 0.1;
        public static final double turnDeadband = 0.1;

        // Power curve for smoother control (raise input to this power)
        public static final double movePow = 2;
        public static final double turnPow = 2;

        // Track width - distance between left and right wheels (meters)
        public static final double trackWidthMeters = 0.546;

        // Wheel radius (meters) - adjust for your robot's wheels
        public static final double wheelRadiusMeters = Units.inchesToMeters(3);

        // Gear ratio from motor to wheel (motor rotations per wheel rotation)
        public static final double gearRatio = 8.45;

        // Maximum robot speeds (m/s and rad/s)
        public static final double maxLinearSpeedMetersPerSec = 4.0;
        public static final double maxAngularSpeedRadPerSec = maxLinearSpeedMetersPerSec / (trackWidthMeters / 2);

        // Toggle to disable all drive motors
        public static final LoggedNetworkBoolean driveDisabled = new LoggedNetworkBoolean("Drive/Disabled", false);

        // Drive motor PID gains
        public static final LoggedNetworkNumber driveKP = new LoggedNetworkNumber("Drive/KP", 0);
        public static final LoggedNetworkNumber driveKI = new LoggedNetworkNumber("Drive/KI", 0);
        public static final LoggedNetworkNumber driveKD = new LoggedNetworkNumber("Drive/KD", 0);
        public static final LoggedNetworkNumber driveKS = new LoggedNetworkNumber("Drive/KS", 0);
        public static final LoggedNetworkNumber driveKV = new LoggedNetworkNumber("Drive/KV", 0.2);
        public static final LoggedNetworkNumber driveKA = new LoggedNetworkNumber("Drive/KA", 0);

        // Motor CAN IDs
        public static final int leftFrontMotorId = 1;
        public static final int leftBackMotorId = 2;
        public static final int rightFrontMotorId = 3;
        public static final int rightBackMotorId = 4;
    }

    // Motor controllers for left and right sides (front = leaders, back = followers)
    private MotorIO leftFrontMotor;
    private MotorIO leftBackMotor;
    private MotorIO rightFrontMotor;
    private MotorIO rightBackMotor;

    // Gyro for heading measurement
    private GyroIO gyro;

    // Current robot heading from gyro
    private Rotation2d gyroAngle = new Rotation2d();

    // Kinematics for converting chassis speeds to wheel speeds
    private DifferentialDriveKinematics kinematics;

    // Pose estimator for tracking robot position on the field
    private DifferentialDrivePoseEstimator estimator;

    // Previous wheel positions for delta calculation
    private double prevLeftPositionMeters = 0;
    private double prevRightPositionMeters = 0;

    // Target speeds for arcade/tank drive (m/s)
    private double targetLeftSpeed = 0; // m/s
    private double targetRightSpeed = 0; // m/s

    // For Elastic/Glass visualization
    private Field2d field = new Field2d();

    public Drive(
            GyroIO gyro,
            MotorIO leftFrontMotor,
            MotorIO leftBackMotor,
            MotorIO rightFrontMotor,
            MotorIO rightBackMotor) {
        this.gyro = gyro;
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;

        // Configure motors
        configureMotors();

        // Initialize kinematics
        kinematics = new DifferentialDriveKinematics(Constants.trackWidthMeters);

        // Initialize pose estimator
        Pose2d initialPose = new Pose2d(Field.fieldLength / 2, Field.fieldWidth / 2, Rotation2d.kZero);

        estimator = new DifferentialDrivePoseEstimator(
                kinematics,
                gyroAngle,
                0, // left position
                0, // right position
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // state std devs
                VecBuilder.fill(1, 1, 1)); // vision std devs (not used currently)

        // Send field visual to SmartDashboard
        SmartDashboard.putData("Field", field);

        // Reset gyro to 0
        resetGyro();
    }

    private void configureMotors() {
        // Set gear ratios for wheel distance calculation
        // Position will be in radians of wheel rotation
        leftFrontMotor.setGearRatio(Constants.gearRatio);
        rightFrontMotor.setGearRatio(Constants.gearRatio);

        // Right motors are inverted on a tank drive
        rightFrontMotor.setInverted(true);
        rightBackMotor.setInverted(true);

        // Configure back motors to follow front motors
        leftBackMotor.follow(Constants.leftFrontMotorId, false);
        rightBackMotor.follow(Constants.rightFrontMotorId, false);

        // Set initial PID gains on leader motors
        leftFrontMotor.setkP(Constants.driveKP.get());
        leftFrontMotor.setkI(Constants.driveKI.get());
        leftFrontMotor.setkD(Constants.driveKD.get());
        leftFrontMotor.setkS(Constants.driveKS.get());
        leftFrontMotor.setkV(Constants.driveKV.get());
        leftFrontMotor.setkA(Constants.driveKA.get());

        rightFrontMotor.setkP(Constants.driveKP.get());
        rightFrontMotor.setkI(Constants.driveKI.get());
        rightFrontMotor.setkD(Constants.driveKD.get());
        rightFrontMotor.setkS(Constants.driveKS.get());
        rightFrontMotor.setkV(Constants.driveKV.get());
        rightFrontMotor.setkA(Constants.driveKA.get());

        // Enable braking when stopped
        leftFrontMotor.setBraking(true);
        leftBackMotor.setBraking(true);
        rightFrontMotor.setBraking(true);
        rightBackMotor.setBraking(true);
    }

    // Get left wheel position in meters: distance = angle * radius
    // Motor inputs.position is in wheel radians (mechanism units)
    public double getLeftPositionMeters() {
        return leftFrontMotor.getInputs().position * Constants.wheelRadiusMeters; // wheel rad * m/rad = m
    }

    // Get right wheel position in meters: distance = angle * radius
    // Motor inputs.position is in wheel radians (mechanism units)
    public double getRightPositionMeters() {
        return rightFrontMotor.getInputs().position * Constants.wheelRadiusMeters; // wheel rad * m/rad = m
    }

    // Get left wheel velocity in m/s: v = omega * radius
    // Motor inputs.velocity is in wheel rad/s (mechanism units)
    public double getLeftVelocityMetersPerSec() {
        return leftFrontMotor.getInputs().velocity * Constants.wheelRadiusMeters; // wheel rad/s * m/rad = m/s
    }

    // Get right wheel velocity in m/s: v = omega * radius
    // Motor inputs.velocity is in wheel rad/s (mechanism units)
    public double getRightVelocityMetersPerSec() {
        return rightFrontMotor.getInputs().velocity * Constants.wheelRadiusMeters; // wheel rad/s * m/rad = m/s
    }

    // Get current wheel positions
    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(getLeftPositionMeters(), getRightPositionMeters());
    }

    // Get current wheel speeds
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    // Get the robot's current field position
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    // Get the robot's current heading
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    // Reset the pose estimator to a known field position
    public void setPose(Pose2d newPose) {
        estimator.resetPosition(gyroAngle, getLeftPositionMeters(), getRightPositionMeters(), newPose);
    }

    // Reset the gyro to 0
    public void resetGyro() {
        gyro.setYaw(0);
        gyroAngle = Rotation2d.kZero;
    }

    // Drive using arcade controls (forward/backward + turn)
    public void arcadeDrive(double forward, double turn) {
        double left = forward + turn;
        double right = forward - turn;

        // Normalize if either exceeds 1
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if (maxMagnitude > 1.0) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }

        tankDrive(left, right);
    }

    // Drive using tank controls (left speed, right speed) as percentages (-1 to 1)
    public void tankDrive(double leftPercent, double rightPercent) {
        targetLeftSpeed = leftPercent * Constants.maxLinearSpeedMetersPerSec;
        targetRightSpeed = rightPercent * Constants.maxLinearSpeedMetersPerSec;
    }

    // Drive using wheel speeds in m/s
    public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
        targetLeftSpeed =
                MathUtil.clamp(leftSpeed, -Constants.maxLinearSpeedMetersPerSec, Constants.maxLinearSpeedMetersPerSec);
        targetRightSpeed =
                MathUtil.clamp(rightSpeed, -Constants.maxLinearSpeedMetersPerSec, Constants.maxLinearSpeedMetersPerSec);
    }

    // Drive using chassis speeds (vx forward, omega rotation)
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        setWheelSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    // Stop the drive
    public void stop() {
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
    }

    // Get the kinematics object
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    // Get the left front motor (for simulation)
    public MotorIO getLeftFrontMotor() {
        return leftFrontMotor;
    }

    // Get the right front motor (for simulation)
    public MotorIO getRightFrontMotor() {
        return rightFrontMotor;
    }

    // Get the gyro (for simulation)
    public GyroIO getGyro() {
        return gyro;
    }

    @Override
    public void periodic() {
        // Update motor inputs (all 4 motors for logging)
        leftFrontMotor.update();
        leftBackMotor.update();
        rightFrontMotor.update();
        rightBackMotor.update();

        // Update motor disabled state
        leftFrontMotor.setDisabled(Constants.driveDisabled.get());
        leftBackMotor.setDisabled(Constants.driveDisabled.get());
        rightFrontMotor.setDisabled(Constants.driveDisabled.get());
        rightBackMotor.setDisabled(Constants.driveDisabled.get());

        // Update PID gains from dashboard (only on leader motors)
        leftFrontMotor.setkP(Constants.driveKP.get());
        leftFrontMotor.setkI(Constants.driveKI.get());
        leftFrontMotor.setkD(Constants.driveKD.get());
        leftFrontMotor.setkS(Constants.driveKS.get());
        leftFrontMotor.setkV(Constants.driveKV.get());
        leftFrontMotor.setkA(Constants.driveKA.get());

        rightFrontMotor.setkP(Constants.driveKP.get());
        rightFrontMotor.setkI(Constants.driveKI.get());
        rightFrontMotor.setkD(Constants.driveKD.get());
        rightFrontMotor.setkS(Constants.driveKS.get());
        rightFrontMotor.setkV(Constants.driveKV.get());
        rightFrontMotor.setkA(Constants.driveKA.get());

        // Set motor velocities on leader motors
        // Convert linear speed (m/s) to wheel angular velocity (rad/s): omega = v / r
        // Back motors follow automatically
        if (!Constants.driveDisabled.get()) {
            double leftWheelRadPerSec = targetLeftSpeed / Constants.wheelRadiusMeters; // wheel rad/s
            double rightWheelRadPerSec = targetRightSpeed / Constants.wheelRadiusMeters; // wheel rad/s
            leftFrontMotor.setVelocityWithVoltage(leftWheelRadPerSec);
            rightFrontMotor.setVelocityWithVoltage(rightWheelRadPerSec);
        }

        // Update gyro
        gyro.update();

        if (gyro.getInputs().connected) {
            // If gyro is connected, read the angle
            gyroAngle = Rotation2d.fromRadians(gyro.getInputs().yawPositionRad);
        } else {
            // If gyro is disconnected (like in some sims), estimate from wheel odometry
            double leftDelta = getLeftPositionMeters() - prevLeftPositionMeters;
            double rightDelta = getRightPositionMeters() - prevRightPositionMeters;
            Twist2d twist = kinematics.toTwist2d(leftDelta, rightDelta);
            gyroAngle = gyroAngle.plus(Rotation2d.fromRadians(twist.dtheta));
        }

        // Update previous positions
        prevLeftPositionMeters = getLeftPositionMeters();
        prevRightPositionMeters = getRightPositionMeters();

        // Update pose estimator
        estimator.updateWithTime(
                RobotController.getFPGATime() / 1000000.0,
                gyroAngle,
                getLeftPositionMeters(),
                getRightPositionMeters());

        // Log data
        Logger.recordOutput("Drive/LeftPositionMeters", getLeftPositionMeters());
        Logger.recordOutput("Drive/RightPositionMeters", getRightPositionMeters());
        Logger.recordOutput("Drive/LeftVelocityMPS", getLeftVelocityMetersPerSec());
        Logger.recordOutput("Drive/RightVelocityMPS", getRightVelocityMetersPerSec());
        Logger.recordOutput("Drive/TargetLeftSpeedMPS", targetLeftSpeed);
        Logger.recordOutput("Drive/TargetRightSpeedMPS", targetRightSpeed);
        Logger.recordOutput("Drive/GyroAngleRad", gyroAngle.getRadians());
        Logger.recordOutput("Drive/EstimatedPose", getPose());
        Logger.recordOutput("Drive/ChassisSpeeds", kinematics.toChassisSpeeds(getWheelSpeeds()));

        // Update field pose for visualization
        field.setRobotPose(getPose());
    }
}
