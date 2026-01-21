package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.CameraIO;
import frc.robot.io.CameraIO.CameraIOInputs;
import frc.robot.io.GyroIO;
import frc.robot.util.Field;
import frc.robot.util.FieldPose2d;
import frc.robot.util.RobotUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;

// Make the swerve drive move the robot in any direction and rotate at the same time.
// Uses a pose estimator to keep track of where the robot is on the field.
// Units:
// - Distances in meters (m)
// - Linear speeds in meters per second (m/s)
// - Angles in radians
// - Angular speeds in radians per second (rad/s)
public class Swerve extends SubsystemBase {
    public static class Constants {
        public static final double moveDeadband =
                0.1; // How far the stick must move from center before the robot starts translating (0 to 1 range)

        public static final double turnDeadband =
                0.1; // How far the stick must move from center before the robot starts turning (0 to 1 range)

        // Smart shortcut to make small moves easier: raise input to a power.
        // Example: stick = 0.5, movePow = 2 -> 0.5^2 = 0.25 (finer control near center)
        public static final double movePow = 2;

        // Same idea as movePow but for turning
        public static final double turnPow = 2;

        // The maximum speed of the robot, obtained via characterization
        public static final double maxLinearSpeedMetersPerSec = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        // Radius of the drivebase: distance from center of bot to furthest swerve module
        public static final double driveBaseRadius = Math.max(
                Math.max(
                        Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                        Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
                Math.max(
                        Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                        Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

        // Maximum angular speed of the robot
        public static final double maxAngularSpeedRadPerSec = maxLinearSpeedMetersPerSec / driveBaseRadius;

        // Initial pose of the bot in simulation
        public static final FieldPose2d simInitialPose =
                new FieldPose2d(new Pose2d(3.5, Field.fieldWidth / 2, Rotation2d.k180deg));

        public static final LoggedNetworkBoolean swerveLocked =
                new LoggedNetworkBoolean("Swerve/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean swerveDisabled = new LoggedNetworkBoolean(
                "Swerve/Disabled", false); // Toggle to completely disable all motors in the swerve subsystem

        public static final LoggedNetworkBoolean swerveFieldCentric =
                new LoggedNetworkBoolean("Swerve/FieldCentric", true); // Toggle for field centric controls

        // Drive motor PID
        public static final LoggedNetworkNumber driveKP = new LoggedNetworkNumber("Swerve/DriveKP", 0.1);
        public static final LoggedNetworkNumber driveKI = new LoggedNetworkNumber("Swerve/DriveKI", 0);
        public static final LoggedNetworkNumber driveKD = new LoggedNetworkNumber("Swerve/DriveKD", 0);
        public static final LoggedNetworkNumber driveKS = new LoggedNetworkNumber("Swerve/DriveKS", 0);
        public static final LoggedNetworkNumber driveKV = new LoggedNetworkNumber("Swerve/DriveKV", 0.124);
        public static final LoggedNetworkNumber driveKA = new LoggedNetworkNumber("Swerve/DriveKA", 0);

        // Steer motor PID
        public static final LoggedNetworkNumber steerKP = new LoggedNetworkNumber("Swerve/SteerKP", 20);
        public static final LoggedNetworkNumber steerKI = new LoggedNetworkNumber("Swerve/SteerKI", 0);
        public static final LoggedNetworkNumber steerKD = new LoggedNetworkNumber("Swerve/SteerKD", 0);
        public static final LoggedNetworkNumber steerKS = new LoggedNetworkNumber("Swerve/SteerKS", 0);
        public static final LoggedNetworkNumber steerKV = new LoggedNetworkNumber("Swerve/SteerKV", 0);
        public static final LoggedNetworkNumber steerKA = new LoggedNetworkNumber("Swerve/SteerKA", 0);

        // Auto align translation PID
        public static final LoggedNetworkNumber translationkP = new LoggedNetworkNumber("Swerve/TransKP", 2);
        public static final LoggedNetworkNumber translationkD = new LoggedNetworkNumber("Swerve/TransKD", 0);
        public static final LoggedNetworkNumber translationkI = new LoggedNetworkNumber("Swerve/TransKI", 0);

        // Auto align rotation PID
        public static final LoggedNetworkNumber rotationkP = new LoggedNetworkNumber("Swerve/RotKP", 0.4);
        public static final LoggedNetworkNumber rotationkD = new LoggedNetworkNumber("Swerve/RotKD", 0);
        public static final LoggedNetworkNumber rotationkI = new LoggedNetworkNumber("Swerve/RotKI", 0);
    }

    public static class VisionConstants {
        // Vision standard deviation tuning constants
        // Base XY standard deviation in meters (tune based on testing)
        public static final double visionXYStdDevBase = 0.5;
        // XY standard deviation multiplier based on distance squared
        public static final double visionXYStdDevDistanceMultiplier = 0.1;
        // Base theta standard deviation in radians
        public static final double visionThetaStdDevBase = 0.5;
        // Theta standard deviation multiplier based on distance
        public static final double visionThetaStdDevDistanceMultiplier = 0.2;

        public static final Transform3d bratPose = new Transform3d(
                new Translation3d(-0.193, -0.288, 0.31), new Rotation3d(0, 0, Units.degreesToRadians(200)));

        public static final Transform3d blatPose = new Transform3d(
                new Translation3d(-0.208, 0.13, 0.33), new Rotation3d(0, 0, Units.degreesToRadians(210)));

        // How many robot pose measurements to store per camera
        public static final int maxMeasurements = 8;
    }

    // Find out the robot heading from the gyro (real or simulated)
    private GyroIO gyro;

    // The four swerve modules, in order: front-left, front-right, back-left, back-right
    private SwerveModule[] modules = new SwerveModule[4];

    // Current robot heading (radians) used by the pose estimator
    private Rotation2d gyroAngle = new Rotation2d();

    // Math helper that converts chassis speeds (vx, vy, omega) into wheel angles and speeds
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    // Make the best guess of the robot's field position using wheel odometry, gyro, and vision
    private SwerveDrivePoseEstimator estimator;

    private List<CameraIO> cameras = new ArrayList<>();

    // On-screen drawing of the drive to show module directions and speeds
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // Holonomic controller for auto-align
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;

    // Target pose for auto-align
    private FieldPose2d targetPose = new FieldPose2d();

    // Target dx, dy, dtheta for manual control
    private double dx, dy, dtheta;

    // Whether manual position control is field-oriented
    private boolean fieldOriented = true;

    // Whether PID is controlling position or manual is
    private boolean pidPosition;

    // Whether PID is controlling rotation or manual is
    private boolean pidRotation;

    // Whether the bot is currently in the X position
    private boolean locked;

    // For Elastic visualization
    private Field2d field = new Field2d();

    // Alert for no vision measurements
    private Alert noVision = new Alert("No vision measurements have been recorded yet", AlertType.kWarning);

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    public Swerve(GyroIO gyro, SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        this.gyro = gyro;
        this.modules = new SwerveModule[] {fl, fr, bl, br};

        Pose2d initialPose = new Pose2d(Field.fieldLength / 2, Field.fieldWidth / 2, Rotation2d.kZero);

        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyroAngle,
                getModulePositions(),
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(1, 1, 1));

        // Initialize PID controllers for auto align
        xController = new PIDController(
                Constants.translationkP.get(), Constants.translationkI.get(), Constants.translationkD.get());
        yController = new PIDController(
                Constants.translationkP.get(), Constants.translationkI.get(), Constants.translationkD.get());
        thetaController = new ProfiledPIDController(
                Constants.rotationkP.get(),
                Constants.rotationkI.get(),
                Constants.rotationkD.get(),
                new Constraints(10, 10));

        // Set wraparound on theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Send field visual to SmartDashboard
        SmartDashboard.putData("Field", field);

        // Reset the gyro
        resetGyro();

        // Enable vision alert
        noVision.set(true);
    }

    // Find out where the modules are mounted on the robot relative to the center (meters)
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    // Find out each module's odometry: wheel angle (radians) and total distance driven (meters)
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    // Find out each wheel's change in position
    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPositionDelta();
        }
        return states;
    }

    // Find out each module's current state: wheel angle (radians) and speed (m/s)
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // Find out each module's current target state
    public SwerveModuleState[] getModuleTargets() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getTargetState();
        }
        return states;
    }

    // Find out the robot's current field position (x, y in meters, rotation in radians)
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    // Find out the robot's current heading
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    // Tell the pose estimator to reset to a known field position (meters, radians)
    public void setPose(Pose2d newPose) {
        estimator.resetPosition(gyroAngle, getModulePositions(), newPose);
    }

    // Set the gyro to rotation 0
    public void resetGyro() {
        gyro.setYaw(0);
        gyroAngle = Rotation2d.kZero;
    }

    // Tell the robot how fast to move:
    // - dx, dy are strafe commands (m/s)
    // - omega is rotation speed (rad/s)
    // - fieldRelative = true means the commands are relative to the field (forward = away from our driver station)
    //   and will auto-flip for alliance side using RobotUtils
    private void setSpeeds(double dx, double dy, double omega, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(dx, dy, omega);
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
        }
        setChassisSpeeds(speeds);
    }

    // Tell the modules to reach a target chassis speed: vx (m/s), vy (m/s), omega (rad/s)
    private void setChassisSpeeds(ChassisSpeeds speeds) {
        lastChassisSpeeds = speeds;
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, frc.robot.Constants.loopTime);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.maxLinearSpeedMetersPerSec);
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    // Make the modules point in an X pattern so it's harder to push the robot.
    public void lock() {
        locked = true;
    }

    // Set manual control for position
    public void setPositionOutput(double dx, double dy) {
        locked = false;
        pidPosition = false;
        this.dx = dx;
        this.dy = dy;
    }

    public boolean getPositionPIDSetting() {
        return pidPosition;
    }

    public boolean getRotationPIDSetting() {
        return pidRotation;
    }

    // Set manual control for rotation
    public void setRotationOutput(double dtheta) {
        locked = false;
        pidRotation = false;
        this.dtheta = dtheta;
    }

    // Set PID control for position
    public void setPositionTarget(double x, double y) {
        locked = false;
        pidPosition = true;
        Pose2d lastPose = targetPose.getOnBlue();
        targetPose = new FieldPose2d(new Pose2d(x, y, lastPose.getRotation()));
    }

    // Set PID control for rotation
    public void setRotationTarget(double theta) {
        locked = false;
        pidRotation = true;
        Pose2d lastPose = targetPose.getOnBlue();
        targetPose = new FieldPose2d(new Pose2d(lastPose.getTranslation(), Rotation2d.fromRadians(theta)));
    }

    // Set the target pose (just position and rotation in one method)
    public void setPoseTarget(FieldPose2d pose) {
        locked = false;
        pidPosition = true;
        pidRotation = true;
        targetPose = pose;
    }

    // Sets whether manual position control should be field-oriented
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    // Add a vision measurement with the given pose, timestamp, and standard deviations
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        estimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

        // Disable vision alert
        noVision.set(false);
    }

    // Calculate vision standard deviations based on distance, ambiguity, and tag count
    private Matrix<N3, N1> calculateVisionStdDevs(Pose2d visionPose, double ambiguity, int tagCount) {
        // Distance from current pose to vision measurement
        double distance = getPose().getTranslation().getDistance(visionPose.getTranslation());

        // Base standard deviations (increases with distance)
        double xyStdDev = VisionConstants.visionXYStdDevBase
                + (distance * distance * VisionConstants.visionXYStdDevDistanceMultiplier);
        double thetaStdDev = VisionConstants.visionThetaStdDevBase
                + (distance * VisionConstants.visionThetaStdDevDistanceMultiplier);

        // Scale by ambiguity (higher ambiguity = less trust)
        xyStdDev *= (1 + ambiguity);
        thetaStdDev *= (1 + ambiguity);

        // Multi-tag measurements are more reliable
        if (tagCount > 1) {
            xyStdDev /= Math.sqrt(tagCount);
            thetaStdDev /= Math.sqrt(tagCount);
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    // Adds a new CameraIO as a source of data
    public void addCameraSource(CameraIO camera) {
        cameras.add(camera);
    }

    // Gets all CameraIO's currently being used as a source
    public List<CameraIO> getCameras() {
        return cameras;
    }

    // Gets translation error to the goal in meters
    public double getTranslationError() {
        return getPose().getTranslation().getDistance(targetPose.get().getTranslation());
    }

    // Gets rotation error to the goal in radians
    public double getRotationError() {
        return Math.abs(getPose().getRotation().getRadians()
                - targetPose.get().getRotation().getRadians());
    }

    @Override
    public void periodic() {
        // This runs every robot loop (~50 times per second)
        // Run each module's periodic to update sensors and control, and brake and disable if necessary
        for (SwerveModule module : modules) {
            module.setLocked(Constants.swerveLocked.get());
            module.setDisabled(Constants.swerveDisabled.get());
            module.periodic();
        }

        // Update PID controllers
        xController.setP(Constants.translationkP.get());
        yController.setP(Constants.translationkP.get());
        thetaController.setP(Constants.rotationkP.get());

        xController.setD(Constants.translationkD.get());
        yController.setD(Constants.translationkD.get());
        thetaController.setD(Constants.rotationkD.get());

        xController.setI(Constants.translationkI.get());
        yController.setI(Constants.translationkI.get());
        thetaController.setI(Constants.rotationkI.get());

        for (SwerveModule module : modules) {
            // Update drive motor PID
            module.setDriveKP(Constants.driveKP.get());
            module.setDriveKI(Constants.driveKI.get());
            module.setDriveKD(Constants.driveKD.get());
            module.setDriveKS(Constants.driveKS.get());
            module.setDriveKV(Constants.driveKV.get());
            module.setDriveKA(Constants.driveKA.get());

            // Update steer motor PID
            module.setAngleKP(Constants.steerKP.get());
            module.setAngleKI(Constants.steerKI.get());
            module.setAngleKD(Constants.steerKD.get());
            module.setAngleKS(Constants.steerKS.get());
            module.setAngleKV(Constants.steerKV.get());
            module.setAngleKA(Constants.steerKA.get());
        }

        double xSpeed = 0, ySpeed = 0;

        // Set swerve module targets depending on current settings
        if (locked) {
            Rotation2d[] headings = new Rotation2d[4];
            for (int i = 0; i < 4; i++) {
                headings[i] = getModuleTranslations()[i].getAngle();
            }
            kinematics.resetHeadings(headings);
        } else {

            boolean positionFieldOriented = true;
            if (pidPosition) {
                xSpeed =
                        xController.calculate(getPose().getX(), targetPose.get().getX());
                ySpeed =
                        yController.calculate(getPose().getY(), targetPose.get().getY());
            } else {
                xSpeed = dx;
                ySpeed = dy;
                if (!fieldOriented) {
                    positionFieldOriented = false;
                } else if (RobotUtils.onRedAlliance()) {
                    xSpeed *= -1;
                    ySpeed *= -1;
                }
            }
            double thetaSpeed;
            if (pidRotation) {
                thetaSpeed = thetaController.calculate(
                        getPose().getRotation().getRadians(),
                        targetPose.get().getRotation().getRadians());
            } else {
                thetaSpeed = dtheta;
            }
            setSpeeds(xSpeed, ySpeed, thetaSpeed, positionFieldOriented);
        }

        Logger.recordOutput("Swerve/Locked", locked);
        Logger.recordOutput("Swerve/dx", xSpeed);
        Logger.recordOutput("Swerve/dy", ySpeed);
        Logger.recordOutput("Swerve/dtheta", dtheta);
        Logger.recordOutput("Swerve/TargetPose", targetPose.get());
        Logger.recordOutput("Swerve/PoseTranslationError", getTranslationError());
        Logger.recordOutput("Swerve/PoseRotationError", getRotationError());
        Logger.recordOutput("Swerve/PIDPosition", pidPosition);
        Logger.recordOutput("Swerve/PIDRotation", pidRotation);

        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        Logger.recordOutput("Swerve/States", states);
        Logger.recordOutput("Swerve/ChassisSpeeds", lastChassisSpeeds);
        Logger.recordOutput("Swerve/EstimatedPose", getPose());
        Logger.recordOutput("Swerve/Rotation", getRotation());

        // Get measurements from all connected cameras and add them to the pose estimator
        for (CameraIO cam : cameras) {
            cam.update();
            CameraIOInputs inputs = cam.getInputs();
            for (int i = 0; i < inputs.measurements; i++) {
                Matrix<N3, N1> stdDevs =
                        calculateVisionStdDevs(inputs.poses[i].toPose2d(), inputs.ambiguities[i], inputs.tagCounts[i]);
                addVisionMeasurement(inputs.poses[i].toPose2d(), inputs.poseTimestamps[i], stdDevs);
            }
        }

        // Update the gyro inputs (logging and alerts happen automatically)
        gyro.update();

        if (gyro.getInputs().connected) {
            // If gyro is connected, read the angle
            gyroAngle = Rotation2d.fromRadians(gyro.getInputs().yawPositionRad);
        } else {
            // If gyro is disconnected, like in sim, get module deltas and use odometry to figure out the change in
            // angle
            Twist2d twist = kinematics.toTwist2d(getModuleDeltas());
            gyroAngle = gyroAngle.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        // Feed odometry to the pose estimator (time, heading, and wheel distances)
        estimator.updateWithTime(RobotController.getFPGATime() / 1000000., gyroAngle, getModulePositions());

        Logger.recordOutput("Swerve/Visualization", mech);

        // Update field pose
        field.setRobotPose(getPose());
    }
}
