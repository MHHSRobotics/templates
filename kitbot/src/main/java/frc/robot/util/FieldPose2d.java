package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Alliance-agnostic 2D pose (position + rotation) on the field.
 *
 * This class lets you store robot poses in "blue alliance coordinates"
 * and automatically converts them to the correct coordinates based on which
 * alliance you're actually on. This way you don't have to worry about
 * flipping coordinates and rotations in your code - just store poses as if
 * you're always on blue alliance and this class handles the rest!
 */
public class FieldPose2d {
    private double x, y;
    private double rotationRadians;

    // Create a field pose using blue alliance coordinates (x and y in meters, rotation in radians)
    public FieldPose2d(double x, double y, double rotationRadians) {
        this.x = x;
        this.y = y;
        this.rotationRadians = rotationRadians;
    }

    // Create a field pose using blue alliance coordinates (x and y in meters, rotation as Rotation2d)
    public FieldPose2d(double x, double y, Rotation2d rotation) {
        this(x, y, rotation.getRadians());
    }

    // Create a field pose from a regular Pose2d (assumes it's in blue alliance coordinates)
    public FieldPose2d(Pose2d pose) {
        this(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    // Create a field pose at the origin (0,0) facing forward (0 radians)
    public FieldPose2d() {
        this(0, 0, 0);
    }

    // Get the pose for whatever alliance we're currently on (automatically flips for red alliance)
    public Pose2d get() {
        return RobotUtils.invertToAlliance(getOnBlue());
    }

    // Get the pose as if we're on blue alliance (original coordinates)
    public Pose2d getOnBlue() {
        return new Pose2d(x, y, new Rotation2d(rotationRadians));
    }

    // Get the pose as if we're on red alliance (flipped coordinates and rotation)
    public Pose2d getOnRed() {
        return RobotUtils.invert(getOnBlue());
    }
}
