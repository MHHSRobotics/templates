package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Alliance-agnostic 3D pose (position + rotation) on the field.
 *
 * This class lets you store robot poses in "blue alliance coordinates"
 * and automatically converts them to the correct coordinates based on which
 * alliance you're actually on. This way you don't have to worry about
 * flipping coordinates and rotations in your code - just store poses as if
 * you're always on blue alliance and this class handles the rest!
 *
 * Useful for things like camera positions, arm targets, or any 3D objects
 * that need to work on both sides of the field.
 */
public class FieldPose3d {
    private double x, y, z;
    private double rollRadians, pitchRadians, yawRadians;

    // Create a field pose using blue alliance coordinates (x, y, z in meters, rotations in radians)
    public FieldPose3d(double x, double y, double z, double rollRadians, double pitchRadians, double yawRadians) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.rollRadians = rollRadians;
        this.pitchRadians = pitchRadians;
        this.yawRadians = yawRadians;
    }

    // Create a field pose using blue alliance coordinates (x, y, z in meters, rotation as Rotation3d)
    public FieldPose3d(double x, double y, double z, Rotation3d rotation) {
        this(x, y, z, rotation.getX(), rotation.getY(), rotation.getZ());
    }

    // Create a field pose from a regular Pose3d (assumes it's in blue alliance coordinates)
    public FieldPose3d(Pose3d pose) {
        this(pose.getX(), pose.getY(), pose.getZ(), pose.getRotation());
    }

    // Create a field pose at the origin (0,0,0) with no rotation
    public FieldPose3d() {
        this(0, 0, 0, 0, 0, 0);
    }

    // Get the pose for whatever alliance we're currently on (automatically flips for red alliance)
    public Pose3d get() {
        return RobotUtils.invertToAlliance(getOnBlue());
    }

    // Get the pose as if we're on blue alliance (original coordinates)
    public Pose3d getOnBlue() {
        return new Pose3d(x, y, z, new Rotation3d(rollRadians, pitchRadians, yawRadians));
    }

    // Get the pose as if we're on red alliance (flipped coordinates and rotation)
    public Pose3d getOnRed() {
        return RobotUtils.invert(getOnBlue());
    }
}
