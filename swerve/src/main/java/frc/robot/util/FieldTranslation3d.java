package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Alliance-agnostic 3D position on the field.
 *
 * This class lets you store field positions in "blue alliance coordinates"
 * and automatically converts them to the correct coordinates based on which
 * alliance you're actually on. This way you don't have to worry about
 * flipping coordinates in your code - just store positions as if you're
 * always on blue alliance and this class handles the rest!
 *
 * Useful for things like camera positions, arm targets, or any 3D objects
 * that need to work on both sides of the field.
 */
public class FieldTranslation3d {
    private double x, y, z;

    // Create a field position using blue alliance coordinates (x, y, z in meters)
    public FieldTranslation3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Create a field position from a regular Translation3d (assumes it's in blue alliance coordinates)
    public FieldTranslation3d(Translation3d translation) {
        this(translation.getX(), translation.getY(), translation.getZ());
    }

    // Create a field position at the origin (0,0,0)
    public FieldTranslation3d() {
        this(0, 0, 0);
    }

    // Get the position for whatever alliance we're currently on (automatically flips for red alliance)
    public Translation3d get() {
        return RobotUtils.invertToAlliance(getOnBlue());
    }

    // Get the position as if we're on blue alliance (original coordinates)
    public Translation3d getOnBlue() {
        return new Translation3d(x, y, z);
    }

    // Get the position as if we're on red alliance (flipped coordinates)
    public Translation3d getOnRed() {
        return RobotUtils.invert(getOnBlue());
    }
}
