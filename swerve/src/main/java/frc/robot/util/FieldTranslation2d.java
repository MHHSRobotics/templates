package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Alliance-agnostic 2D position on the field.
 *
 * This class lets you store field positions in "blue alliance coordinates"
 * and automatically converts them to the correct coordinates based on which
 * alliance you're actually on. This way you don't have to worry about
 * flipping coordinates in your code - just store positions as if you're
 * always on blue alliance and this class handles the rest!
 */
public class FieldTranslation2d {
    private double x, y;

    // Create a field position using blue alliance coordinates (meters)
    public FieldTranslation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Create a field position at the origin (0,0)
    public FieldTranslation2d() {
        this(0, 0);
    }

    // Get the position for whatever alliance we're currently on (automatically flips for red alliance)
    public Translation2d get() {
        return RobotUtils.invertToAlliance(getOnBlue());
    }

    // Get the position as if we're on blue alliance (original coordinates)
    public Translation2d getOnBlue() {
        return new Translation2d(x, y);
    }

    // Get the position as if we're on red alliance (flipped coordinates)
    public Translation2d getOnRed() {
        return RobotUtils.invert(getOnBlue());
    }
}
