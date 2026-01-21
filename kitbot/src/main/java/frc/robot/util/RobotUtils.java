package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

// Utility functions used it multiple places
public class RobotUtils {
    /**
     * Checks if the robot is on the red alliance.
     *
     * @return True if the robot is on the red alliance, false otherwise.
     */
    public static boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // Mirror a Pose2d across the field depending on field symmetry (C2 = rotate 180°, D2 = reflect across center line)
    public static Pose2d invert(Pose2d pose) {
        switch (Field.symm) {
            case C2:
                // C2 symmetry: rotate 180° around field center (like spinning the field upside down)
                return new Pose2d(
                        Field.fieldLength - pose.getX(),
                        Field.fieldWidth - pose.getY(),
                        pose.getRotation().rotateBy(Rotation2d.k180deg));
            default: // D2 symmetry
                // D2 symmetry: reflect across the field centerline (like flipping the field left-to-right)
                return new Pose2d(
                        Field.fieldLength - pose.getX(),
                        pose.getY(), // Y stays the same for reflection symmetry
                        Rotation2d.fromRadians(Math.PI - pose.getRotation().getRadians())); // Reflect the angle
        }
    }

    // Mirror a Pose2d if we're on the red alliance (blue alliance uses coordinates as-is)
    public static Pose2d invertToAlliance(Pose2d pose) {
        if (onRedAlliance()) {
            return invert(pose);
        } else {
            return pose;
        }
    }

    // Mirror a Pose3d across the field depending on field symmetry (C2 = rotate 180°, D2 = reflect across center line)
    public static Pose3d invert(Pose3d pose) {
        switch (Field.symm) {
            case C2:
                // C2 symmetry: rotate 180° around field center (like spinning the field upside down)
                return new Pose3d(
                        Field.fieldLength - pose.getX(),
                        Field.fieldWidth - pose.getY(),
                        pose.getZ(),
                        pose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI))); // Rotate 180° around Z-axis only
            default: // D2 symmetry
                // D2 symmetry: reflect across the field centerline (like flipping the field left-to-right)
                // Keep pitch and roll the same, only reflect the yaw (Z rotation)
                return new Pose3d(
                        Field.fieldLength - pose.getX(),
                        pose.getY(), // Y stays the same for reflection symmetry
                        pose.getZ(),
                        new Rotation3d(
                                pose.getRotation().getX(), // Keep roll (X rotation) the same
                                pose.getRotation().getY(), // Keep pitch (Y rotation) the same
                                Math.PI - pose.getRotation().getZ())); // Reflect only the yaw (Z rotation)
        }
    }

    // Mirror a Pose3d if we're on the red alliance (blue alliance uses coordinates as-is)
    public static Pose3d invertToAlliance(Pose3d pose) {
        if (onRedAlliance()) {
            return invert(pose);
        } else {
            return pose;
        }
    }

    // Mirror a Translation2d across the field depending on field symmetry (C2 = rotate 180°, D2 = reflect across center
    // line)
    public static Translation2d invert(Translation2d trans) {
        switch (Field.symm) {
            case C2:
                // C2 symmetry: rotate 180° around field center (both X and Y flip)
                return new Translation2d(Field.fieldLength - trans.getX(), Field.fieldWidth - trans.getY());
            default: // D2 symmetry
                // D2 symmetry: reflect across the field centerline (only X flips, Y stays same)
                return new Translation2d(Field.fieldLength - trans.getX(), trans.getY());
        }
    }

    // Mirror a Translation2d if we're on the red alliance (blue alliance uses coordinates as-is)
    public static Translation2d invertToAlliance(Translation2d trans) {
        if (onRedAlliance()) {
            return invert(trans);
        } else {
            return trans;
        }
    }

    // Mirror a Translation3d across the field depending on field symmetry (C2 = rotate 180°, D2 = reflect across center
    // line)
    public static Translation3d invert(Translation3d trans) {
        switch (Field.symm) {
            case C2:
                // C2 symmetry: rotate 180° around field center (both X and Y flip, Z stays same)
                return new Translation3d(
                        Field.fieldLength - trans.getX(), Field.fieldWidth - trans.getY(), trans.getZ());
            default: // D2 symmetry
                // D2 symmetry: reflect across the field centerline (only X flips, Y and Z stay same)
                return new Translation3d(Field.fieldLength - trans.getX(), trans.getY(), trans.getZ());
        }
    }

    // Mirror a Translation3d if we're on the red alliance (blue alliance uses coordinates as-is)
    public static Translation3d invertToAlliance(Translation3d trans) {
        if (onRedAlliance()) {
            return invert(trans);
        } else {
            return trans;
        }
    }

    // Mirror a Rotation2d across the field depending on field symmetry (C2 = rotate 180°, D2 = reflect)
    public static Rotation2d invert(Rotation2d rot) {
        switch (Field.symm) {
            case C2:
                // C2 symmetry: rotate 180° (like spinning the robot around)
                return rot.rotateBy(Rotation2d.k180deg);
            default: // D2 symmetry
                // D2 symmetry: reflect the angle (like looking in a mirror)
                return Rotation2d.fromRadians(Math.PI - rot.getRadians());
        }
    }

    // Mirror a Rotation2d if we're on the red alliance (blue alliance uses angles as-is)
    public static Rotation2d invertToAlliance(Rotation2d rot) {
        if (onRedAlliance()) {
            return invert(rot);
        } else {
            return rot;
        }
    }
}
