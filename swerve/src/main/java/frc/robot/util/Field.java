package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Field {
    // Field dimensions
    public static final double fieldLength = 16.54;
    public static final double fieldWidth = 8.07;

    // Whether the field this season has rotational (C2) or reflected (D2) symmetry
    public enum FieldSymmetry {
        C2,
        D2
    }

    public static final FieldSymmetry symm = FieldSymmetry.C2;

    public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Get the april tag field layout for the current season
}
