package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class FieldDimensions {
    public static final Distance ROBOT_LENGTH = Inches.of(34.25);
    public static final Distance FIELD_LENGTH = Inches.of(651.2);
    public static final Distance FIELD_WIDTH = Inches.of(317.7);

    public enum Side {
        Outpost,
        Depot,
    }
}
