package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public abstract class DriveConstants {
    public Angle kFrontLeftOffset;
    public Angle kFrontRightOffset;
    public Angle kRearLeftOffset;
    public Angle kRearRightOffset;

    public double kTurnGearRatio;

    public class TuskDriveConstants extends DriveConstants {
        public TuskDriveConstants() {
            kFrontLeftOffset = Rotations.of(-0.893798828125);
            kFrontRightOffset = Rotations.of(-0.101806640625);
            kRearLeftOffset = Rotations.of(-0.6767578125);
            kRearRightOffset = Rotations.of(-0.470947265625);

            kTurnGearRatio = 150.0 / 7.0;
        }
    }

    public class HadesDriveConstants extends DriveConstants {
        public HadesDriveConstants() {
            kFrontLeftOffset = Rotations.of(-0.403076171875);
            kFrontRightOffset = Rotations.of(-0.0966796875);
            kRearLeftOffset = Rotations.of(-0.109375);
            kRearRightOffset = Rotations.of(-0.310546875);

            kTurnGearRatio = 150.0 / 7.0;
        }
    }

    public class KitbotDriveConstants extends DriveConstants {
        public KitbotDriveConstants() {
            kFrontLeftOffset = Rotations.of(-0.65380859375);
            kFrontRightOffset = Rotations.of(-0.073974609375);
            kRearLeftOffset = Rotations.of(-0.82666015625);
            kRearRightOffset = Rotations.of(-0.55419921875);

            kTurnGearRatio = 150.0 / 7.0;
        }
    }
}
