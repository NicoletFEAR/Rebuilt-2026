package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.field.Zone;

public class AprilTagZone {
    Zone zone;
    Translation2d aprilTagPosition;

    public AprilTagZone(Zone zone, Translation2d aprilTagPosition) {
        this.zone = zone;
        this.aprilTagPosition = aprilTagPosition;
    }
}
