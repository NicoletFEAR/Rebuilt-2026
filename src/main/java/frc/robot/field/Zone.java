package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;

public class Zone {
    Translation2d topLeft;
    Translation2d bottomRight;

    public Zone(Translation2d topLeft, Translation2d bottomRight) {
        this.topLeft = topLeft;
        this.bottomRight = bottomRight;
    }

    public boolean contains(Translation2d position) {
        double x = position.getX();
        double y = position.getY();

        return topLeft.getX() > x && x > bottomRight.getX()
            && topLeft.getY() > y && y > bottomRight.getY();
    }
}
