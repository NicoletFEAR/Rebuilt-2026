// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class Utils {
    public static void copyModuleStates(SwerveModuleState[] copier, SwerveModuleState[] reciever) {
        for (int i = 0; i < copier.length; i++) {
            reciever[i] = new SwerveModuleState(copier[i].speedMetersPerSecond, copier[i].angle);
        }
    }

    public static double getAdjustedYawDegrees(double initialvalue, double addedValue) {
        double numTo180 = 180 - addedValue;

        return (initialvalue + numTo180) % 360 < 0
            ? ((initialvalue + numTo180) % 360) + 360.0
            : ((initialvalue + numTo180) % 360);
    }

    public static double calculateYawError(double drivebaseYaw, double setpoint) {
        // Normalize yaw error to be within -180 to 180
        double error = drivebaseYaw - setpoint;
        error = ((error + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;
        return error;
    }

    public static double calculateShortestPath(double currentAngle, double targetAngle) {
        double error = targetAngle - currentAngle;
        return ((error + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]
    }

    public static double normalizeAngleError(double currentAngle, double targetAngle) {
        double error = targetAngle - currentAngle;
        return ((error + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]
    }

    /**
     *
     *
     * <h3>ModifyInputs</h3>
     *
     * Returns the input to the power of the modifier
     *
     * @param input    Input to modify
     * @param modifier Puts the input to the power of this
     */
    public static double modifyInputs(double input, double modifier) {
        return input >= 0 ? Math.pow(input, modifier) : -Math.pow(-input, modifier);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }

        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }

        return newAngle;
    }

    public static Translation2d getNudgedVector(Pose2d robotPose, Translation2d currentVector,
            Translation2d pointOfInterest, double kp) {
        if (currentVector.getX() == 0 && currentVector.getY() == 0) {
            return currentVector;
        }

        // Compute the vector from the robot to the point of interest
        Translation2d toPoint = pointOfInterest.minus(robotPose.getTranslation());

        if (toPoint.getNorm() < .25) {
            return currentVector;
        }

        // Project the toPoint vector onto the perpendicular of the velocity vector
        double perpDistance = (toPoint.getX() * -currentVector.getY() + toPoint.getY() * currentVector.getX())
                / currentVector.getNorm();

        // Compute the magnitude of the nudge
        double nudgeMagnitude = MathUtil.clamp(perpDistance * kp, -DriveConstants.getMaxModuleSpeed(),
                DriveConstants.getMaxModuleSpeed());

        // Compute the unit perpendicular vector to current velocity
        Translation2d perpDirection = new Translation2d(-currentVector.getY(), currentVector.getX())
                .div(currentVector.getNorm());

        // Compute the nudge vector
        Translation2d nudgeVector = perpDirection.times(nudgeMagnitude);

        // Return the modified velocity vector
        return currentVector.plus(nudgeVector);
    }

    public static double findAngleBetweenPoses(Pose2d pose1, Pose2d pose2) {
        Translation2d distanceVector = pose1.getTranslation().minus(pose2.getTranslation());

        if (distanceVector.getNorm() < 0) {
            return Double.POSITIVE_INFINITY;
        }

        if (pose1.getX() > pose2.getX()) {
            return new Rotation2d(Math.atan(distanceVector.getY() / distanceVector.getX()) + Math.PI).getDegrees();
        }

        return new Rotation2d(Math.atan(distanceVector.getY() / distanceVector.getX())).getDegrees();
    }
}
