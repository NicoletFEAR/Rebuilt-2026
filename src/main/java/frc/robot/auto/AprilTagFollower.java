package frc.robot.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

public class AprilTagFollower extends LocalADStar {
    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        PathPlannerPath path = super.getCurrentPath(constraints, goalEndState);

        for (PathPoint point : path.getAllPathPoints()) {
            if (shouldFollowAprilTag(point.position)) {
                point.rotationTarget = new RotationTarget(point.waypointRelativePos, nearestAprilTag(point.position).minus(point.position).getAngle());
            }
        }

        return path;
    }

    public Translation2d nearestAprilTag(Translation2d position) {
        return DriveConstants.kBlueHubPosition;
    }

    public boolean shouldFollowAprilTag(Translation2d position) {
        return true;
    }
}
