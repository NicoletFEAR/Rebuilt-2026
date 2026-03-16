package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.drive.DriveState;
import frc.robot.subsystems.drive.DriveStateAutoLogged;

@AutoLog
public class KitbotState extends RobotState<KitbotState> {
    protected DriveStateAutoLogged DriveState = new DriveStateAutoLogged();

    public KitbotState() {}

    @Override
    public KitbotState update(KitbotState newState) {
        DriveState.update(newState.getDriveState());
        return this;
    }

    public DriveState getDriveState() {
        return DriveState;
    }

    public KitbotState updateDriveState(DriveState newState) {
        DriveState.update(newState);
        return this;
    }

    public KitbotStateAutoLogged toAutoLogged() {
        KitbotStateAutoLogged result = new KitbotStateAutoLogged();
        result.update(this);
        return result;
    }
}
