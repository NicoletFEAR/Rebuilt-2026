package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.DriveState;
import frc.robot.subsystems.drive.DriveStateAutoLogged;

@AutoLog
public class TuskState {
    protected DriveStateAutoLogged DriveState = new DriveStateAutoLogged();

    public TuskState() {}

    public TuskState update(TuskState newState) {
        DriveState.update(newState.getDriveState());
        return this;
    }

    public DriveState getDriveState() {
        return DriveState;
    }

    public TuskState updateDriveState(DriveState newState) {
        DriveState.update(newState);
        return this;
    }

    public TuskStateAutoLogged toAutoLogged() {
        TuskStateAutoLogged result = new TuskStateAutoLogged();
        result.update(this);
        return result;
    }
}