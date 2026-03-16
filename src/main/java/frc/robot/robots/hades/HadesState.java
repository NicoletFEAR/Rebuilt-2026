package frc.robot.robots.hades;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.drive.DriveState;
import frc.robot.subsystems.drive.DriveStateAutoLogged;

@AutoLog
public class HadesState extends RobotState<HadesState> {
    protected DriveStateAutoLogged DriveState = new DriveStateAutoLogged();

    public HadesState() {}

    @Override
    public HadesState update(HadesState newState) {
        DriveState.update(newState.getDriveState());
        return this;
    }

    public DriveState getDriveState() {
        return DriveState;
    }

    public HadesState updateDriveState(DriveState newState) {
        DriveState.update(newState);
        return this;
    }

    public HadesStateAutoLogged toAutoLogged() {
        HadesStateAutoLogged result = new HadesStateAutoLogged();
        result.update(this);
        return result;
    }
}
