package frc.robot.robots.hades;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;
import frc.robot.subsystems.drive.DriveState;
import frc.robot.subsystems.drive.DriveStateAutoLogged;

@AutoLog
public class HadesState extends RobotState<HadesState> {
    public ControllerStateAutoLogged ControllerState = new ControllerStateAutoLogged();
    public DriveStateAutoLogged DriveState = new DriveStateAutoLogged();

    @Override
    public HadesState update(HadesState newState) {
        ControllerState.update(newState.ControllerState);
        DriveState.update(newState.DriveState);
        return this;
    }

    public HadesState updateControllerState(ControllerState newState) {
        ControllerState.update(newState);
        return this;
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
