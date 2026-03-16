package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;
import frc.robot.subsystems.drive.DriveState;
import frc.robot.subsystems.drive.DriveStateAutoLogged;

@AutoLog
public class TuskState extends RobotState<TuskState> {
    public DriveStateAutoLogged DriveState = new DriveStateAutoLogged();
    public ControllerStateAutoLogged DriverControllerState = new ControllerStateAutoLogged();
    public ControllerStateAutoLogged OperatorControllerState = new ControllerStateAutoLogged();

    @Override
    public TuskState update(TuskState newState) {
        DriveState.update(newState.DriveState);
        DriverControllerState.update(newState.DriverControllerState);
        OperatorControllerState.update(newState.OperatorControllerState);
        return this;
    }

    public TuskState updateDriveState(DriveState newState) {
        DriveState.update(newState);
        return this;
    }

    public TuskState updateDriverControllerState(ControllerState newState) {
        DriverControllerState.update(newState);
        return this;
    }

    public TuskState updateOperatorControllerState(ControllerState newState) {
        OperatorControllerState.update(newState);
        return this;
    }

    public TuskStateAutoLogged toAutoLogged() {
        TuskStateAutoLogged result = new TuskStateAutoLogged();
        result.update(this);
        return result;
    }
}