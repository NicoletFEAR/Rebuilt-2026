package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.base.State;

@AutoLog
public class DriveState extends State<DriveState> {
    public DriveState update(DriveState newState) {
        return this;
    }

    public DriveStateAutoLogged toAutoLogged() {
        DriveStateAutoLogged result = new DriveStateAutoLogged();
        result.update(this);
        return result;
    }
}
