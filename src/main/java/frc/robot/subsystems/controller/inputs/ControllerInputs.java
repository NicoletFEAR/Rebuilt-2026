package frc.robot.subsystems.controller.inputs;

import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.controller.ControllerState;

public abstract class ControllerInputs extends Inputs<ControllerState> {
    public abstract void leftRumble(double strength);
    public abstract void rightRumble(double strength);
    public abstract void rumble(double strength);
}
