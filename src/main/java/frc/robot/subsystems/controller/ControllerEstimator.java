package frc.robot.subsystems.controller;

import frc.robot.io.joystick.Joystick;
import frc.robot.io.joystick.JoystickIdentity;
import frc.robot.io.joystick.JoystickState;
import frc.robot.subsystems.base.Estimator;
import frc.robot.util.Container;

public class ControllerEstimator extends Estimator<JoystickState, ControllerState, Joystick, JoystickIdentity> {
    private final JoystickState state;

    public ControllerEstimator(Container<Joystick> io) {
        super(io);
        state = new JoystickState();
        estimate = new ControllerState();
    }

    public ControllerState estimate() {
        state.update(io.get().updateState());
        estimate.Circle = state.Circle;
        estimate.Create = state.Create;
        estimate.Cross = state.Cross;

        return estimate;
    }
}
