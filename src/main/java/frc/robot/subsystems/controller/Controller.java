package frc.robot.subsystems.controller;

import frc.robot.io.joystick.EmptyJoystick;
import frc.robot.io.joystick.Joystick;
import frc.robot.io.joystick.JoystickIdentity;
import frc.robot.io.joystick.JoystickState;
import frc.robot.subsystems.base.Subsystem;
import frc.robot.util.Container;

public class Controller extends Subsystem<JoystickState, ControllerState, ControllerRequest, Joystick, ControllerName, JoystickIdentity> {
    public Controller(ControllerName name) {
        super(name);

        state = new ControllerState();
        request = new ControllerRequest();

        missingIO.setText(String.format("%s Controller disconnected! (port %d)", name.toString(), name.getPort()));
        io = new Container<Joystick>(new EmptyJoystick(name.getPort()));
        estimator = new ControllerEstimator(io);
        requestor = new ControllerRequestor(name);
        choreographer = new ControllerChoreographer(name, io);

        createIOChangeTriggers();
    }

    @Override
    public void updateMissingIO() {
        missingIO.set(state.CurrentIdentity == JoystickIdentity.NONE);
    }
}
