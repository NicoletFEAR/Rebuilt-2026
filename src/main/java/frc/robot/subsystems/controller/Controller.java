package frc.robot.subsystems.controller;

import frc.robot.io.controller.ControllerIdentity;
import frc.robot.io.controller.ControllerIO;
import frc.robot.io.controller.ControllerIONone;
import frc.robot.io.controller.ControllerState;
import frc.robot.subsystems.base.Subsystem;
import frc.robot.util.Container;

public class Controller extends Subsystem<ControllerState, ControllerRequest, ControllerIO, ControllerName, ControllerIdentity> {
    public Controller(ControllerName name) {
        super(name);

        state = new ControllerState();
        request = new ControllerRequest();

        missingIO.setText(String.format("%s Controller disconnected! (port %d)", name.toString(), name.getPort()));
        io = new Container<ControllerIO>(new ControllerIONone(name.getPort()));
        requestor = new ControllerRequestor(name);
        choreographer = new ControllerChoreographer(name, io);

        createIOChangeTriggers();
    }

    @Override
    public void updateMissingIO() {
        missingIO.set(state.CurrentIdentity == ControllerIdentity.NONE);
    }
}
