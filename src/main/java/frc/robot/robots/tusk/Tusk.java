package frc.robot.robots.tusk;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Tusk extends RobotContainer {
    private final Controller driverController;
    private final Controller operatorController;

    private final TuskStateAutoLogged state = new TuskStateAutoLogged();
    private final TuskRequestAutoLogged request = new TuskRequestAutoLogged();

    public Tusk() {
        driverController = new Controller(ControllerName.DRIVER);
        operatorController = new Controller(ControllerName.OPERATOR);
    }

    @Override
    public void periodic() {
        state.updateDriverController(driverController.update());
        state.updateOperatorController(operatorController.update());

        driverController.updateMissingIO();
        operatorController.updateMissingIO();

        request.updateDriverController(driverController.requestTusk(state));
        request.updateOperatorController(operatorController.requestTusk(state));

        Logger.processInputs("Tusk/State", state);
        Logger.processInputs("Tusk/Request", request);

        driverController.runTusk(state);
        operatorController.runTusk(state);
    }
}
