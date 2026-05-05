package frc.robot.robots.tusk;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.DriveConstants.TuskDriveConstants;
import frc.robot.io.controller.ControllerName;
import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.turn.Turn;
import frc.robot.subsystems.turn.TurnName;

public class Tusk extends RobotContainer {
    private final TuskDriveConstants m_driveConstants = new TuskDriveConstants();

    private final Controller m_driverController;
    private final Turn m_frontLeftTurn;
    private final Turn m_frontRightTurn;
    private final Turn m_rearLeftTurn;
    private final Turn m_rearRightTurn;
    private final Controller m_operatorController;

    private final TuskStateAutoLogged m_state = new TuskStateAutoLogged();
    private final TuskRequestAutoLogged m_request = new TuskRequestAutoLogged();

    public Tusk() {
        m_driverController = new Controller(ControllerName.DRIVER);
        m_frontLeftTurn = new Turn(TurnName.FRONT_LEFT, m_driveConstants);
        m_frontRightTurn = new Turn(TurnName.FRONT_RIGHT, m_driveConstants);
        m_operatorController = new Controller(ControllerName.OPERATOR);
        m_rearLeftTurn = new Turn(TurnName.REAR_LEFT, m_driveConstants);
        m_rearRightTurn = new Turn(TurnName.REAR_RIGHT, m_driveConstants);
    }

    @Override
    public void periodic() {
        m_state.updateDriverController(m_driverController.update());
        m_state.updateFrontLeftTurn(m_frontLeftTurn.update());
        m_state.updateFrontRightTurn(m_frontRightTurn.update());
        m_state.updateOperatorController(m_operatorController.update());
        m_state.updateRearLeftTurn(m_rearLeftTurn.update());
        m_state.updateRearRightTurn(m_rearRightTurn.update());

        m_driverController.updateMissingIO();
        m_frontLeftTurn.updateMissingIO();
        m_frontRightTurn.updateMissingIO();
        m_operatorController.updateMissingIO();
        m_rearLeftTurn.updateMissingIO();
        m_rearRightTurn.updateMissingIO();

        m_request.updateDriverController(m_driverController.requestTusk(m_state));
        m_request.updateFrontLeftTurn(m_frontLeftTurn.requestTusk(m_state));
        m_request.updateFrontRightTurn(m_frontRightTurn.requestTusk(m_state));
        m_request.updateOperatorController(m_operatorController.requestTusk(m_state));
        m_request.updateRearLeftTurn(m_rearLeftTurn.requestTusk(m_state));
        m_request.updateRearRightTurn(m_rearRightTurn.requestTusk(m_state));

        Logger.processInputs("Tusk/State", m_state);
        Logger.processInputs("Tusk/Request", m_request);

        m_driverController.runTusk(m_state);
        m_frontLeftTurn.runTusk(m_state);
        m_frontRightTurn.runTusk(m_state);
        m_operatorController.runTusk(m_state);
        m_rearLeftTurn.runTusk(m_state);
        m_rearRightTurn.runTusk(m_state);
    }
}
