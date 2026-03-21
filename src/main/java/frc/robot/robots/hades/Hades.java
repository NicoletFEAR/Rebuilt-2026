package frc.robot.robots.hades;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.DriveConstants.HadesDriveConstants;
import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;
import frc.robot.subsystems.turn.Turn;
import frc.robot.subsystems.turn.TurnName;

public class Hades extends RobotContainer {
    private HadesDriveConstants m_driveConstants = new HadesDriveConstants();

    private final Controller m_controller;
    private final Turn m_frontLeftTurn;
    private final Turn m_frontRightTurn;
    private final Turn m_rearLeftTurn;
    private final Turn m_rearRightTurn;

    private final HadesStateAutoLogged m_state = new HadesStateAutoLogged();
    private final HadesRequestAutoLogged m_request = new HadesRequestAutoLogged();

    public Hades() {
        m_controller = new Controller(ControllerName.DRIVER);
        m_frontLeftTurn = new Turn(TurnName.FRONT_LEFT, m_driveConstants);
        m_frontRightTurn = new Turn(TurnName.FRONT_RIGHT, m_driveConstants);
        m_rearLeftTurn = new Turn(TurnName.REAR_LEFT, m_driveConstants);
        m_rearRightTurn = new Turn(TurnName.REAR_RIGHT, m_driveConstants);
    }

    @Override
    public void periodic() {
        m_state.updateController(m_controller.update());
        m_state.updateFrontLeftTurn(m_frontLeftTurn.update());
        m_state.updateFrontRightTurn(m_frontRightTurn.update());
        m_state.updateRearLeftTurn(m_rearLeftTurn.update());
        m_state.updateRearRightTurn(m_rearRightTurn.update());

        m_controller.updateMissingIO();
        m_frontLeftTurn.updateMissingIO();
        m_frontRightTurn.updateMissingIO();
        m_rearLeftTurn.updateMissingIO();
        m_rearRightTurn.updateMissingIO();

        m_request.updateController(m_controller.requestHades(m_state));
        m_request.updateFrontLeftTurn(m_frontLeftTurn.requestHades(m_state));
        m_request.updateFrontRightTurn(m_frontRightTurn.requestHades(m_state));
        m_request.updateRearLeftTurn(m_rearLeftTurn.requestHades(m_state));
        m_request.updateRearRightTurn(m_rearRightTurn.requestHades(m_state));

        Logger.processInputs("Hades/State", m_state);
        Logger.processInputs("Hades/Request", m_request);

        m_controller.runHades(m_state);
        m_frontLeftTurn.runHades(m_state);
        m_frontRightTurn.runHades(m_state);
        m_rearLeftTurn.runHades(m_state);
        m_rearRightTurn.runHades(m_state);
    }
}
