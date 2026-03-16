package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.universal.UniversalChoreographer;

public class ControllerChoreographer extends UniversalChoreographer<ControllerState> {
    private PS5Controller m_controller;
    private ControllerName m_name;

    public ControllerChoreographer(int port, ControllerName name) {
        m_controller = new PS5Controller(port);
        m_name = name;
    }

    @Override
    public Command choreographHades(HadesState fullState, ControllerState requestedState) {
        return choreograph(fullState.ControllerState, requestedState);
    }

    @Override
    public Command choreographKitbot(KitbotState fullState, ControllerState requestedState) {
        return choreograph(fullState.ControllerState, requestedState);
    }

    @Override
    public Command choreographTusk(TuskState fullState, ControllerState requestedState) {
        return choreograph(switch (m_name) {
            case DRIVER -> fullState.DriverControllerState;
            case OPERATOR -> fullState.OperatorControllerState;
        }, requestedState);
    }

    public Command choreograph(ControllerState currentState, ControllerState requestedState) {
        if (requestedState.LeftRumbleStrength == requestedState.RightRumbleStrength 
            && !MathUtil.isNear(
                currentState.LeftRumbleStrength,
                requestedState.LeftRumbleStrength,
                Constants.kGeneralTolerance
            ) && !MathUtil.isNear(
                currentState.RightRumbleStrength,
                requestedState.RightRumbleStrength,
                Constants.kGeneralTolerance
        )) {
            return Commands.runOnce(
                () -> m_controller.setRumble(RumbleType.kBothRumble, requestedState.LeftRumbleStrength)
            );
        } else {
            SequentialCommandGroup result = new SequentialCommandGroup();

            if (!MathUtil.isNear(
                currentState.LeftRumbleStrength,
                requestedState.LeftRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                result.addCommands(Commands.runOnce(
                    () -> m_controller.setRumble(RumbleType.kLeftRumble, requestedState.LeftRumbleStrength)
                ));
            }

            if (!MathUtil.isNear(
                currentState.RightRumbleStrength,
                requestedState.RightRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                result.addCommands(Commands.runOnce(
                    () -> m_controller.setRumble(RumbleType.kRightRumble, requestedState.RightRumbleStrength)
                ));
            }

            return result;
        }
    }
}
