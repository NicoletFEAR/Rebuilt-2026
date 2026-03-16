package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.universal.UniversalChoreographer;
import frc.robot.subsystems.controller.inputs.ControllerInputs;

public class ControllerChoreographer extends UniversalChoreographer<ControllerState, ControllerInputs, ControllerName, ControllerIdentity> {
    public ControllerChoreographer(ControllerName name, ControllerInputs inputs) {
        super(name, inputs);
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
                () -> m_inputs.rumble(requestedState.LeftRumbleStrength)
            );
        } else {
            SequentialCommandGroup result = new SequentialCommandGroup();

            if (!MathUtil.isNear(
                currentState.LeftRumbleStrength,
                requestedState.LeftRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                result.addCommands(Commands.runOnce(
                    () -> m_inputs.leftRumble(requestedState.LeftRumbleStrength)
                ));
            }

            if (!MathUtil.isNear(
                currentState.RightRumbleStrength,
                requestedState.RightRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                result.addCommands(Commands.runOnce(
                    () -> m_inputs.rightRumble(requestedState.RightRumbleStrength)
                ));
            }

            return result;
        }
    }
}
