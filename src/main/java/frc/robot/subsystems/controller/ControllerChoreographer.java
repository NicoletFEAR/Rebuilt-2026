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
import frc.robot.subsystems.controller.io.ControllerIO;
import frc.robot.util.constraint.ConstraintType;

public class ControllerChoreographer extends UniversalChoreographer<ControllerState, ControllerRequest, ControllerIO, ControllerName, ControllerIdentity> {
    public ControllerChoreographer(ControllerName name, ControllerIO io) {
        super(name, io);
    }

    @Override
    public Command choreographHades(HadesState fullState, ControllerRequest requestedState) {
        return choreograph(fullState.Controller, requestedState);
    }

    @Override
    public Command choreographKitbot(KitbotState fullState, ControllerRequest requestedState) {
        return choreograph(fullState.Controller, requestedState);
    }

    @Override
    public Command choreographTusk(TuskState fullState, ControllerRequest requestedState) {
        return choreograph(switch (m_name) {
            case DRIVER -> fullState.DriverController;
            case OPERATOR -> fullState.OperatorController;
        }, requestedState);
    }

    public Command choreograph(ControllerState currentState, ControllerRequest requestedState) {
        ConstraintType leftRumbleStrengthType = requestedState.LeftRumbleStrength.getType();
        ConstraintType rightRumbleStrengthType = requestedState.RightRumbleStrength.getType();
        double leftRumbleStrength = requestedState.LeftRumbleStrength.get();
        double rightRumbleStrength = requestedState.RightRumbleStrength.get();

        if (leftRumbleStrengthType == ConstraintType.IGNORE
            && rightRumbleStrengthType == ConstraintType.IGNORE) {
            return Commands.none();
        } else if (leftRumbleStrengthType == ConstraintType.IGNORE) {
            if (!MathUtil.isNear(
                currentState.RightRumbleStrength,
                rightRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                return m_io.rightRumble(rightRumbleStrength);
            } else {
                return Commands.none();
            }
        } else if (rightRumbleStrengthType == ConstraintType.IGNORE) {
            if (!MathUtil.isNear(
                currentState.LeftRumbleStrength,
                leftRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                return m_io.leftRumble(leftRumbleStrength);
            } else {
                return Commands.none();
            }
        } else {
            if (leftRumbleStrength == rightRumbleStrength
                && !MathUtil.isNear(
                    currentState.LeftRumbleStrength,
                    leftRumbleStrength,
                    Constants.kGeneralTolerance
                ) && !MathUtil.isNear(
                    currentState.RightRumbleStrength,
                    rightRumbleStrength,
                    Constants.kGeneralTolerance
            )) {
                return m_io.rumble(leftRumbleStrength);
            } else {
                SequentialCommandGroup result = new SequentialCommandGroup();

                if (!MathUtil.isNear(
                    currentState.RightRumbleStrength,
                    rightRumbleStrength,
                    Constants.kGeneralTolerance
                )) {
                    result.addCommands(m_io.rightRumble(rightRumbleStrength));
                }

                if (!MathUtil.isNear(
                    currentState.LeftRumbleStrength,
                    leftRumbleStrength,
                    Constants.kGeneralTolerance
                )) {
                    result.addCommands(m_io.leftRumble(leftRumbleStrength));
                }

                return result;
            }
        }
    }
}
