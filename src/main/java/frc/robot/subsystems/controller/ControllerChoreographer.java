package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.io.controller.ControllerIO;
import frc.robot.io.controller.ControllerIdentity;
import frc.robot.io.controller.ControllerName;
import frc.robot.io.controller.ControllerState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Choreographer;
import frc.robot.util.Container;
import frc.robot.util.constraint.ConstraintType;

public class ControllerChoreographer extends Choreographer<ControllerState, ControllerRequest, ControllerIO, ControllerName, ControllerIdentity> {
    public ControllerChoreographer(ControllerName name, Container<ControllerIO> io) {
        super(name, io);
    }

    @Override
    public void choreographTusk(TuskState fullState, ControllerRequest requestedState) {
        ControllerState currentState = switch (m_name) {
            case DRIVER -> fullState.DriverController;
            case OPERATOR -> fullState.OperatorController;
        };

        ConstraintType leftRumbleStrengthType = requestedState.LeftRumbleStrength.getType();
        ConstraintType rightRumbleStrengthType = requestedState.RightRumbleStrength.getType();
        double leftRumbleStrength = requestedState.LeftRumbleStrength.get();
        double rightRumbleStrength = requestedState.RightRumbleStrength.get();

        if (leftRumbleStrengthType == ConstraintType.MATCH
            && rightRumbleStrengthType == ConstraintType.MATCH) {
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
                m_io.get().rumble(leftRumbleStrength);
            } else {
                if (!MathUtil.isNear(
                    currentState.RightRumbleStrength,
                    rightRumbleStrength,
                    Constants.kGeneralTolerance
                )) {
                    m_io.get().rightRumble(rightRumbleStrength);
                }

                if (!MathUtil.isNear(
                    currentState.LeftRumbleStrength,
                    leftRumbleStrength,
                    Constants.kGeneralTolerance
                )) {
                    m_io.get().leftRumble(leftRumbleStrength);
                }
            }
        } else if (leftRumbleStrengthType == ConstraintType.MATCH) {
            if (!MathUtil.isNear(
                currentState.LeftRumbleStrength,
                leftRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                m_io.get().leftRumble(leftRumbleStrength);
            }
        } else if (rightRumbleStrengthType == ConstraintType.MATCH) {
            if (!MathUtil.isNear(
                currentState.RightRumbleStrength,
                rightRumbleStrength,
                Constants.kGeneralTolerance
            )) {
                m_io.get().rightRumble(rightRumbleStrength);
            }
        }
    }
}
