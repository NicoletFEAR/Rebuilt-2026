// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.PositionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.OperatorConstants;

public class Climb extends SubsystemBase implements PositionSubsystem {
    private double m_desiredPosition;
    private TalonFX m_motor;

    public Climb() {
        m_motor = new TalonFX(MotorIds.kClimb, Constants.hasCANivore() ? "*" : "rio");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ClimbConstants.kP;
        config.Slot0.kI = ClimbConstants.kI;
        config.Slot0.kD = ClimbConstants.kD;
        m_motor.getConfigurator().apply(config);

        m_desiredPosition = ClimbConstants.kHomePosition;
        m_motor.setPosition(m_desiredPosition);
    }

    @Override
    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < ClimbConstants.kSetpointTolerance;
    }

    @Override
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climb/Desired Position", m_desiredPosition);
        Logger.recordOutput("Climb/Position", getPosition());
        Logger.recordOutput("Climb/Current", m_motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Climb/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Climb/Velocity", m_motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Climb/Is At Setpoint", getIsAtSetpoint());
    }

    @Override
    public void runToPosition(double position) {
        m_motor.setControl(new PositionVoltage(position).withSlot(0));
    }

    @Override
    public Command runProfileToPosition(double position) {
        return new FunctionalCommand(
            () -> {
                m_desiredPosition = position;
                m_motor.setControl(new MotionMagicVoltage(position));
            },
            () -> {},
            (isFinished) -> {},
            this::getIsAtSetpoint,
            this
        );
    }

    @Override
    public void manualControl(Supplier<Double> throttle) {
        double adjustedThrottle = MathUtil.applyDeadband(throttle.get(), OperatorConstants.kOperatorControllerDeadband)
            * ClimbConstants.kManualMultiplier;

        double newDesiredPosition = MathUtil.clamp(m_desiredPosition + adjustedThrottle, ClimbConstants.kMinPosition, ClimbConstants.kMaxPosition);

        if (m_desiredPosition != newDesiredPosition) {
            m_desiredPosition = newDesiredPosition;
            runToPosition(m_desiredPosition);
        }
    }
}
