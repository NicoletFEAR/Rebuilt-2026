// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.PositionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DeviceIds;

public class Climb extends SubsystemBase implements PositionSubsystem {
    private double m_desiredPosition;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;

    private double m_minPosition;
    private double m_maxPosition;

    public Climb() {
        m_motor = new TalonFX(DeviceIds.getClimbID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, ClimbConstants.getGearRatio()), DCMotor.getKrakenX60(1));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ClimbConstants.getKP();
        config.Slot0.kI = ClimbConstants.getKI();
        config.Slot0.kD = ClimbConstants.getKD();
        config.Feedback.SensorToMechanismRatio = ClimbConstants.getGearRatio();
        m_motor.getConfigurator().apply(config);

        m_desiredPosition = ClimbConstants.getHomePosition();
        m_minPosition = ClimbConstants.getMinPosition();
        m_maxPosition = ClimbConstants.getMaxPosition();
        m_motor.setPosition(m_desiredPosition);

        SmartDashboard.putData("MechSettings/Climb/Reset Min Climb Position", new InstantCommand(() -> m_minPosition = getPosition()));
        SmartDashboard.putData("MechSettings/Climb/Reset Max Climb Position", new InstantCommand(() -> m_maxPosition = getPosition()));
    }

    @Override
    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < ClimbConstants.getSetpointTolerance();
    }

    @Override
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    @Override
    public void runToPosition(double position) {
        m_desiredPosition = position;
        m_motor.setControl(new PositionVoltage(m_desiredPosition).withSlot(0));
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

    public void resetDesiredPosition() {
        runToPosition(getPosition());
    }

    public Command climbL1Height() {
        return runProfileToPosition(ClimbConstants.getL1Position());
    }

    public Command retract() {
        return runProfileToPosition(ClimbConstants.getHomePosition());
    }

    @Override
    public void manualControl(Supplier<Double> throttle) {
        double adjustedThrottle = throttle.get() * ClimbConstants.getManualMultiplier();

        double newDesiredPosition = MathUtil.clamp(m_desiredPosition + adjustedThrottle, m_minPosition, m_maxPosition);

        if (m_desiredPosition != newDesiredPosition) {
            m_desiredPosition = newDesiredPosition;
            runToPosition(m_desiredPosition);
        }
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
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_motor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * ClimbConstants.getGearRatio());
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * ClimbConstants.getGearRatio());
    }
}
