package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIds;

public class IntakeDriver extends SubsystemBase implements VoltageSubsystem {
    private double m_desiredVoltage;
    private TalonFX m_motor;

    public IntakeDriver() {
        m_motor = new TalonFX(MotorIds.kIntakeDriver, Constants.hasCANivore() ? "*" : "rio");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = IntakeConstants.kDriverGearRatio;
        config.Slot0.kP = IntakeConstants.kDriverKP;
        config.Slot0.kI = IntakeConstants.kDriverKI;
        config.Slot0.kD = IntakeConstants.kDriverKD;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_motor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Driver/Voltage", getVoltage());
        Logger.recordOutput("Intake/Driver/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Intake/Driver/Current", m_motor.getStatorCurrent().getValueAsDouble());
    }
}
