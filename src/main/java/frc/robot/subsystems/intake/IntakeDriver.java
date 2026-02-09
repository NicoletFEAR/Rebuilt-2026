package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIds;

public class IntakeDriver extends SubsystemBase implements VoltageSubsystem {
    private double m_desiredVoltage;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;

    public IntakeDriver() {
        m_motor = new TalonFX(MotorIds.kIntakeDriver, Constants.hasCANivore() ? "*" : "rio");
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IntakeConstants.kDriverGearRatio), DCMotor.getKrakenX60(1));

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

    public Command intake() {
        return new InstantCommand(() -> setVoltage(12), this);
    }

    public Command off() {
        return new InstantCommand(() -> setVoltage(0), this);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Driver/Voltage", getVoltage());
        Logger.recordOutput("Intake/Driver/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Intake/Driver/Current", m_motor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_motor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IntakeConstants.kDriverGearRatio);
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IntakeConstants.kDriverGearRatio);
    }
}
