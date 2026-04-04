package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIds;

public class IntakeDriver extends SubsystemBase implements VoltageSubsystem {
    private double m_desiredVoltage;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;
    // private DigitalInput m_beamBreak;
    private IntakeDriverState m_state = IntakeDriverState.OFF;

    public IntakeDriver() {
        m_motor = new TalonFX(DeviceIds.getIntakeDriverID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IntakeConstants.getDriverGearRatio()), DCMotor.getKrakenX60(1));
        // m_beamBreak = new DigitalInput(DeviceIds.kIntakeBeamBreak);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = IntakeConstants.getDriverGearRatio();
        config.Slot0.kP = IntakeConstants.getDriverKP();
        config.Slot0.kI = IntakeConstants.getDriverKI();
        config.Slot0.kD = IntakeConstants.getDriverKD();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_motor.setVoltage(m_desiredVoltage);
    }

    public Command intake() {
        return new InstantCommand(() -> {
            m_state = IntakeDriverState.INTAKING;
            setVoltage(IntakeConstants.getDriverIntakeVoltage());
        }, this);
    }

    public Command off() {
        return new InstantCommand(() -> {
            m_state = IntakeDriverState.OFF;
            setVoltage(IntakeConstants.getDriverOffVoltage());
        }, this);
    }
    
    public Command extake() {
        return new InstantCommand(() -> {
            m_state = IntakeDriverState.EXTAKING;
            setVoltage(IntakeConstants.getDriverExtakeVoltage());
        }, this);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Intaking", m_state == IntakeDriverState.INTAKING);
        SmartDashboard.putBoolean("Is Extaking", m_state == IntakeDriverState.EXTAKING);
        Logger.recordOutput("Intake/Driver/Voltage", getVoltage());
        Logger.recordOutput("Intake/Driver/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Intake/Driver/Current", m_motor.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Intake/Beam Break", m_beamBreak.get());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_motor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IntakeConstants.getDriverGearRatio());
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IntakeConstants.getDriverGearRatio());
    }

    public enum IntakeDriverState {
        INTAKING,
        OFF,
        EXTAKING,
    }
}
