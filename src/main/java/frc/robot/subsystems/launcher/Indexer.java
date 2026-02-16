package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.DeviceIds;

public class Indexer extends SubsystemBase implements VoltageSubsystem{
    private double m_desiredVoltage;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;
    // private DigitalInput m_beamBreak;

    public Indexer () {
        m_motor = new TalonFX(DeviceIds.getIndexerID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        // m_beamBreak = new DigitalInput(DeviceIds.kIndexerBeamBreak);
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IndexerConstants.getGearRatio()), DCMotor.getKrakenX60(1));

        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.Feedback.SensorToMechanismRatio = IndexerConstants.getGearRatio();
        leadConfig.Slot0.kP = IndexerConstants.getKP();
        leadConfig.Slot0.kI = IndexerConstants.getKI();
        leadConfig.Slot0.kD = IndexerConstants.getKD();
        m_motor.getConfigurator().apply(leadConfig);
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

    public Command index() {
        return new WaitCommand(0.5).andThen(new InstantCommand(() -> setVoltage(IndexerConstants.getIndexVoltage())));
    }

    public Command off() {
        return new InstantCommand(() -> setVoltage(IndexerConstants.getOffVoltage()), this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Launcher/Indexer/Voltage", getVoltage());
        Logger.recordOutput("Launcher/Indexer/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Launcher/Indexer/Current", m_motor.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Indexer/Beam Break", m_beamBreak.get());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_motor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IndexerConstants.getGearRatio());
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IndexerConstants.getGearRatio());
    }
}
