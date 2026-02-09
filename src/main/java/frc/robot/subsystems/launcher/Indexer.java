package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
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
    private TalonFX m_indexLauncher;
    private TalonFX m_indexIntake;
    private DCMotorSim m_motorSim;

    public Indexer () {
        m_indexLauncher = new TalonFX(DeviceIds.kIndexLauncher, Constants .hasCANivore() ? "*" : "rio");
        m_indexIntake = new TalonFX(DeviceIds.kIndexIntake, Constants .hasCANivore() ? "*" : "rio");
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IndexerConstants.kGearRatio), DCMotor.getKrakenX60(1));

        m_indexIntake.setControl(new Follower(DeviceIds.kIndexLauncher, MotorAlignmentValue.Aligned));

        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.Feedback.SensorToMechanismRatio = IndexerConstants.kGearRatio;
        leadConfig.Slot0.kP = IndexerConstants.kP;
        leadConfig.Slot0.kI = IndexerConstants.kI;
        leadConfig.Slot0.kD = IndexerConstants.kD;
        m_indexLauncher.getConfigurator().apply(leadConfig);
    }

     @Override
    public double getVoltage() {
        return m_indexLauncher.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_indexLauncher.setVoltage(m_desiredVoltage);
    }

    public Command launch() {
        return new WaitCommand(2).andThen(new InstantCommand(() -> setVoltage(IndexerConstants.kIndexVoltage), this));
    }

    public Command off() {
        return new InstantCommand(() -> setVoltage(IndexerConstants.kOffVoltage), this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/Voltage", getVoltage());
        Logger.recordOutput("Indexer/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Indexer/Current", m_indexLauncher.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_indexLauncher.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IndexerConstants.kGearRatio);
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IndexerConstants.kGearRatio);
    }
}
