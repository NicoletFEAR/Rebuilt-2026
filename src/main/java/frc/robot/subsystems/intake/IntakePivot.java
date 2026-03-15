package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.constants.TuskConstants.IntakeConstants;
import frc.robot.constants.TuskConstants.TuskDeviceIds;
import frc.robot.constants.TuskConstants.TuskGeneralConstants;

public class IntakePivot extends SubsystemBase implements PositionSubsystem {
    private double m_desiredPosition;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;

    private double m_minPosition;
    private double m_maxPosition;

    public IntakePivot() {
        m_motor = new TalonFX(TuskDeviceIds.kIntakePivotId, new CANBus(TuskGeneralConstants.kHasCanivore ? "*" : "rio"));
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IntakeConstants.kIntakePivotGearRatio), DCMotor.getKrakenX60(1));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.kIntakePivotKP;
        config.Slot0.kI = IntakeConstants.kIntakePivotKI;
        config.Slot0.kD = IntakeConstants.kIntakePivotKI;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.kIntakePivotGearRatio;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);

        m_desiredPosition = IntakeConstants.kHomePosition;
        m_minPosition = IntakeConstants.kMinPosition;
        m_maxPosition = IntakeConstants.kMaxPosition;
        m_motor.setPosition(m_desiredPosition);

        SmartDashboard.putData("MechSettings/Intake/Pivot/Reset Min Intake Pivot Position", new InstantCommand(() -> m_minPosition = getPosition()));
        SmartDashboard.putData("MechSettings/Intake/Pivot/Reset Max Intake Pivot Position", new InstantCommand(() -> m_maxPosition = getPosition()));
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
                m_motor.setControl(new PositionVoltage(position).withSlot(0));
            },
            () -> {},
            (isFinished) -> {},
            this::getIsAtSetpoint,
            this
        );
    }

    public void resetDesiredPosition() {
        m_desiredPosition = getPosition();
    }

    public Command out() {
        return runProfileToPosition(IntakeConstants.kOutPosition);
    }

    public Command hold() {
        return runProfileToPosition(IntakeConstants.kHoldPosition);
    }

    public Command in() {
        return runProfileToPosition(IntakeConstants.kHomePosition);
    }

    public Command jostleOut() {
        return runProfileToPosition(Math.min(getPosition() + IntakeConstants.kOutPosition / 10.0, m_maxPosition));
    }

    public boolean isStuckOnBall() {
        return m_motor.getStatorCurrent().getValueAsDouble() > IntakeConstants.kIsStuckOnBallThreshold;
    }

    @Override
    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < IntakeConstants.kSetpointTolerance;
    }

    @Override
    public void manualControl(Supplier<Double> throttle, boolean limitOverrideMode) {
        double adjustedThrottle = throttle.get() * IntakeConstants.kManualModifier;

        double newDesiredPosition = m_desiredPosition + adjustedThrottle;

        if (!limitOverrideMode) {
            newDesiredPosition = MathUtil.clamp(newDesiredPosition, m_minPosition, m_maxPosition);
        }

        if (m_desiredPosition != newDesiredPosition) {
            m_desiredPosition = newDesiredPosition;
            runToPosition(m_desiredPosition);
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Pivot/Desired Position", m_desiredPosition);
        Logger.recordOutput("Intake/Pivot/Position", getPosition());
        Logger.recordOutput("Intake/Pivot/Current", m_motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Pivot/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Intake/Pivot/Velocity", m_motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Intake/Pivot/Is At Setpoint", getIsAtSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_motor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IntakeConstants.kIntakePivotGearRatio);
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IntakeConstants.kIntakePivotGearRatio);
    }
}
