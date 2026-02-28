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
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DeviceIds;

public class IntakePivot extends SubsystemBase implements PositionSubsystem {
    private double m_desiredPosition;
    private TalonFX m_motor;
    private DCMotorSim m_motorSim;

    private double m_minPosition;
    private double m_maxPosition;

    public IntakePivot() {
        m_motor = new TalonFX(DeviceIds.getIntakePivotID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, IntakeConstants.getPivotGearRatio()), DCMotor.getKrakenX60(1));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.getPivotKP();
        config.Slot0.kI = IntakeConstants.getPivotKI();
        config.Slot0.kD = IntakeConstants.getPivotKD();
        config.Feedback.SensorToMechanismRatio = IntakeConstants.getPivotGearRatio();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);

        m_desiredPosition = IntakeConstants.getPivotHomePosition();
        m_minPosition = IntakeConstants.getPivotMinPosition();
        m_maxPosition = IntakeConstants.getPivotMaxPosition();
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

    public Command out() {
        return runProfileToPosition(IntakeConstants.getPivotOutPosition());
    }

    public Command hold() {
        return runProfileToPosition(IntakeConstants.getPivotHoldPosition());
    }

    public Command in() {
        return runProfileToPosition(IntakeConstants.getPivotHomePosition());
    }

    public boolean isStuckOnBall() {
        return m_motor.getStatorCurrent().getValueAsDouble() > IntakeConstants.getIntakeStuckOnBallThreshold();
    }

    @Override
    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < IntakeConstants.getPivotSetpointTolerance();
    }

    @Override
    public void manualControl(Supplier<Double> throttle, boolean limitOverrideMode) {
        double adjustedThrottle = throttle.get() * IntakeConstants.getPivotManualModifier();

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
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * IntakeConstants.getPivotGearRatio());
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * IntakeConstants.getPivotGearRatio());
    }
}
