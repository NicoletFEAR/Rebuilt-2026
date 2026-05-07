package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.io.motor.MotorValues.MotorConfiguration;
import frc.robot.util.Configurator;

public class MotorIOTalonFXSimulated extends MotorIO {
    private final TalonFX motor;
    private final DCMotorSim motorModel;
    private final TalonFXSimState motorSimulation;
    private final VoltageOut request = new VoltageOut(Volts.of(0.0));
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;

    public MotorIOTalonFXSimulated(MotorConfiguration configuration) {
        super(configuration);
        motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.004,
                1.0
            ),
            DCMotor.getKrakenX60(1)
        );
        motor = new TalonFX(configuration.id().device(), configuration.id().bus());
        TalonFXConfiguration directConfiguration = new TalonFXConfiguration();

        InvertedValue inverted = switch (configuration.inversion()) {
            case CLOCKWISE_IS_POSITIVE -> InvertedValue.Clockwise_Positive;
            case COUNTER_CLOCKWISE_IS_POSITIVE -> InvertedValue.CounterClockwise_Positive;
        };

        NeutralModeValue neutralMode = switch (configuration.neutralMode()) {
            case BRAKE -> NeutralModeValue.Brake;
            case COAST -> NeutralModeValue.Coast;
        };

        directConfiguration.MotorOutput.Inverted = inverted;
        directConfiguration.MotorOutput.NeutralMode = neutralMode;

        if (configuration.statorCurrentLimit().isPresent()) {
            directConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
            directConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimit().get();
        }

        Configurator.configure(motor, directConfiguration);
        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        motorSimulation = motor.getSimState();
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        motor.setControl(request.withOutput(voltage));
    }

    @Override
    public MotorState updateState() {
        motorModel.setInputVoltage(motorSimulation.getMotorVoltageMeasure().in(Volts));
        motorModel.update(Constants.LOOP_PERIOD);
        motorSimulation.setRawRotorPosition(motorModel.getAngularPosition());
        motorSimulation.setRotorVelocity(motorModel.getAngularVelocity());
        motorSimulation.setRotorAcceleration(motorModel.getAngularAcceleration());
        BaseStatusSignal.refreshAll(position, velocity, voltage);
        state.CurrentIdentity = MotorIdentity.TALON_FX;
        state.Position = position.getValue();
        state.ProperIdentity = motor.isAlive() ? MotorIdentity.NONE : MotorIdentity.TALON_FX;
        state.Velocity = velocity.getValue();
        state.Voltage = voltage.getValue();
        return state;
    }
}
