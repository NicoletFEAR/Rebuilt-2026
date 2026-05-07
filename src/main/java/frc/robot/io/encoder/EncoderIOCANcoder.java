package frc.robot.io.encoder;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.io.encoder.EncoderValues.EncoderConfiguration;
import frc.robot.util.Configurator;

public class EncoderIOCANcoder extends EncoderIO {
    private final CANcoder encoder;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    public EncoderIOCANcoder(EncoderConfiguration configuration) {
        super(configuration);
        encoder = new CANcoder(configuration.id().device(), configuration.id().bus());
        CANcoderConfiguration directConfiguration = new CANcoderConfiguration();
        directConfiguration.MagnetSensor.MagnetOffset = configuration.offset().in(Rotations);
        Configurator.configure(encoder, directConfiguration);
        position = encoder.getAbsolutePosition();
        velocity = encoder.getVelocity();
    }

    @Override
    public EncoderState updateState() {
        BaseStatusSignal.refreshAll(position, velocity);
        state.CurrentIdentity = EncoderIdentity.CAN_CODER;
        state.Position = position.getValue();
        state.ProperIdentity = encoder.isConnected() ? EncoderIdentity.CAN_CODER : EncoderIdentity.NONE;
        state.Velocity = velocity.getValue();
        return state;
    }
}
