package frc.robot.io.encoder;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Robot;
import frc.robot.io.base.IO;
import frc.robot.io.encoder.EncoderValues.EncoderConfiguration;

public abstract class EncoderIO extends IO<EncoderState, EncoderIO, EncoderIdentity> {
    private final EncoderConfiguration configuration;

    public EncoderIO(EncoderConfiguration configuration) {
        this.configuration = configuration;
        state = new EncoderState();
    }

    @Override
    public EncoderState updateState() {
        state.CurrentIdentity = EncoderIdentity.NONE;
        state.Position = Radians.of(0.0);

        if (Robot.isSimulation()) {
            state.ProperIdentity = EncoderIdentity.CAN_CODER_SIMULATED;
        } else {
            CANcoder canCoder = new CANcoder(configuration.id().device(), configuration.id().bus());

            if (canCoder.isConnected()) {
                canCoder.close();
                state.ProperIdentity = EncoderIdentity.CAN_CODER;
            } else {
                canCoder.close();
                state.ProperIdentity = EncoderIdentity.NONE;
            }
        }

        state.Velocity = RadiansPerSecond.of(0.0);
        return state;
    }

    @Override
    public EncoderIO getProperIO() {
        return switch (state.ProperIdentity) {
            case NONE -> new EncoderIONone(configuration);
            case CAN_CODER -> new EncoderIONone(configuration);
            case CAN_CODER_SIMULATED -> new EncoderIONone(configuration);
        };
    }
}
