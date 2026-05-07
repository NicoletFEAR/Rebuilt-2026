package frc.robot.io.encoder;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CANId;

public final class EncoderValues {
    public static record EncoderConfiguration(CANId id, Angle offset) {}

    public static class EncoderConfigurationBuilder {
        private final CANId id;
        private Angle offset = Radians.of(0.0);

        public EncoderConfigurationBuilder(CANId id) {
            this.id = id;
        }

        public EncoderConfigurationBuilder offset(Angle offset) {
            this.offset = offset;
            return this;
        }

        public EncoderConfiguration build() {
            return new EncoderConfiguration(id, offset);
        }
    }

    private EncoderValues() {}
}
