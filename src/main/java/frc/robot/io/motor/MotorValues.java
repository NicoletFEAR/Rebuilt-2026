package frc.robot.io.motor;

import java.util.Optional;

import frc.robot.util.CANId;

public final class MotorValues {
    public static record MotorConfiguration(
        CANId id,
        MotorInversion inversion,
        MotorIdleMode idleMode,
        Optional<Double> statorCurrentLimit
    ) {}

    public static enum MotorIdleMode {
        BRAKE,
        COAST,
    }

    public static enum MotorInversion {
        CLOCKWISE_IS_POSITIVE,
        COUNTER_CLOCKWISE_IS_POSITIVE,
    }

    public static class MotorConfigurationBuilder {
        private final CANId id;
        private MotorInversion inversion = MotorInversion.COUNTER_CLOCKWISE_IS_POSITIVE;
        private MotorIdleMode idleMode = MotorIdleMode.COAST;
        private Optional<Double> statorCurrentLimit = Optional.empty();

        public MotorConfigurationBuilder(CANId id) {
            this.id = id;
        }

        public MotorConfigurationBuilder inversion(MotorInversion inversion) {
            this.inversion = inversion;
            return this;
        }

        public MotorConfigurationBuilder idleMode(MotorIdleMode idleMode) {
            this.idleMode = idleMode;
            return this;
        }

        public MotorConfigurationBuilder statorCurrentLimit(double limit) {
            statorCurrentLimit = Optional.of(limit);
            return this;
        }

        public MotorConfiguration build() {
            return new MotorConfiguration(id, inversion, idleMode, statorCurrentLimit);
        }
    }

    private MotorValues() {}
}
