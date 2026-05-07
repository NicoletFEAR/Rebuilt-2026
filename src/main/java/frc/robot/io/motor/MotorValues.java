package frc.robot.io.motor;

import java.util.Optional;

import frc.robot.util.CANId;

public final class MotorValues {
    public static record MotorConfiguration(
        CANId id,
        MotorInversion inversion,
        MotorNeutralMode neutralMode,
        Optional<Double> statorCurrentLimit
    ) {}

    public static enum MotorNeutralMode {
        BRAKE("Brake"),
        COAST("Coast"),
        ;

        private final String name;

        MotorNeutralMode(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    public static enum MotorInversion {
        CLOCKWISE_IS_POSITIVE("Clockwise is Positive"),
        COUNTER_CLOCKWISE_IS_POSITIVE("Counter-Clockwise is Positive"),
        ;

        private final String name;

        MotorInversion(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    public static class MotorConfigurationBuilder {
        private final CANId id;
        private MotorInversion inversion = MotorInversion.COUNTER_CLOCKWISE_IS_POSITIVE;
        private MotorNeutralMode neutralMode = MotorNeutralMode.COAST;
        private Optional<Double> statorCurrentLimit = Optional.empty();

        public MotorConfigurationBuilder(CANId id) {
            this.id = id;
        }

        public MotorConfigurationBuilder inversion(MotorInversion inversion) {
            this.inversion = inversion;
            return this;
        }

        public MotorConfigurationBuilder neutralMode(MotorNeutralMode neutralMode) {
            this.neutralMode = neutralMode;
            return this;
        }

        public MotorConfigurationBuilder statorCurrentLimit(double limit) {
            statorCurrentLimit = Optional.of(limit);
            return this;
        }

        public MotorConfiguration build() {
            return new MotorConfiguration(id, inversion, neutralMode, statorCurrentLimit);
        }
    }

    private MotorValues() {}
}
