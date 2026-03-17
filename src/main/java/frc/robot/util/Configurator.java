package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public final class Configurator {
    public static boolean configure(CANcoder encoder, CANcoderConfiguration configuration) {
        boolean result = false;
        int tries = 0;

        while (result == false && tries < 5) {
            StatusCode code = encoder.getConfigurator().apply(configuration);
            int subtries = 0;

            while (code != StatusCode.OK && subtries < 5) {
                code = encoder.getConfigurator().apply(configuration);
                subtries++;
            }

            if (code == StatusCode.OK) {
                CANcoderConfiguration readConfiguration = new CANcoderConfiguration();
                code = encoder.getConfigurator().refresh(readConfiguration);
                subtries = 0;

                while (code != StatusCode.OK && subtries < 5) {
                    code = encoder.getConfigurator().refresh(readConfiguration);
                    subtries++;
                }

                if (code == StatusCode.OK && Equality.equals(readConfiguration, configuration)) {
                    result = true;
                }
            }

            tries++;
        }

        return result;
    }

    public static boolean configure(TalonFX motor, TalonFXConfiguration configuration) {
        boolean result = false;
        int tries = 0;

        while (result == false && tries < 5) {
            StatusCode code = motor.getConfigurator().apply(configuration);
            int subtries = 0;

            while (code != StatusCode.OK && subtries < 5) {
                code = motor.getConfigurator().apply(configuration);
                subtries++;
            }

            if (code == StatusCode.OK) {
                TalonFXConfiguration readConfiguration = new TalonFXConfiguration();
                code = motor.getConfigurator().refresh(readConfiguration);
                subtries = 0;

                while (code != StatusCode.OK && subtries < 5) {
                    code = motor.getConfigurator().refresh(readConfiguration);
                    subtries++;
                }

                if (code == StatusCode.OK && Equality.equals(readConfiguration, configuration)) {
                    result = true;
                }
            }

            tries++;
        }

        return result;
    }

    private Configurator() {}
}
