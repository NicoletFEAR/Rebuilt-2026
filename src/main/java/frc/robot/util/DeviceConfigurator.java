// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.SwerveConstantsInterface;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class DeviceConfigurator {
    /* Configure the `CANcoder` and offsets it by `offset` */
    public static void configureCANcoder(CANcoder encoder, double offset) {
        CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // Make sensor wrap-arround unsigned
        configuration.MagnetSensor.MagnetOffset = offset;
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // Set the sensor to mesure positive distance as counter clockwise

        encoder.getConfigurator().apply(configuration); // Apply the configuration
    }

    public static void configureSparkMaxSteerMotor(SparkMax motor, SwerveConstantsInterface constants) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(constants.getTurnRotationsToDegrees());

        config.closedLoop
            .p(constants.getTurnKP())
            .i(constants.getTurnKI())
            .d(constants.getTurnKD())
            .feedForward
            .kS(constants.getTurnKS())
            .kV(constants.getTurnKV())
            .kA(constants.getTurnKA());

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0);
    }

    public static void configureTalonFXDriveMotor(TalonFX motor, SwerveConstantsInterface constants) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 360;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constants.getRampRate();
        config.Feedback.SensorToMechanismRatio = constants.getDriveRevToMeters();

        config.Slot0.kP = constants.getDriveKP();
        config.Slot0.kI = constants.getDriveKI();
        config.Slot0.kD = constants.getDriveKD();
        config.Slot0.kV = constants.getDriveKV();
        config.Slot0.kS = constants.getDriveKS();
        config.Slot0.kA = constants.getDriveKA();

        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }

    public static void configureSparkMaxDriveMotor(SparkMax motor, SwerveConstantsInterface constants) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(constants.getRampRate());

        config.encoder.positionConversionFactor(1 / constants.getDriveRevToMeters())
            .velocityConversionFactor(60 / constants.getDriveRevToMeters());

        config.closedLoop
            .p(constants.getDriveKP())
            .i(constants.getDriveKI())
            .d(constants.getDriveKD())
            .feedForward
            .kS(constants.getDriveKS())
            .kV(constants.getDriveKV())
            .kA(constants.getDriveKA());

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0);
    }
}
