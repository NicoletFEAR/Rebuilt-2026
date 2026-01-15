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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants.DriveConstants;

public class DeviceConfigurator {
  /* Configure the `CANcoder` and offsets it by `offset` */
  public static void configureCANcoder(CANcoder encoder, double offset) {
    CANcoderConfiguration configuration = new CANcoderConfiguration();

    encoder.getConfigurator().apply(configuration);

    configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // Make sensor wrap-arround unsigned
    configuration.MagnetSensor.MagnetOffset                     = offset;
    configuration.MagnetSensor.SensorDirection                  = SensorDirectionValue.CounterClockwise_Positive; // Set the sensor to mesure positive distance as counter clockwise

    encoder.getConfigurator().apply(configuration); // Apply the configuration
  }

  public static void configureSparkMaxSteerMotor(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true)
          .smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake);

    config.encoder.positionConversionFactor(DriveConstants.getTurnRotationsToDegrees());

    config.closedLoop
          .p(DriveConstants.getTurnKP())
          .i(DriveConstants.getTurnKI())
          .d(DriveConstants.getTurnKD())
          .velocityFF(DriveConstants.getTurnKFF());

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  public static void configureSparkFlexDriveMotor(SparkFlex motor) {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(true)
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake)
          .openLoopRampRate(DriveConstants.getDriveRampRate());

    config.encoder.positionConversionFactor(DriveConstants.getDriveGearRatio())
                  .velocityConversionFactor(DriveConstants.getDriveGearRatio() / 60);

    // config.closedLoop.p(DriveConstants.drivekp)
    //                  .i(DriveConstants.driveki)
    //                  .d(DriveConstants.drivekd)
    //                  .velocityFF(DriveConstants.drivekff);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  public static void configureTalonFXDriveMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 360;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.getDriveRampRate();
    config.Feedback.SensorToMechanismRatio = DriveConstants.getDriveRevToMeters();

    config.Slot0.kP = DriveConstants.getDriveKP();
    config.Slot0.kI = DriveConstants.getDriveKI();
    config.Slot0.kD = DriveConstants.getDriveKD();
    config.Slot0.kV = DriveConstants.getDriveKV();
    config.Slot0.kS = DriveConstants.getDriveKS();
    config.Slot0.kA = DriveConstants.getDriveKA();

    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }
}
