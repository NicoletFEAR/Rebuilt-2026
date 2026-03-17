package frc.robot.util;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public final class Equality {
    public static boolean equals(AudioConfigs a, AudioConfigs b) {
        return a.AllowMusicDurDisable == b.AllowMusicDurDisable
            && a.BeepOnBoot == b.BeepOnBoot
            && a.BeepOnConfig == b.BeepOnConfig;
    }

    public static boolean equals(CANcoderConfiguration a, CANcoderConfiguration b) {
        return equals(a.CustomParams, b.CustomParams)
            && a.FutureProofConfigs == b.FutureProofConfigs
            && equals(a.MagnetSensor, b.MagnetSensor);
    }

    public static boolean equals(ClosedLoopGeneralConfigs a, ClosedLoopGeneralConfigs b) {
        return a.ContinuousWrap == b.ContinuousWrap
            && a.DifferentialContinuousWrap == b.DifferentialContinuousWrap
            && MathUtil.isNear(a.GainSchedErrorThreshold, b.GainSchedErrorThreshold, Constants.kGeneralTolerance)
            && a.GainSchedKpBehavior == b.GainSchedKpBehavior;
    }

    public static boolean equals(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
        return MathUtil.isNear(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod, Constants.kGeneralTolerance)
            && MathUtil.isNear(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod, Constants.kGeneralTolerance * 10.0)
            && MathUtil.isNear(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod, Constants.kGeneralTolerance);
    }

    public static boolean equals(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
        return MathUtil.isNear(a.StatorCurrentLimit, b.StatorCurrentLimit, Constants.kGeneralTolerance * 800.0)
            && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
            && MathUtil.isNear(a.SupplyCurrentLimit, b.SupplyCurrentLimit, Constants.kGeneralTolerance * 800.0)
            && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable
            && MathUtil.isNear(a.SupplyCurrentLowerLimit, b.SupplyCurrentLowerLimit, Constants.kGeneralTolerance * 500.0)
            && MathUtil.isNear(a.SupplyCurrentLowerTime, b.SupplyCurrentLowerTime, Constants.kGeneralTolerance * 5.0);
    }

    public static boolean equals(CustomParamsConfigs a, CustomParamsConfigs b) {
        return a.CustomParam0 == b.CustomParam0
            && a.CustomParam1 == b.CustomParam1;
    }

    public static boolean equals(DifferentialConstantsConfigs a, DifferentialConstantsConfigs b) {
        return MathUtil.isNear(a.PeakDifferentialDutyCycle, b.PeakDifferentialDutyCycle, Constants.kGeneralTolerance)
            && MathUtil.isNear(a.PeakDifferentialTorqueCurrent, b.PeakDifferentialTorqueCurrent, Constants.kGeneralTolerance * 800.0)
            && MathUtil.isNear(a.PeakDifferentialVoltage, b.PeakDifferentialVoltage, Constants.kGeneralTolerance * 32.0);
    }

    public static boolean equals(DifferentialSensorsConfigs a, DifferentialSensorsConfigs b) {
        return a.DifferentialRemoteSensorID == b.DifferentialRemoteSensorID
            && a.DifferentialSensorSource == b.DifferentialSensorSource
            && a.DifferentialTalonFXSensorID == b.DifferentialTalonFXSensorID
            && MathUtil.isNear(a.SensorToDifferentialRatio, b.SensorToDifferentialRatio, Constants.kUnboundedTolerance);
    }

    public static boolean equals(FeedbackConfigs a, FeedbackConfigs b) {
        return a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID
            && MathUtil.isNear(a.FeedbackRotorOffset, b.FeedbackRotorOffset, Constants.kGeneralTolerance * 2.0)
            && a.FeedbackSensorSource == b.FeedbackSensorSource
            && MathUtil.isNear(a.RotorToSensorRatio, b.RotorToSensorRatio, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.SensorToMechanismRatio, b.SensorToMechanismRatio, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.VelocityFilterTimeConstant, b.VelocityFilterTimeConstant, Constants.kGeneralTolerance);
    }

    public static boolean equals(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
        return a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
            && MathUtil.isNear(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue, Constants.kUnboundedTolerance)
            && a.ForwardLimitEnable == b.ForwardLimitEnable
            && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
            && a.ForwardLimitSource == b.ForwardLimitSource
            && a.ForwardLimitType == b.ForwardLimitType
            && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
            && MathUtil.isNear(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue, Constants.kUnboundedTolerance)
            && a.ReverseLimitEnable == b.ReverseLimitEnable
            && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
            && a.ReverseLimitSource == b.ReverseLimitSource
            && a.ReverseLimitType == b.ReverseLimitType;
    }

    public static boolean equals(MagnetSensorConfigs a, MagnetSensorConfigs b) {
        return MathUtil.isNear(a.AbsoluteSensorDiscontinuityPoint, b.AbsoluteSensorDiscontinuityPoint, Constants.kGeneralTolerance)
            && MathUtil.isNear(a.MagnetOffset, b.MagnetOffset, Constants.kGeneralTolerance * 2.0)
            && a.SensorDirection == b.SensorDirection;
    }

    public static boolean equals(MotionMagicConfigs a, MotionMagicConfigs b) {
        return MathUtil.isNear(a.MotionMagicAcceleration, b.MotionMagicAcceleration, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.MotionMagicExpo_kA, b.MotionMagicExpo_kA, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.MotionMagicExpo_kV, b.MotionMagicExpo_kV, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.MotionMagicJerk, b.MotionMagicJerk, Constants.kUnboundedTolerance);
    }

    public static boolean equals(MotorOutputConfigs a, MotorOutputConfigs b) {
        return MathUtil.isNear(a.ControlTimesyncFreqHz, b.ControlTimesyncFreqHz, Constants.kGeneralTolerance * 450.0)
            && MathUtil.isNear(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, Constants.kGeneralTolerance * 0.25)
            && a.Inverted == b.Inverted
            && a.NeutralMode == b.NeutralMode
            && MathUtil.isNear(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle, Constants.kGeneralTolerance * 2.0)
            && MathUtil.isNear(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle, Constants.kGeneralTolerance * 2.0);
    }

    public static boolean equals(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
        return MathUtil.isNear(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod, Constants.kGeneralTolerance)
            && MathUtil.isNear(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod, Constants.kGeneralTolerance * 10.0)
            && MathUtil.isNear(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod, Constants.kGeneralTolerance);
    }

    public static boolean equals(Slot0Configs a, Slot0Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, Constants.kGeneralTolerance * 0.5)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kD, b.kD, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kG, b.kG, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kI, b.kI, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kP, b.kP, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kS, b.kS, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kV, b.kV, Constants.kUnboundedTolerance)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(Slot1Configs a, Slot1Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, Constants.kGeneralTolerance * 0.5)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kD, b.kD, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kG, b.kG, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kI, b.kI, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kP, b.kP, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kS, b.kS, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kV, b.kV, Constants.kUnboundedTolerance)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(Slot2Configs a, Slot2Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, Constants.kGeneralTolerance * 0.5)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kD, b.kD, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kG, b.kG, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kI, b.kI, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kP, b.kP, Constants.kUnboundedTolerance)
            && MathUtil.isNear(a.kS, b.kS, Constants.kGeneralTolerance * 12.0)
            && MathUtil.isNear(a.kV, b.kV, Constants.kUnboundedTolerance)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
        return a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable
            && MathUtil.isNear(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold, Constants.kUnboundedTolerance)
            && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
            && MathUtil.isNear(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold, Constants.kUnboundedTolerance);
    }

    public static boolean equals(TalonFXConfiguration a, TalonFXConfiguration b) {
        return equals(a.Audio, b.Audio)
            && equals(a.ClosedLoopGeneral, b.ClosedLoopGeneral)
            && equals(a.ClosedLoopRamps, b.ClosedLoopRamps)
            && equals(a.CurrentLimits, b.CurrentLimits)
            && equals(a.CustomParams, b.CustomParams)
            && equals(a.DifferentialConstants, b.DifferentialConstants)
            && equals(a.DifferentialSensors, b.DifferentialSensors)
            && equals(a.Feedback, b.Feedback)
            && a.FutureProofConfigs == b.FutureProofConfigs
            && equals(a.HardwareLimitSwitch, b.HardwareLimitSwitch)
            && equals(a.MotionMagic, b.MotionMagic)
            && equals(a.MotorOutput, b.MotorOutput)
            && equals(a.OpenLoopRamps, b.OpenLoopRamps)
            && equals(a.Slot0, b.Slot0)
            && equals(a.Slot1, b.Slot1)
            && equals(a.Slot2, b.Slot2)
            && equals(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch)
            && equals(a.TorqueCurrent, b.TorqueCurrent)
            && equals(a.Voltage, b.Voltage);
    }

    public static boolean equals(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
        return MathUtil.isNear(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, Constants.kGeneralTolerance * 1600.0)
            && MathUtil.isNear(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, Constants.kGeneralTolerance * 1600.0)
            && MathUtil.isNear(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, Constants.kGeneralTolerance * 25.0);
    }

    public static boolean equals(VoltageConfigs a, VoltageConfigs b) {
        return MathUtil.isNear(a.PeakForwardVoltage, b.PeakForwardVoltage, Constants.kGeneralTolerance * 24.0)
            && MathUtil.isNear(a.PeakReverseVoltage, b.PeakReverseVoltage, Constants.kGeneralTolerance * 24.0)
            && MathUtil.isNear(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant, Constants.kGeneralTolerance * 0.1);
    }

    private Equality() {}
}
