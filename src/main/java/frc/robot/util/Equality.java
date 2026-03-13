package frc.robot.util;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
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

public class Equality {
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

    public static boolean equals(AudioConfigs a, AudioConfigs b) {
        return a.AllowMusicDurDisable == b.AllowMusicDurDisable
            && a.BeepOnBoot == b.BeepOnBoot
            && a.BeepOnConfig == b.BeepOnConfig;
    }

    public static boolean equals(ClosedLoopGeneralConfigs a, ClosedLoopGeneralConfigs b) {
        return a.ContinuousWrap == b.ContinuousWrap
            && a.DifferentialContinuousWrap == b.DifferentialContinuousWrap
            && MathUtil.isNear(a.GainSchedErrorThreshold, b.GainSchedErrorThreshold, 0.05)
            && a.GainSchedKpBehavior == b.GainSchedKpBehavior;
    }

    public static boolean equals(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
        return MathUtil.isNear(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod, 0.05)
            && MathUtil.isNear(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod, 0.05)
            && MathUtil.isNear(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod, 0.05);
    }

    public static boolean equals(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
        return MathUtil.isNear(a.StatorCurrentLimit, b.StatorCurrentLimit, 0.05)
            && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
            && MathUtil.isNear(a.SupplyCurrentLimit, b.SupplyCurrentLimit, 0.05)
            && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable
            && MathUtil.isNear(a.SupplyCurrentLowerLimit, b.SupplyCurrentLowerLimit, 0.05)
            && MathUtil.isNear(a.SupplyCurrentLowerTime, b.SupplyCurrentLowerTime, 0.05);
    }

    public static boolean equals(CustomParamsConfigs a, CustomParamsConfigs b) {
        return a.CustomParam0 == b.CustomParam0
            && a.CustomParam1 == b.CustomParam1;
    }

    public static boolean equals(DifferentialConstantsConfigs a, DifferentialConstantsConfigs b) {
        return MathUtil.isNear(a.PeakDifferentialDutyCycle, b.PeakDifferentialDutyCycle, 0.05)
            && MathUtil.isNear(a.PeakDifferentialTorqueCurrent, b.PeakDifferentialTorqueCurrent, 0.05)
            && MathUtil.isNear(a.PeakDifferentialVoltage, b.PeakDifferentialVoltage, 0.05);
    }

    public static boolean equals(DifferentialSensorsConfigs a, DifferentialSensorsConfigs b) {
        return a.DifferentialRemoteSensorID == b.DifferentialRemoteSensorID
            && a.DifferentialSensorSource == b.DifferentialSensorSource
            && a.DifferentialTalonFXSensorID == b.DifferentialTalonFXSensorID
            && MathUtil.isNear(a.SensorToDifferentialRatio, b.SensorToDifferentialRatio, 0.05);
    }

    public static boolean equals(FeedbackConfigs a, FeedbackConfigs b) {
        return a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID
            && MathUtil.isNear(a.FeedbackRotorOffset, b.FeedbackRotorOffset, 0.05)
            && a.FeedbackSensorSource == b.FeedbackSensorSource
            && MathUtil.isNear(a.RotorToSensorRatio, b.RotorToSensorRatio, 0.05)
            && MathUtil.isNear(a.SensorToMechanismRatio, b.SensorToMechanismRatio, 0.05)
            && MathUtil.isNear(a.VelocityFilterTimeConstant, b.VelocityFilterTimeConstant, 0.05);
    }

    public static boolean equals(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
        return a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
            && MathUtil.isNear(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue, 0.05)
            && a.ForwardLimitEnable == b.ForwardLimitEnable
            && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
            && a.ForwardLimitSource == b.ForwardLimitSource
            && a.ForwardLimitType == b.ForwardLimitType
            && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
            && MathUtil.isNear(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue, 0.05)
            && a.ReverseLimitEnable == b.ReverseLimitEnable
            && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
            && a.ReverseLimitSource == b.ReverseLimitSource
            && a.ReverseLimitType == b.ReverseLimitType;
    }

    public static boolean equals(MotionMagicConfigs a, MotionMagicConfigs b) {
        return MathUtil.isNear(a.MotionMagicAcceleration, b.MotionMagicAcceleration, 0.05)
            && MathUtil.isNear(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, 0.05)
            && MathUtil.isNear(a.MotionMagicExpo_kA, b.MotionMagicExpo_kA, 0.05)
            && MathUtil.isNear(a.MotionMagicExpo_kV, b.MotionMagicExpo_kV, 0.05)
            && MathUtil.isNear(a.MotionMagicJerk, b.MotionMagicJerk, 0.05);
    }

    public static boolean equals(MotorOutputConfigs a, MotorOutputConfigs b) {
        return MathUtil.isNear(a.ControlTimesyncFreqHz, b.ControlTimesyncFreqHz, 0.05)
            && MathUtil.isNear(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, 0.05)
            && a.Inverted == b.Inverted
            && a.NeutralMode == b.NeutralMode
            && MathUtil.isNear(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle, 0.05)
            && MathUtil.isNear(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle, 0.05);
    }

    public static boolean equals(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
        return MathUtil.isNear(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod, 0.05)
            && MathUtil.isNear(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod, 0.05)
            && MathUtil.isNear(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod, 0.05);
    }

    public static boolean equals(Slot0Configs a, Slot0Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, 0.05)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, 0.05)
            && MathUtil.isNear(a.kD, b.kD, 0.05)
            && MathUtil.isNear(a.kG, b.kG, 0.05)
            && MathUtil.isNear(a.kI, b.kI, 0.05)
            && MathUtil.isNear(a.kP, b.kP, 0.05)
            && MathUtil.isNear(a.kS, b.kS, 0.05)
            && MathUtil.isNear(a.kV, b.kV, 0.05)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(Slot1Configs a, Slot1Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, 0.05)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, 0.05)
            && MathUtil.isNear(a.kD, b.kD, 0.05)
            && MathUtil.isNear(a.kG, b.kG, 0.05)
            && MathUtil.isNear(a.kI, b.kI, 0.05)
            && MathUtil.isNear(a.kP, b.kP, 0.05)
            && MathUtil.isNear(a.kS, b.kS, 0.05)
            && MathUtil.isNear(a.kV, b.kV, 0.05)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(Slot2Configs a, Slot2Configs b) {
        return a.GainSchedBehavior == b.GainSchedBehavior
            && MathUtil.isNear(a.GravityArmPositionOffset, b.GravityArmPositionOffset, 0.05)
            && a.GravityType == b.GravityType
            && MathUtil.isNear(a.kA, b.kA, 0.05)
            && MathUtil.isNear(a.kD, b.kD, 0.05)
            && MathUtil.isNear(a.kG, b.kG, 0.05)
            && MathUtil.isNear(a.kI, b.kI, 0.05)
            && MathUtil.isNear(a.kP, b.kP, 0.05)
            && MathUtil.isNear(a.kS, b.kS, 0.05)
            && MathUtil.isNear(a.kV, b.kV, 0.05)
            && a.StaticFeedforwardSign == b.StaticFeedforwardSign;
    }

    public static boolean equals(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
        return a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable
            && MathUtil.isNear(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold, 0.05)
            && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
            && MathUtil.isNear(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold, 0.05);
    }

    public static boolean equals(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
        return MathUtil.isNear(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, 0.05)
            && MathUtil.isNear(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, 0.05)
            && MathUtil.isNear(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, 0.05);
    }

    public static boolean equals(VoltageConfigs a, VoltageConfigs b) {
        return MathUtil.isNear(a.PeakForwardVoltage, b.PeakForwardVoltage, 0.05)
            && MathUtil.isNear(a.PeakReverseVoltage, b.PeakReverseVoltage, 0.05)
            && MathUtil.isNear(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant, 0.05);
    }
}
