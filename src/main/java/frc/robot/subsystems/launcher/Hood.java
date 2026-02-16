package frc.robot.subsystems.launcher;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;

public class Hood extends SubsystemBase {
    private double m_desiredPosition;
    private Servo m_leftServo;
    private Servo m_rightServo;
    private SparkMax m_hood;

    public Hood() {
        //m_hood = new SparkMax(DeviceIds.getHoodID(), MotorType.kBrushed);

        m_leftServo = new Servo(DeviceIds.kLeftLauncherServo);
        m_rightServo = new Servo(DeviceIds.kRightLauncherServo);

        m_leftServo.setBoundsMicroseconds(2000, 1600, 1500, 1400, 1000);
        m_rightServo.setBoundsMicroseconds(2000, 1600, 1500, 1400, 1000);
    }

    // TODO: Add position estimation
    public double getPosition() {
        return m_desiredPosition;
    }

    public void runToPosition(double position) {
        m_desiredPosition = position;
        m_leftServo.setPosition(position);
        m_rightServo.setPosition(position);
    }

    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < LauncherConstants.getHoodSetpointTolerance();
    }

    public void manualControl(Supplier<Double> throttle) {
        double adjustedThrottle = MathUtil.applyDeadband(throttle.get(), OperatorConstants.getOperatorControllerDeadband())
            * LauncherConstants.getHoodManualModifier();

        double newDesiredPosition = MathUtil.clamp(m_desiredPosition + adjustedThrottle, LauncherConstants.getHoodMinPosition(), LauncherConstants.getHoodMaxPosition());

        if (m_desiredPosition != newDesiredPosition) {
            runToPosition(m_desiredPosition);
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Launcher/Hood/Desired Position", m_desiredPosition);
        Logger.recordOutput("Launcher/Hood/Position", getPosition());
        Logger.recordOutput("Launcher/Hood/Is At Setpoint", getIsAtSetpoint());
    }
}
