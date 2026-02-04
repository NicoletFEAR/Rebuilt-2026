package frc.robot.subsystems.kitbot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitbotSubsystemConstants;

public class Launcher extends SubsystemBase {
    private static Launcher m_instance = null;

    private SparkMax m_intakeMotor;

    // 
    public Launcher() {
        m_intakeMotor = new SparkMax(KitbotSubsystemConstants.kLauncher, MotorType.kBrushed);
    }

    public static Launcher getInstance() {
        if (m_instance == null) {
            m_instance = new Launcher();
        }

        return m_instance;
    }

    // Get Launcher Voltage
    public double getVoltage() {
        return m_intakeMotor.getAppliedOutput();
    }

    // Sets the intake Voltage to a chosen number
    public void setVoltage(double voltage) {
        m_intakeMotor.setVoltage(voltage);
    }

    // Sets the voltages of the motors to zero
    public Command off() {
        return new InstantCommand(() -> setVoltage(0), this);
    }

    public Command intake() {
        return new InstantCommand(() -> setVoltage(12), this);
    }

    // Launches fuel
    public Command launch() {
        return new InstantCommand(() -> setVoltage(-12), this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Voltage", getVoltage());
    }
}
