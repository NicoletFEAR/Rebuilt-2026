package frc.robot.subsystems.kitbot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitbotSubsystemConstants;;

public class IntakeAndLauncher extends SubsystemBase {
    private static IntakeAndLauncher m_instance = null;

    private SparkMax m_intakeMotor;
    private SparkMax m_launcherMotor;

    // 
    public IntakeAndLauncher() {
        m_intakeMotor = new SparkMax(KitbotSubsystemConstants.kIntake, MotorType.kBrushed);
        m_launcherMotor = new SparkMax(KitbotSubsystemConstants.kLauncher, MotorType.kBrushed);
    }

    public static IntakeAndLauncher getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeAndLauncher();
        }

        return m_instance;
    }

    // Get Intake Voltage
    public double getIntakeVoltage() {
        return m_intakeMotor.getAppliedOutput();
    }

    // Get Launcher Voltage
    public double getLauncherVoltage() {
        return m_launcherMotor.getAppliedOutput();
    }

    // Sets the intake Voltage to a chosen number
    public void setIntakeVoltage(double voltage) {
        m_intakeMotor.setVoltage(voltage);
    }

    // Sets the Launcher Voltage to a chosen number
    public void setLauncherVoltage(double voltage) {
        m_launcherMotor.setVoltage(voltage);
    }

    // Sets the voltages of the motors to zero
    public Command off() {
        return new InstantCommand(() -> setIntakeVoltage(0), this).alongWith(new InstantCommand(() -> setLauncherVoltage(0), this));
    }

    // Intakes fuel
    public Command intake() {
        return new InstantCommand(() -> setIntakeVoltage(12), this);
    }

    // Launches fuel
    public Command launch() {
        return new InstantCommand(() -> setIntakeVoltage(-12), this).alongWith(new InstantCommand(() -> setLauncherVoltage(12), this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", getIntakeVoltage());
        SmartDashboard.putNumber("Launcher Voltage", getLauncherVoltage());
    }
}
