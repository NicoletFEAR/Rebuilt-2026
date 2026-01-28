package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

public interface SwerveModule {
    public SwerveModulePosition getModulePosition();
    public SwerveModuleState getModuleState();
    public void resetAngleToAbsolute();
    public void setSwerveModuleState(SwerveModuleState moduleState, boolean isOpenLoop);
    public void runVolts(Voltage volts, double position);
}
