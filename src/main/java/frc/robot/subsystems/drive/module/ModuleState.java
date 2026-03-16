package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.base.State;

@AutoLog
public class ModuleState extends State<ModuleState> {
    public boolean DriveConnected = false;
    public Distance DrivePosition = Meters.of(0.0);
    public LinearVelocity DriveVelocity = MetersPerSecond.of(0.0);
    public boolean TurnConnected = false;
    public Angle TurnPosition = Radians.of(0.0);
    public AngularVelocity TurnVelocity = RadiansPerSecond.of(0.0);

    @Override
    public ModuleState update(ModuleState newState) {
        this.DriveConnected = newState.DriveConnected;
        this.DrivePosition = newState.DrivePosition;
        this.DriveVelocity = newState.DriveVelocity;
        this.TurnConnected = newState.TurnConnected;
        this.TurnPosition = newState.TurnPosition;
        this.TurnVelocity = newState.TurnVelocity;
        return this;
    }

    public ModuleStateAutoLogged toAutoLogged() {
        ModuleStateAutoLogged result = new ModuleStateAutoLogged();
        result.update(this);
        return result;
    }
}
