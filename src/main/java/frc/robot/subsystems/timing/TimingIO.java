package frc.robot.subsystems.timing;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.architecture.SubsystemIO;

public interface TimingIO extends SubsystemIO<TimingIO.TimingIOInputs> {
    @AutoLog
    public static class TimingIOInputs {
        public Optional<Alliance> AutoWinner;
        public Optional<Measure<TimeUnit>> TimeSinceMatchStart;
        public Measure<TimeUnit> TimeSinceRobotOn;
    }
}
