package frc.robot.subsystems.timing;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class TimingIOFMS implements TimingIO {
    private Optional<Alliance> m_autoWinner;
    private double m_matchTime;

    @Override
    public void refreshData() {
        if (m_autoWinner.isEmpty()) {
            String data = DriverStation.getGameSpecificMessage();

            if (data.length() > 0){
                m_autoWinner = switch (data.charAt(0)) {
                    case 'R': {
                        yield Optional.of(Alliance.Red);
                    }
                    case 'B': {
                        yield Optional.of(Alliance.Blue);
                    }
                    default: {
                        yield Optional.empty();
                    }
                };
            }
        }

        m_matchTime = DriverStation.getMatchTime();
    }

    @Override
    public void updateInputs(TimingIOInputs inputs) {
        inputs.AutoWinner = m_autoWinner;
        inputs.TimeSinceMatchStart = Math.abs(m_matchTime + 1.0) < Constants.kFloatingPointEqualityTolerance
            ? Optional.empty()
            : DriverStation.isAutonomous()
            ? Optional.of(Seconds.of(20.0 - m_matchTime))
            : DriverStation.isTeleopEnabled()
            ? Optional.of(Seconds.of(160.0 - m_matchTime))
            : Optional.empty();
    }
}
