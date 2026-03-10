package frc.robot.subsystems.timing;

import java.util.Optional;

public class TimingIONone implements TimingIO {
    @Override
    public void refreshData() {}

    @Override
    public void updateInputs(TimingIOInputs inputs) {
        inputs.AutoWinner = Optional.empty();
        inputs.TimeSinceMatchStart = Optional.empty();
    }
}
