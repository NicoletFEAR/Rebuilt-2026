package frc.lib.architecture;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateSubsystem<T> extends SubsystemBase {
    protected abstract T updateState();
    protected abstract void applyState();
}
