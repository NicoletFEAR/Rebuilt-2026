package frc.robot.robots.base;

public abstract class RobotState<T extends RobotState<T>> {
    public abstract T update(T newState);
}
