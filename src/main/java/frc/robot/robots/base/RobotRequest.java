package frc.robot.robots.base;

public abstract class RobotRequest<T extends RobotState<T>, U extends RobotRequest<T, U>> {
    public abstract U update(T newState);
    public abstract U update(U newRequest);
}
