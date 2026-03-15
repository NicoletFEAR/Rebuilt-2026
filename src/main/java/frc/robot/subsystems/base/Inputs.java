package frc.robot.subsystems.base;

public abstract class Inputs<T extends State<T>> {
    public abstract T updateState();
}
