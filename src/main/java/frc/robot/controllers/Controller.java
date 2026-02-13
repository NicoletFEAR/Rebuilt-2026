package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
    public Trigger L1();
    public Trigger L2();
    public Trigger R1();
    public Trigger R2();
    public Trigger topLeft();
    public Trigger topRight();
    public Trigger up();
    public Trigger down();
    public Trigger left();
    public Trigger right();
}
