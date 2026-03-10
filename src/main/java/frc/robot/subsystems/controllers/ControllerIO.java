package frc.robot.subsystems.controllers;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.architecture.SubsystemIO;

public interface ControllerIO extends SubsystemIO<ControllerIO.ControllerIOInputs> {
    @AutoLog
    public static class ControllerIOInputs {
        public boolean Circle;
        public boolean Create;
        public boolean Cross;
        public boolean Down;
        public boolean L1;
        public double L2;
        public boolean L3;
        public boolean Left;
        public double LeftX;
        public double LeftY;
        public boolean Options;
        public boolean PlayStation;
        public boolean R1;
        public double R2;
        public boolean R3;
        public boolean Right;
        public double RightX;
        public double RightY;
        public boolean Square;
        public boolean Touchpad;
        public boolean Triangle;
        public boolean Up;
    }

    void setLeftRumble(double strength);
    void setRightRumble(double strength);
    void setRumble(double strength);
}
