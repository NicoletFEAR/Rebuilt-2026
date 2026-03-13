package frc.robot.data.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.architecture.SubsystemIO;

public interface ControllerIO extends SubsystemIO<ControllerIO.ControllerIOInputs> {
    @AutoLog
    public static class ControllerIOInputs {
        public boolean Circle;
        public boolean Create;
        public boolean Cross;
        public boolean Down;
        public boolean LeftBumper;
        public double LeftTrigger;
        public boolean LeftStick;
        public boolean Left;
        public double LeftX;
        public double LeftY;
        public boolean Options;
        public boolean PlayStation;
        public boolean RightBumper;
        public double RightTrigger;
        public boolean RightStick;
        public boolean Right;
        public double RightX;
        public double RightY;
        public boolean Square;
        public boolean Touchpad;
        public boolean Triangle;
        public boolean Up;
    }
    
    void setRumble(double strength);
}
