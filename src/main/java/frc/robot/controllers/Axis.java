package frc.robot.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;

public class Axis {

    private ControllerType type;

    public Axis(ControllerType type) {
        this.type = type;
    }

    public int getAxis(String axString) {
        switch (type) {
            case PS4:
                return PS4Controller.Axis.valueOf(axString).value;
            case PS5:
                return PS5Controller.Axis.valueOf(axString).value;
            case XBOX:
                return XboxController.Axis.valueOf(axString).value;
            default:
                throw new RuntimeException("Unsupported Controller Type: " + type);
        }
    }
}
