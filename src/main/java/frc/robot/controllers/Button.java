package frc.robot.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;

public class Button {

    private ControllerType type;

    public Button(ControllerType type) {
        this.type = type;
    }

    public int getValue(String buttonName) {
        switch (type) {
            case PS4:
                return PS4Controller.Button.valueOf(buttonName).value;
            case PS5:
                return PS5Controller.Button.valueOf(buttonName).value;
            case XBOX:
                return XboxController.Button.valueOf(buttonName).value;
            default:
                throw new RuntimeException("Unsupported Controller Type: " + type);
        }
    }
}
