package frc.robot.controllers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Class used to abstract all controller types into one universal class.  This will make
 * use the of the CommandPS5Controller, CommandPS4Controller, and CommandXboxController classes
 * so that other systems won't have to be coded for a specific controller.
 */
public class UniversalController {

    private int port;
    private ControllerType controllerType;
    private CommandPS4Controller ps4Controller;
    private CommandPS5Controller ps5Controller;
    private CommandXboxController xboxController;
    private Axis controllerAxis;
    private Button controllerButton;
    
    /**
     * Create a new instance
     * @param port The port for the controller on the laptop
     * @param controllerType The type of controller, aligns to a ControllerType
     */
    public UniversalController(int port, ControllerType controllerType) {
        super();
        this.port = port;
        this.controllerType = controllerType;

        try {
            switch (controllerType) {

                case PS4:
                    this.ps4Controller = new CommandPS4Controller(port);
                    break;

                case PS5:
                    this.ps5Controller = new CommandPS5Controller(port);
                    break;

                case XBOX:
                    this.xboxController = new CommandXboxController(port);
                    break;

                default:
                    System.out.println("Unsupported Controller Type: " + controllerType);
                    new Alert("Unsupported Controller Type: " + controllerType, AlertType.kError);
            }

            this.controllerAxis = new Axis(controllerType);
            this.controllerButton = new Button(controllerType);
        }
        catch (Exception e) {
            // For now, dump an error message to system out.  In the future a better 
            // path for logging should be used
            System.out.println("Error creating controller: " + e.getMessage());
        }
    }

    public ControllerType getType() {
        return controllerType;
    }

    public int getPort() { return port; }

    public Axis getAxis() {
        return controllerAxis;
    }

    public Button getButton() {
        return controllerButton;
    }

    public double getRawAxis(int axis) {
        switch (controllerType) {
            case PS4:
                return ps4Controller.getRawAxis(axis);
            case PS5:
                return ps5Controller.getRawAxis(axis);
            case XBOX:
                return xboxController.getRawAxis(axis);
            default:
                System.out.println("Unsupported Controller Type, Axis: " + axis + " for type:" + controllerType);
                new Alert("Unsupported Controller Type, Axis: " + axis + " for type:" + controllerType, AlertType.kError);
                return 0D;
        }
    }

    public Trigger create() {
        switch (controllerType) {
            case PS5:
                return ps5Controller.create();
            case PS4: 
                return ps4Controller.share();
            case XBOX:
                return xboxController.back();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType);
                new Alert("Unsupported Controller Type:" + controllerType, AlertType.kError);
                return null;
        }
    }

    public Trigger L1() { 
        switch (controllerType) {
            case PS4:
                return ps4Controller.L1();
            case PS5:
                return ps5Controller.L1();
            case XBOX:
                return xboxController.leftBumper();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button L1");
                new Alert("Unsupported Controller Type:" + controllerType + ", button L1", AlertType.kError);
                return null;
        }
    }

    public Trigger L2() { 
        switch (controllerType) {
            case PS4:
                return ps4Controller.L2();
            case PS5:
                return ps5Controller.L2();
            case XBOX:
                return xboxController.leftTrigger();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button L2");
                new Alert("Unsupported Controller Type:" + controllerType + ", button L2", AlertType.kError);
                return null;
        }
    }

    public Trigger options() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.options();
            case PS5:
                return ps5Controller.options();
            case XBOX:
                return xboxController.start();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button options");
                new Alert("Unsupported Controller Type:" + controllerType + ", button options", AlertType.kError);
                return null;
        }
    }

    public Trigger R1() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.R1();
            case PS5:
                return ps5Controller.R1();
            case XBOX:
                return xboxController.rightBumper();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button R1");
                new Alert("Unsupported Controller Type:" + controllerType + ", button R1", AlertType.kError);
                return null;
        }
    }

    public Trigger R2() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.R2();
            case PS5:
                return ps5Controller.R2();
            case XBOX:
                return xboxController.rightTrigger();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button R2");
                new Alert("Unsupported Controller Type:" + controllerType + ", button R2", AlertType.kError);
                return null;
        }
    }

    public Trigger povDown() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.povDown();
            case PS5:
                return ps5Controller.povDown();
            case XBOX:
                return xboxController.povDown();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", povDown");
                new Alert("Unsupported Controller Type:" + controllerType + ", povDown", AlertType.kError);
                return null;
        }
    }

    public Trigger povUp() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.povUp();
            case PS5:
                return ps5Controller.povUp();
            case XBOX:
                return xboxController.povUp();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", povUp");
                new Alert("Unsupported Controller Type:" + controllerType + ", povUp", AlertType.kError);
                return null;
        }
    }

    public Trigger povLeft() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.povLeft();
            case PS5:
                return ps5Controller.povLeft();
            case XBOX:
                return xboxController.povLeft();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", povLeft");
                new Alert("Unsupported Controller Type:" + controllerType + ", povLeft", AlertType.kError);
                return null;
        }
    }

    public Trigger povRight() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.povRight();
            case PS5:
                return ps5Controller.povRight();
            case XBOX:
                return xboxController.povRight();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", povRight");
                new Alert("Unsupported Controller Type:" + controllerType + ", povRight", AlertType.kError);
                return null;
        }
    }

    public Trigger triangle() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.triangle();
            case PS5:
                return ps5Controller.triangle();
            case XBOX:
                return xboxController.y();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button triangle");
                new Alert("Unsupported Controller Type:" + controllerType + ", button triangle", AlertType.kError);
                return null;
        }
    }

    public Trigger circle() {
        switch(controllerType) {
            case PS4:
                return ps4Controller.circle();
            case PS5:
                return ps5Controller.circle();
            case XBOX:
                return xboxController.b();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button circle");
                new Alert("Unsupported Controller Type:" + controllerType + ", button circle", AlertType.kError);
                return null;
        }
    }

    public Trigger square() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.square();
            case PS5:
                return ps5Controller.square();
            case XBOX:
                return xboxController.x();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button square");
                new Alert("Unsupported Controller Type:" + controllerType + ", button square", AlertType.kError);
                return null;
        }
    }

    public Trigger cross() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.cross();
            case PS5:
                return ps5Controller.cross();
            case XBOX:
                return xboxController.a();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button x");
                new Alert("Unsupported Controller Type:" + controllerType + ", button x", AlertType.kError);
                return null;
        }
    }

    public Trigger PS() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.PS();
            case PS5:
                return ps5Controller.PS();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button PS");
                new Alert("Unsupported Controller Type:" + controllerType + ", button PS", AlertType.kError);
                return null;
        }
    }

    public Trigger touchpad() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.touchpad();
            case PS5:
                return ps5Controller.touchpad();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button touchpad");
                new Alert("Unsupported Controller Type:" + controllerType + ", button touchpad", AlertType.kError);
                return null;
        }
    }

    public Trigger leftStick() {
        switch (controllerType) {
            case XBOX:
                return xboxController.leftStick();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button leftStick");
                new Alert("Unsupported Controller Type:" + controllerType + ", button leftStick", AlertType.kError);
                return null;
        }
    }

    public Trigger rightStick() {
        switch (controllerType) {
            case XBOX:
                return xboxController.rightStick();
            default:
                System.out.println("Unsupported Controller Type:" + controllerType + ", button rightStick");
                new Alert("Unsupported Controller Type:" + controllerType + ", button rightStick", AlertType.kError);
                return null;
        }
    }
}
