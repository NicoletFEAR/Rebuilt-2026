package frc.robot.controllers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Class used to abstract all controller types into one universal class.  This will make
 * use the of the CommandPS5Controller, CommandPS4Controller, and CommandXboxController classes
 * so that other systems won't have to be coded for a specific controller.  The code will assume
 * that all controllers use PS5 buttons.  In cases where a controller has a different button id or
 * name for the same thing, the PS5 equivalent button method can be called.
 */
public class UniversalController {

    private int port;
    private ControllerType controllerType;
    private CommandPS4Controller ps4Controller;
    private CommandPS5Controller ps5Controller;
    private CommandXboxController xboxController;
    private Axis controllerAxis;
    private Button controllerButton;
    private Alert alert = new Alert("Unsupported Controller Type: " + controllerType, AlertType.kError);
    
    /**
     * Create a new instance
     * @param port The port for the controller on the laptop
     * @param controllerType The type of controller, aligns to a ControllerType
     */
    public UniversalController(int port, ControllerType controllerType) {
        this.port = port;
        this.controllerType = controllerType;

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
                sendAlert("supplied controller type: " + controllerType);
        }

        this.controllerAxis = new Axis(controllerType);
        this.controllerButton = new Button(controllerType);
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
                sendAlert("getRawAxis");
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
                sendAlert("button create");
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
                sendAlert("button L1");
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
                sendAlert("button L2");
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
                sendAlert("button options");
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
                sendAlert("button R1");
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
                sendAlert("button R2");
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
                sendAlert("povDown");
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
                sendAlert("povUp");
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
                sendAlert("povLeft");
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
                sendAlert("povRight");
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
                sendAlert("button triangle");
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
                sendAlert("button circle");
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
                sendAlert("button square");
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
                sendAlert("button x");
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
                sendAlert("button PS");
                return null;
        }
    }

    public Trigger button(int button) {
        switch (controllerType) {
            case PS4:
                return ps4Controller.button(button);
            case PS5:
                return ps5Controller.button(button);
            case XBOX:
                return xboxController.button(button);        
            default:
                sendAlert("button " + button);
                return null;
        }
    }

    public boolean isConnected() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.isConnected();
            case PS5:
                return ps5Controller.isConnected();
            case XBOX:
                return xboxController.isConnected();
            default:
                sendAlert("isConnected");
                return false;
        }
    }

    public Trigger touchpad() {
        switch (controllerType) {
            case PS4:
                return ps4Controller.touchpad();
            case PS5:
                return ps5Controller.touchpad();
            default:
                sendAlert("button touchpad");
                return null;
        }
    }

    public Trigger leftStick() {
        switch (controllerType) {
            case XBOX:
                return xboxController.leftStick();
            default:
                sendAlert("button leftStick");
                return null;
        }
    }

    public Trigger rightStick() {
        switch (controllerType) {
            case XBOX:
                return xboxController.rightStick();
            default:
                sendAlert("button rightStick");
                return null;
        }
    }

    public void setRumble(RumbleType type, double value) {
        switch(controllerType) {
            case PS4:
                ps4Controller.setRumble(type, value);
                break;
            case PS5:
                ps5Controller.setRumble(type, value);
                break;
            case XBOX:
                xboxController.setRumble(type, value);
                break;
            default:
                sendAlert("setRumble");
        }
    }

    /**
     * This method will return the underlying Command Controller. It is up to 
     * the caller to cast it to the correct type.
     * @return The underlying command controller
     */
    public CommandGenericHID getCommandController() {
        switch (controllerType) {
            case PS4:
                return ps4Controller;
            case PS5:
                return ps5Controller;
            case XBOX:
                return xboxController;
            default:
                sendAlert("getCommandController");
                return null;
        }
    }

    /**
     * Send an alert and send message to console
     * @param message The message to output
     */
    private void sendAlert(String message) {
        sendAlert(message, AlertType.kError);
    }

    /**
     * Send an alert including a custom messages and send the message to the console
     * @param message  The message to send in the alert
     * @param alertType The type of alert
     */
    private void sendAlert(String message, AlertType alertType) {
        System.out.println("Unsupported Controller Type: " + controllerType + " " + message);

        // An issue occurred, set the alert as active
        alert.set(true);
    }
}
