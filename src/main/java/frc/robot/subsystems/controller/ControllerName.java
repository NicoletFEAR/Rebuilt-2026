package frc.robot.subsystems.controller;

public enum ControllerName {
    DRIVER("Driver"),
    OPERATOR("Operator"),
    ;

    private final String m_name;

    ControllerName(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }
}
