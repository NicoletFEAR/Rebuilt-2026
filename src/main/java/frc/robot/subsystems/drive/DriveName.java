package frc.robot.subsystems.drive;

public enum DriveName {
    DRIVE("Drive"),
    ;

    private final String name;

    DriveName(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
