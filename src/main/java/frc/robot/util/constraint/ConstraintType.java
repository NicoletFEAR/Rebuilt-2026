package frc.robot.util.constraint;

public enum ConstraintType {
    IGNORE("Ignore"),
    MATCH("Match"),
    ;

    private final String name;

    ConstraintType(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
