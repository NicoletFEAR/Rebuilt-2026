package frc.robot.util.constraint;

public enum ConstraintType {
    IGNORE("Ignore"),
    MATCH("Match"),
    ;

    private final String m_name;

    ConstraintType(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }
}
