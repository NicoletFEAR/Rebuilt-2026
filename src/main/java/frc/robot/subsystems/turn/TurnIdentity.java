package frc.robot.subsystems.turn;

public enum TurnIdentity {
    NONE("None"),
    SPARK_MAX("SparkMax"),
    SPARK_MAX_SIMULATED("SparkMax Simulated"),
    ;

    private final String m_name;

    TurnIdentity(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }
}
