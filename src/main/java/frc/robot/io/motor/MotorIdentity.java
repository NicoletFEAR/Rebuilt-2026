package frc.robot.io.motor;

public enum MotorIdentity {
    NONE("None"),
    SPARK_MAX("SparkMAX"),
    SPARK_MAX_SIMULATED("SparkMAX Simulated"),
    TALON_FX("TalonFX"),
    TALON_FX_SIMULATED("TalonFX Simulated"),
    ;

    private final String name;

    MotorIdentity(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
