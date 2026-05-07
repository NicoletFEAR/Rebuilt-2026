package frc.robot.io.encoder;

public enum EncoderIdentity {
    NONE("None"),
    CAN_CODER("CANcoder"),
    CAN_CODER_SIMULATED("CANcoder Simulated"),
    ;

    private final String name;

    EncoderIdentity(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
