package frc.lib.architecture;

public interface SubsystemIO<T> {
    void refreshInputs();
    void updateInputs(T inputs);
}
