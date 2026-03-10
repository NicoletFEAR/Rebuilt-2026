package frc.lib.architecture;

/**
 * Represents the inputs and outputs for a subsystem.
 * 
 * Placing the inputs and outputs for a subsystem in a separate class from the
 * subsystem itself is beneficial for several reasons. For one, the inputs can be
 * updated at regular intervals, fixing the non-determinism of CTRE status signals and
 * ensuring deterministic replay of log files. Additionally, this allows hardware to be
 * easily interchangeable, since the code that interacts with the hardware is separated
 * from the logic of the subsystem.
 * <p>
 * This interface shouldn't be implemented directly; instead, it should be extended by
 * a different interface made specifically for a certain subsystem, and then that
 * interface can be implemented by several classes. For example, a {@code LauncherIO}
 * interface that extends this one might be created, and then that interface could be
 * implemented as {@code LauncherIO1Talon} and {@code LauncherIO2Talons}, both of which
 * could be used interchangeably in a {@code LauncherSubsystem} class.
 * <p>
 * When extending, a nested class should be created with the same name as the class
 * except for {@code Inputs} appended. It should have a field for every value the
 * subsystem might need to access, and can then be automatically logged to
 * AdvantageScope with the {@link org.littletonrobotics.junction.AutoLog AutoLog}
 * annotation. This generates a class with the same name as the inputs class except
 * with {@code AutoLogged} appended, and this class can be accessed from other files
 * without needing any import statements.
 * 
 * @param <T>  the associated inputs class
 */
public interface SubsystemIO<T> {
    /**
     * Updates the fields with the latest data from the robot.
     * 
     * Along with {@link #updateInputs(T)}, can be run in a separate thread to ensure
     * that if waiting for sensors takes too long, it doesn't cause a loop overrun.
     * Should be called before {@code updateInputs(T)}, or else inputs will receive old
     * data.
     */
    void refreshData();
    
    /**
     * Updates the data of an inputs object.
     * 
     * Replaces the values of the fields of the inputs object with the
     * {@code SubsystemIO}'s own values. Along with {@link #refreshData()}, can be run
     * in a separate thread to ensure that if waiting for sensors takes too long, it
     * doesn't cause a loop overrun. Should be called after {@code refreshData()}, or
     * else inputs will be updated with old data.
     * 
     * @param inputs the inputs object to update
     */
    void updateInputs(T inputs);
}
