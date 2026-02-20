package frc.robot.subsystems.led;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.launcher.Launcher;

public class Led extends SubsystemBase {

    private CANdle candle;
    private Launcher launcher;
    private Climb climb;

    /**
     * Create a new LED and supply the subsystems that will be used to control the LEDs
     *
     * @param launcher The launcher subsystem
     * @param climb    The climb subsystem
     * @param candleId The ID of the CANdle
     */
    public Led(Launcher launcher, Climb climb, int candleId) {
        this.launcher = launcher;
        this.climb = climb;
        this.candle = new CANdle(candleId);
    }

    @Override
    public void periodic() {
    }

}
