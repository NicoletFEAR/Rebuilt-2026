package frc.robot.subsystems.led;

// import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.LEDConstants;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.EmptyAnimation;
// import com.ctre.phoenix6.controls.ColorFlowAnimation;
// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.FireAnimation;
// import com.ctre.phoenix6.controls.TwinkleAnimation;
// import com.ctre.phoenix6.controls.TwinkleOffAnimation;
// import com.ctre.phoenix6.controls.LarsonAnimation;
// import com.ctre.phoenix6.controls.RgbFadeAnimation;
// import com.ctre.phoenix6.controls.SingleFadeAnimation;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;


public class Led extends SubsystemBase {

    // color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex
    public static final RGBWColor kGreen = new RGBWColor(Color.kGreen);
    public static final RGBWColor kWhite = new RGBWColor(Color.kWhite);
    public static final RGBWColor kRed = new RGBWColor(Color.kRed);
    public static final RGBWColor kTeal = new RGBWColor(Color.kTeal);
    public static final RGBWColor kYellow = new RGBWColor(Color.kYellow);
    public static final RGBWColor kBlue = new RGBWColor(Color.kBlue);

    // private CANdle m_candle;
    private LedState m_state = LedState.OFF;

    // /*
    //  * Start and end index for LED animations.
    //  * 0-7 are onboard, 8-399 are an external strip.
    //  * CANdle supports 8 animation slots (0-7).
    //  */
    // private static final int kSlot0StartIdx = LEDConstants.getSlotStart(0);
    // private static final int kSlot0EndIdx = LEDConstants.getSlotEnd(0);

    /**
     * The type of animation to play for the LEDs
     */
    public enum AnimationType {
        None,
        Solid,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    /**
     * Create a new LED and supply the subsystems that will be used to control the LEDs
     *
     * @param launcher The launcher subsystem
     * @param climb    The climb subsystem
     * @param candleId The ID of the CANdle
     */
    public Led(int candleId) {
        // // Create the candle and set it on the right bus
        // this.m_candle = new CANdle(candleId, new CANBus(Constants.hasCANivore() ? "*" : "rio"));

        // // Configure CANdle
        // CANdleConfiguration cfg = new CANdleConfiguration();
        // cfg.LED.BrightnessScalar = 0.5;

        // // disable status LED when being controlled
        // cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        // // Apply the config
        // m_candle.getConfigurator().apply(cfg);

        // // Clear out previous animations
        // clearAnimations();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/State", m_state);
    }

    @Override
    public void simulationPeriodic() {
        // TODO This may not be needed
    }

    /**
     * Clear out all previous animations
     */
    public void clearAnimations() {
        for (int i = 0; i < 8; ++i) {
            // m_candle.setControl(new EmptyAnimation(i));
        }
        applyAnimation(0, AnimationType.None, null);
    }

    public Command startLaunchAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.Fire, kTeal));
    }

    public Command startIntakeAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.Fire, kYellow));
    }

    public Command startClimbAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.SingleFade, kBlue));
    }

    public Command startAutoAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.Rainbow, kRed));
    }

    public Command startSwerveAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.ColorFlow, kWhite));
    }

    public Command startScoringSwitchAnimation() {
        // TODO Add the concept of duration
        return new InstantCommand(() -> applyAnimation(0, AnimationType.Strobe, kRed));
    }

    public Command stopAnimation() {
        return new InstantCommand(() -> applyAnimation(0, AnimationType.None, null));
    }

    /**
     * This method will apply an animation for the selected slot and color
     * @param slot The slot to apply the animation to
     * @param animationType The animation to apply
     * @param color The color to use for the animation
     */
    public void applyAnimation(int slot, AnimationType animationType, RGBWColor color) {

        // // if the selection for slot 0 changes, change animations
        // switch (animationType) {
        //     case ColorFlow:
        //         m_candle.setControl(
        //             new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case Rainbow:
        //         m_candle.setControl(new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot));
        //         return;

        //     case Twinkle:
        //         m_candle.setControl(
        //             new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case TwinkleOff:
        //         m_candle.setControl(
        //             new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case Fire:
        //         m_candle.setControl(new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot));
        //         return;

        //     case Larson:
        //         m_candle.setControl(
        //             new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case RgbFade:
        //         m_candle.setControl(new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot));
        //         return;

        //     case SingleFade:
        //         m_candle.setControl(
        //             new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case Strobe:
        //         m_candle.setControl(
        //             new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(slot).withColor(color));
        //         return;

        //     case Solid:
        //         m_candle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(color));
        //         return;

        //     case None:
        //     default:
        //         m_candle.setControl(new EmptyAnimation(slot));
        //         return;

        // }
    }
}
