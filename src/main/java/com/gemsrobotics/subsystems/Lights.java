package com.gemsrobotics.subsystems;

import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;

public class Lights extends SubsystemBase{

    private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private final CANdle m_candle = new CANdle(0);

    private static final int LED_COUNT = 100;

    private enum Animation {

        NONE(new EmptyAnimation(0)),
        OUT_RANGE(new SolidColor(0, LED_COUNT - 1).withColor(kYellow)),
        IN_RANGE(new SolidColor(0, LED_COUNT - 1).withColor(kGreen)),
        JAMMED(new StrobeAnimation(0, LED_COUNT - 1).withColor(kRed).withFrameRate(10)),;

        private ControlRequest anim;

        Animation(ControlRequest anim) {
            this.anim = anim;
        }

        public ControlRequest getAnim() {
            return anim;
        }

    }

    private Animation m_anim0State = Animation.NONE;

    //private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();

    public Lights() {
        var cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        m_candle.setControl(Animation.NONE.getAnim());

        //m_anim0Chooser.setDefaultOption("Single Fade", AnimationType.None);

        //SmartDashboard.putData("Animation 0", m_anim0Chooser);
    }

    public Command setOff() {
        return run(() -> m_candle.setControl(Animation.NONE.getAnim()));
    }

    public Command setOutOfRange() {
        return run(() -> m_candle.setControl(Animation.OUT_RANGE.getAnim()));
    }

    public Command setWithinRange() {
        return run(() -> m_candle.setControl(Animation.IN_RANGE.getAnim()));
    }

    public Command setJammed() {
        return run(() -> m_candle.setControl(Animation.JAMMED.getAnim()));
    }








}
