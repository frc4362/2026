package com.gemsrobotics.subsystems;

import com.ctre.phoenix6.signals.*;

import com.gemsrobotics.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;

public class Lights extends SubsystemBase {

    private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private final CANdle m_candle;

    private static final int LED_COUNT = 30;

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

    private Animation m_animState = Animation.NONE;
    private final StringPublisher m_animStatePublisher;

    public Lights() {
        m_candle = new CANdle(0, Constants.CAN.kAUX_BUS);

        var cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Lights");
        m_animStatePublisher = nt.getStringTopic("animation state").publish();

        m_animState = Animation.NONE;
        m_animStatePublisher.set(Animation.NONE.name());
        m_candle.setControl(Animation.NONE.getAnim());

        //m_anim0Chooser.setDefaultOption("Single Fade", AnimationType.None);

        //SmartDashboard.putData("Animation 0", m_anim0Chooser);
    }

    private Command set(Animation a) {
        return run(() -> {
            m_animState = a;
            m_animStatePublisher.set(a.name());
            m_candle.setControl(a.getAnim());
        });
    }

    public Command setOff() {
        return set(Animation.NONE);
    }

    public Command setOutOfRange() {
        return set(Animation.OUT_RANGE);
    }

    public Command setWithinRange() {
        return set(Animation.IN_RANGE);
    }

    public Command setJammed() {
        return set(Animation.JAMMED);
    }
}
