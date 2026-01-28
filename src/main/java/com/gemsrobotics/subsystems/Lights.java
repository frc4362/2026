package com.gemsrobotics.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Lights extends SubsystemBase{

    private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private final CANdle m_candle = new CANdle(0);

    private enum AnimationType {
        None,
        SingleFade,
    }

    private AnimationType m_anim0State = AnimationType.None;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();

    public Lights() {
        var cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }

        m_candle.setControl(new SolidColor(0, 7).withColor(kRed));

        m_anim0Chooser.setDefaultOption("Single Fade", AnimationType.None);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
    }

//    @Override
//    public void () {
//        final var m_anim0Selection = m_anim0Chooser.getSelected();
//        if (m_anim0State != m_anim0Selection) {
//            m_anim0State = m_anim0Selection;
//
//            switch (m_anim0State) {
//                default:
//                case SingleFade:
//                    m_candle.setControl(
//                        new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(kRed)
//                    );
//                    break;
//
//            }
//        }
//
//    }

}
