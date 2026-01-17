// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Shooter m_shooter;

    public RobotContainer() {
        m_shooter = new Shooter();

        configureBindings();
    }

    private void configureBindings() {
        joystick.rightBumper().onTrue(m_shooter.setVelocity(45));
        joystick.rightBumper().onFalse(m_shooter.setOff());
    }


}
