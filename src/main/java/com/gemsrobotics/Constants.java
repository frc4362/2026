// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.CANBus;
import com.gemsrobotics.subsystems.swerve.TunerConstants;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Distance BUMPER_DEPTH = Inches.of(3.5);

  public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CAN {
    public static final CANBus kMAIN_BUS = new CANBus("main");
    public static final CANBus kAUX_BUS = new CANBus("aux");

    public static final int SWERVE_IMU = 0; // Pigeon 2

    public static final int SWERVE_NW_DRIVE = 0;
    public static final int SWERVE_NE_DRIVE = 1;
    public static final int SWERVE_SW_DRIVE = 2;
    public static final int SWERVE_SE_DRIVE = 3;

    public static final int SWERVE_NW_AZIMUTH = 4;
    public static final int SWERVE_NE_AZIMUTH = 5;
    public static final int SWERVE_SW_AZIMUTH = 6;
    public static final int SWERVE_SE_AZIMUTH = 7;

    public static final int SWERVE_NW_ENCODER = 8;
    public static final int SWERVE_NE_ENCODER = 9;
    public static final int SWERVE_SW_ENCODER = 10;
    public static final int SWERVE_SE_ENCODER = 11;

    //--------------------aux-bus---------------------------

    public static final int INTAKE_TOP_TRANSLATION = 30;
    public static final int INTAKE_BOTTOM_TRANSLATION = 31;
    public static final int INTAKE_DEPLOYER = 32;
    public static final int SHOOTER_WEST = 33;
    public static final int SHOOTER_EAST = 34;
    public static final int HOPPER_NORTH = 35;
    public static final int HOPPER_SOUTH = 36;
    public static final int UPTAKE_LEADER = 37; // TODO: rename to give spatial information
    public static final int UPTAKE_FOLLOWER = 38; // TODO: rename to give spatial information
    public static final int HOOD = 39;
  }
}
