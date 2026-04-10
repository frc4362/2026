// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.gemsrobotics;

import com.ctre.phoenix6.CANBus;
import com.gemsrobotics.lib.math.Translation2dPlus;
import com.gemsrobotics.subsystems.superstructure.Intake;
import com.gemsrobotics.subsystems.swerve.SwerveConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.sqrt;

public final class Constants {
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final double PHASE_LAG_SECONDS = 0.03;

  public static final double LAUNCH_TIME_AFTER_ACTIVE = 1.0;
  public static final double LAUNCH_TIME_BEFORE_ACTIVE = 1.5 + LAUNCH_TIME_AFTER_ACTIVE;
  public static final int TOF_RECURSION_LIMIT = 10;

  public static final Distance BUMPER_DEPTH = Inches.of(3.5);

  // robot is square
  public static final Distance ROBOT_WIDTH = Inches.of(27.5);
  public static final Distance ROBOT_RADIUS = SwerveConstants.moduleTranslations[0].getMeasureX().times(sqrt(2));
  public static final Distance INTAKE_WIDTH = Inches.of(26.0);
  public static final Distance INTAKE_REACH = Inches.of(10.5);
  public static final Transform2d INTAKE_CORNER_NW = new Transform2d(ROBOT_WIDTH.div(2).plus(INTAKE_REACH), INTAKE_WIDTH.div(2), Rotation2d.kZero);
  public static final Transform2d INTAKE_CORNER_NE = new Transform2d(ROBOT_WIDTH.div(2).plus(INTAKE_REACH), INTAKE_WIDTH.div(2).unaryMinus(), Rotation2d.kZero);
  public static final Distance INTAKE_ROLLER_RADIUS = Inches.of(1.32).div(2);
  public static final double INTAKE_MAX_VELOCITY = DCMotor.getKrakenX60(2).withReduction(Intake.TRANSLATION_GEARING).freeSpeedRadPerSec * INTAKE_ROLLER_RADIUS.in(Meters);

  public static final double MAX_SPEED = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  public static final double MAX_ACCELERATION = MAX_SPEED * 0.75;
  public static final Mass ROBOT_WEIGHT = Pounds.of(115.0 + 20.0 + 13.0);
  public static final Distance BALL_RADIUS = Inches.of(5.91).div(2.0);
  public static final Distance ROBOT_TO_LAUNCHER_WEST_Y = Inches.of(4.77).plus(Inches.of(1.5));
  public static final Distance ROBOT_TO_LAUNCHER_EAST_Y = ROBOT_TO_LAUNCHER_WEST_Y.unaryMinus();
//  public static final Distance ROBOT_TO_LAUNCHER_X = Inches.of(-3.231).minus(Inches.of(2.0)).minus(Inches.of(4.964 / 2.0));
  public static final Distance ROBOT_TO_LAUNCHER_X = Inches.of(-2.96);
//  public static final Distance ROBOT_TO_LAUNCHER_Z = Inches.of(22.826);
  public static final Distance ROBOT_TO_LAUNCHER_Z = Inches.of(23.008);
  public static final Transform3d ROBOT_TO_LAUNCHER_WEST = new Transform3d(ROBOT_TO_LAUNCHER_X, ROBOT_TO_LAUNCHER_WEST_Y, ROBOT_TO_LAUNCHER_Z, new Rotation3d());
  public static final Transform3d ROBOT_TO_LAUNCHER_EAST = new Transform3d(ROBOT_TO_LAUNCHER_X, ROBOT_TO_LAUNCHER_EAST_Y, ROBOT_TO_LAUNCHER_Z, new Rotation3d());
  public static final Transform2d ROBOT_TO_LAUNCHER;
  static {
    final Transform3d robotToLauncher3d = ROBOT_TO_LAUNCHER_WEST.plus(ROBOT_TO_LAUNCHER_EAST).div(2.0);
    ROBOT_TO_LAUNCHER = new Transform2d(
            robotToLauncher3d.getX(),
            robotToLauncher3d.getY(),
            Rotation2d.fromDegrees(180));
  }
  public static final Distance BALL_STREAM_WIDTH = ROBOT_TO_LAUNCHER_WEST_Y.times(2.0).plus(BALL_RADIUS.times(2.0));

  public static final String SIM_VIZ_TABLE_KEY = "viz";

  public static class OperatorConstants {
    public static final int kPilotControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }

  public static class Vision {
    public static final boolean ACCEPT_VISION_MEASUREMENTS = true;
    public static final double ACCEPTABLE_AMBIGUITY = 0.2;
    public static final double YAW_LOOKBACK_SECONDS = 0.1;
    public static final AngularVelocity HIGH_YAW_RATE = RadiansPerSecond.of(4);

    public static final String LIMELIGHT_LAUNCHER_NAME = "limelight-launch";
    public static final String LIMELIGHT_CLIMBER_NAME = "limelight-climb";

//    public static final Transform3d LIMELIGHT_LAUNCHER_TRANSFORM = new Transform3d(
//            new Translation3d(Inches.of(-11.14), Inches.of(0.0), Inches.of(28.97)),
//            new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(0.0)));
    // we flipped the launcher lol
    public static final Transform3d LIMELIGHT_LAUNCHER_TRANSFORM = new Transform3d(
            new Translation3d(Inches.of(-2.96), Inches.of(0.25), Inches.of(28.151)),
            new Rotation3d(Degrees.of(0.0), Degrees.of(10), Degrees.of(-180.0)));

    // lol
    public static final Transform3d LIMELIGHT_CLIMBER_TRANSFORM = new Transform3d();

    public static final double HIGH_VARIANCE = 1_000_000.0;
    // skips N frames and then processes one
    public static final int DISABLED_THROTTLE = 0;
    // how fast the Limelight IMU filter converges on the Pigeon, default 0.001
    public static final double IMU_ALPHA = 0.005;

    // Standard deviation constants
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;

    // Standard deviation array indices for Megatag2
    public static final int kMegatag2XStdDevIndex = 6;
    public static final int kMegatag2YStdDevIndex = 7;
    public static final int kMegatag2YawStdDevIndex = 11;
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

    public static final Angle SWERVE_NW_ENCODER_OFFSET = Rotations.of(0.456787109375);
    public static final Angle SWERVE_NE_ENCODER_OFFSET = Rotations.of(-0.46630859375);
    public static final Angle SWERVE_SW_ENCODER_OFFSET = Rotations.of(-0.45703125);
    public static final Angle SWERVE_SE_ENCODER_OFFSET = Rotations.of(-0.374267578125);

    //--------------------aux-bus---------------------------

    public static final int INTAKE_TRANSLATION_LEADER = 31;
    public static final int INTAKE_TRANSLATION_FOLLOWER = 30;
    public static final int INTAKE_DEPLOYER = 32;
    public static final int LAUNCHER_LOWER_WEST = 33;
    public static final int LAUNCHER_UPPER_WEST = 34;
    public static final int LAUNCHER_LOWER_EAST = 43;
    public static final int LAUNCHER_UPPER_EAST = 44;
    public static final int SINGULATOR_WEST = 35;
    public static final int SINGULATOR_EAST = 36;
    public static final int UPTAKE_WEST = 37;
    public static final int UPTAKE_EAST = 38;
    public static final int HOOD = 39;
  }
}
