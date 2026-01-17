// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CAN {
    public static final CANBus kMAIN_BUS = new CANBus("main");
    public static final CANBus kAUX_BUS = new CANBus("aux");

    public static final int INTAKE_TRANSLATION = 9; //TODO: replace with actual value
    public static final int INTAKE_DEPLOYER = 10; //TODO: replace with actual value
    public static final int SHOOTER_LEFT = 11;
    public static final int SHOOTER_RIGHT = 12;
  }
}
