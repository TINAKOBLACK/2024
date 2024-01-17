// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static class Chassis {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24.75); // Measure and set trackwidth
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.75); // Measure and set wheelbase
  }

  public static class Autonomous {
    public static final double kPXController = 4.0;
    public static final double kPYController = 4.0;
    public static final double kPThetaController = 4.0;
  }
}
