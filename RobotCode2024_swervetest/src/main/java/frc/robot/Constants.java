// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = 70; // kg
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.020 + 0.110; //s, 20ms + 110ms sprk max velocity lag, maybe 100ms for TalonFX

  public static final double WHEEL_LOCK_TIME = 3; // seconds

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.14;
    public static final double LEFT_Y_DEADBAND = 0.14;
    public static final double RIGHT_X_DEADBAND = 0.14;
    public static final double TURN_CONSTANT = 6;
  }

  public static class Chassis {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.75); // Measure and set trackwidth
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.75); // Measure and set wheelbase

    public static final double MAXSPEED = 4.0; // m/s
    public static final double MAXANGULARSPEED = MAXSPEED / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);; // m/s
  }

  public static class Autonomous {
    public static final double kPXController = 4.0;
    public static final double kPYController = 4.0;
    public static final double kPThetaController = 4.0;
  }
}
