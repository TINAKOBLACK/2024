// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                               "swerve"));
  //private final Shooter shooter = new Shooter();                                                                        

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /* private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort); */

  HashMap<String, Command> eventMap = new HashMap<>();
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser(); //default none
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));                                                                   

    /* swerveSubsystem.setDefaultCommand(new AbsoluteFieldDrive(
            swerveSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY()) * Chassis.MAXSPEED,
            () -> -modifyAxis(m_driverController.getLeftX()) * Chassis.MAXSPEED,
            () -> -modifyAxis(m_driverController.getRightX()) * Chassis.MAXANGULARSPEED
    )); */
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    /* m_driverController.a().onTrue(shooter.shotCommand(0.25)).onFalse(new InstantCommand(shooter::neutral, shooter));
    m_driverController.b().onTrue(shooter.shotCommand(0.5)).onFalse(new InstantCommand(shooter::neutral, shooter));
    m_driverController.x().onTrue(shooter.shotCommand(0.75)).onFalse(new InstantCommand(shooter::neutral, shooter));
    m_driverController.y().onTrue(shooter.shotCommand(1.0)).onFalse(new InstantCommand(shooter::neutral, shooter)); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    //PathPlannerPath pathchoreo = PathPlannerPath.fromChoreoTrajectory("test");

    swerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path);
    //return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveSubsystem.setMotorBrake(brake);
  }
}
