// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightServer;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Chassis.*;

public class PositionAprilTag extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLightServer lServer;
  private static double MAX_VELOCITY = MAXSPEED;
  private static double MAX_OMEGA = MAXANGULARSPEED;
  private double kP = 0.08;
  
  PIDController pX = new PIDController(0.2, 0.000, 0.03);
  PIDController pY = new PIDController(0.2, 0.000, 0.03);
  
  public PositionAprilTag(SwerveSubsystem swerveSubsystem, LimeLightServer lServer) {
    this.swerveSubsystem = swerveSubsystem;
    this.lServer = lServer;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pX.setSetpoint(0.0);
    pY.setSetpoint(0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pXspeed = modifyAxis(maxValue(-pX.calculate(lServer.getY()), 0.5)) * MAX_VELOCITY;
    double pYspeed = modifyAxis(maxValue(pY.calculate(lServer.getX()), 0.5)) * MAX_VELOCITY;


    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              pXspeed, //xSpeed
              pYspeed, //ySpeed
              0.0,
              drivetrain.getGyroscopeRotation()
      )
    );

  SmartDashboard.putNumber("pY", pYspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(lServer.getX()) < 0.2) && (Math.abs(lServer.getY()) < 0.2);
  }

  private static double modifyAxis(double value) {
    // Deadband
    //value = deadband(value, 0.11);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private double maxValue(double value, double max){
    if (value > max)
      return max;
    else if (value < -max)
      return -max;
    else
      return value;
  }
}
