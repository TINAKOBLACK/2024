// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkMax left = new CANSparkMax(23, MotorType.kBrushless);
  CANSparkMax right = new CANSparkMax(28, MotorType.kBrushless);

  public Shooter() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();
    left.setIdleMode(IdleMode.kCoast);
    right.setIdleMode(IdleMode.kCoast);

    boolean inverted = false;
    left.setInverted(!inverted);
    right.setInverted(inverted);
  }

  /* public void shot(double speed){
    left.set(speed);
    right.set(speed);
  } */

  public void neutral(){
    left.set(0);
    right.set(0);
  }

  public Command shotCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          left.set(speed);
          right.set(0.75*speed);
        }
    );
  }

  public Command setNeutral() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          neutral();
        }
    );
  }
  
  @Override
  public void periodic() {
  }
}
