// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.dismountMotorSubsystem;


public class dismountCommand extends Command {

  private final dismountMotorSubsystem dismountSub;
  private final SparkMax dismountMotor;
  private double power;
  
  public dismountCommand(dismountMotorSubsystem dismountSub, double power) {
    this.dismountSub = dismountSub;
    this.dismountMotor = dismountSub.getdismountMotor();
    this.power = power;

    addRequirements(dismountSub);
   
  }

  
  @Override
  public void initialize() {
    dismountMotor.set(0);
    dismountMotor.stopMotor();
    SmartDashboard.putBoolean("Dismount_Motor_Initialized", true);
  }

  
  @Override
  public void execute() {
   dismountMotor.set(power);
  }

  
  @Override
  public void end(boolean interrupted) {
   dismountMotor.set(0);
   dismountMotor.stopMotor();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
