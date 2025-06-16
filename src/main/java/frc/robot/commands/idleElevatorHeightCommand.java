// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class idleElevatorHeightCommand extends Command {

  private final ElevatorSubsystem elevatorSub;

  private final SparkMax primaryMotor;
  
  private final PIDController elevatorController;

  public idleElevatorHeightCommand(ElevatorSubsystem elevatorSub) {
    
    this.elevatorSub = elevatorSub;
    this.primaryMotor = elevatorSub.getPrimaryElevatorMotor();
    this.elevatorController = elevatorSub.getElevatorController();

    addRequirements(elevatorSub);
    
  }

  
  @Override
  public void initialize() {

    primaryMotor.set(0);
    primaryMotor.stopMotor();

    SmartDashboard.putBoolean("IdleElevatorHeightCMD", true);
  }

  
  @Override
  public void execute() {

    SmartDashboard.putData("elevatorController", elevatorController);
    SmartDashboard.putNumber("intakeHeightSetpoint", elevatorSub.getIntakeHeightSetpoint_Inches());
    SmartDashboard.putNumber("elevatorPositionError_Inches", elevatorController.getError());
    SmartDashboard.putNumber("currentElevatorPosition_Inches", elevatorSub.getPrimaryElevatorPosition());

    double output = elevatorController.calculate(elevatorSub.getPrimaryElevatorPosition(), elevatorSub.getIntakeHeightSetpoint_Inches());
    primaryMotor.set(output);
  }

  
  @Override
  public void end(boolean interrupted) {

    primaryMotor.set(0);
    primaryMotor.stopMotor();

    SmartDashboard.putBoolean("IdleElevatorHeightCMD",false);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
