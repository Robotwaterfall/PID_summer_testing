// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.jsontype.impl.ClassNameIdResolver;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climberSubsystem;


public class climbCommand extends Command {

  private final climberSubsystem climbSub;

  private final SparkMax primaryClimbMotor;

  private final PIDController climbController;
  
  public climbCommand(climberSubsystem climbSub, double setpoint_degress) {

    this.climbSub = climbSub;
    this.primaryClimbMotor = climbSub.getPrimaryClimberMotor();
    this.climbController = climbSub.getClimberController();

    addRequirements(climbSub);
   
  }

  
  @Override
  public void initialize() {

    primaryClimbMotor.set(0);
    primaryClimbMotor.stopMotor();

    SmartDashboard.putBoolean("Climbing", true);
  }

  
  @Override
  public void execute() {

    SmartDashboard.putData("climbController", climbController);
    SmartDashboard.putNumber("currentPostion_Angle", climbSub.getClimberPositionDegress());
    SmartDashboard.putNumber("currentClimbError", climbController.getError());

    double output = climbController.calculate(climbSub.getClimberPositionDegress(), 
    climbSub.getClimbSetpoint_Degress());

    primaryClimbMotor.set(output);
  }

  
  @Override
  public void end(boolean interrupted) {

    primaryClimbMotor.set(0);
    primaryClimbMotor.stopMotor();

    SmartDashboard.putBoolean("Climbing", false);
  }

  
  @Override
  public boolean isFinished() {

    if (Math.abs(climbController.getError()) <= 0.3){
      return true;
    }


    return false;
  }
}
