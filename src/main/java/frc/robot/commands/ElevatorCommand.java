// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSub;

  private final PIDController elevatorController;

  private final SparkMax primaryElevatorMotor;

  private final double intakeHeightSetpoint_Meters;


  public ElevatorCommand(ElevatorSubsystem elevatorSub, PIDController elevatorController, double intakeHeightSetpoint_Inches) {

    this.elevatorSub = elevatorSub;

    this.elevatorController = elevatorSub.getElevatorController();

    this.primaryElevatorMotor = elevatorSub.getPrimaryElevatorMotor();

    this.intakeHeightSetpoint_Meters = intakeHeightSetpoint_Inches;

    addRequirements(elevatorSub);
   
  }

  
  @Override
  public void initialize() {

    primaryElevatorMotor.set(0);
    primaryElevatorMotor.stopMotor();

    elevatorSub.setIntakeHeightToGround_Inches(intakeHeightSetpoint_Meters);

    SmartDashboard.putBoolean("idleHeightCMD", true);
  }

  
  @Override
  public void execute() {
    SmartDashboard.putData("elevatorController", elevatorController);
    SmartDashboard.putNumber("intakeHeightSetpoint", elevatorSub.getIntakeHeightSetpoint_Inches());
    SmartDashboard.putNumber("elevatorPosition_Meters", elevatorSub.getPrimaryElevatorPosition());
    SmartDashboard.putNumber("elevatorController_Error", elevatorController.getPositionError());

    double output = elevatorController.calculate(
      elevatorSub.getPrimaryElevatorPosition(),
      elevatorSub.getIntakeHeightSetpoint_Inches()
    );

    primaryElevatorMotor.set(output);
  }


  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("idleHeightCMD", false);
    primaryElevatorMotor.set(0);
    primaryElevatorMotor.stopMotor();
  }


  @Override
  public boolean isFinished() {

    //if the current elevator position is less then or equal too it will end the command

    if (Math.abs(elevatorController.getError()) <= 0.2){
      return true;
    }

    return false;
  }
}
