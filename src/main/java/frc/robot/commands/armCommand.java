// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.armConstants;
import frc.robot.subsystems.armSubsystem;


public class armCommand extends Command {
  private final armSubsystem armSub;
  private final SparkMax armMotor;
  private final PIDController pidController;

  
  public armCommand(armSubsystem armSub, SparkMax armMotor, PIDController pidController) {
    this.armSub = armSub;
    this.armMotor = armMotor;
    this.pidController = pidController;
    addRequirements(armSub);
  }


  @Override
  public void initialize() {
    armMotor.set(0);
    armMotor.stopMotor();

    SmartDashboard.putBoolean("Arm Hold Position", true);
    
  }


  @Override
  public void execute() {
    double armCurrentPosition = armMotor.getAbsoluteEncoder().getPosition();
    SmartDashboard.putData(pidController);
    SmartDashboard.putNumber("arm_current_error",pidController.getError());
    SmartDashboard.putNumber("arm_current_degrees", armCurrentPosition);

    double output = pidController.calculate(armCurrentPosition, armSub.getArmSetPoint());
    if(Math.abs(output) >= armConstants.kMaxArmPower) {

      output = armConstants.kMaxArmPower * Math.signum(output);

    }

    armMotor.set(output);

  }

 
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm terminated");
  }

 
  @Override
  public boolean isFinished() {
    return false;
  }
}
