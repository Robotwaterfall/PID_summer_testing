// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.InetSocketAddressSerializer;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake_PitcherSubsystem;


public class intake_PitcherCommand extends Command {

  private final intake_PitcherSubsystem pitcherSub;

  private final SparkMax intakePitcherMotor;

  private final PIDController intakePitcherController;


 
  public intake_PitcherCommand(intake_PitcherSubsystem pitcherSub,
  SparkMax intakePitcherMotor, PIDController intakePitcherController) {

    this.pitcherSub = pitcherSub;
    this.intakePitcherMotor = intakePitcherMotor;
    this.intakePitcherController = intakePitcherController;

    addRequirements(pitcherSub);
    
  }

  @Override
  public void initialize() {

    intakePitcherMotor.set(0);
    intakePitcherMotor.stopMotor();

    SmartDashboard.putBoolean("isPitcherRunning", true);
  }


  @Override
  public void execute() {
    double currentIntakePosition_degress = intakePitcherMotor.getEncoder().getPosition();

    SmartDashboard.putNumber("currentPitch_Degress", currentIntakePosition_degress);
    SmartDashboard.putNumber("pitcher_Error", intakePitcherController.getPositionError());
    SmartDashboard.putData("pitcherController", intakePitcherController);

    double output = intakePitcherController.calculate(currentIntakePosition_degress, 
    pitcherSub.getIntakePitcherSetpoint_Degress());

    intakePitcherMotor.set(output);

    SmartDashboard.putNumber("pitcher_Output", output);
  }

  
  @Override
  public void end(boolean interrupted) {

    intakePitcherMotor.set(0);
    intakePitcherMotor.stopMotor();

    SmartDashboard.putBoolean("isPitcherRunning", false);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
