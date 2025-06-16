// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class climberSubsystem extends SubsystemBase {
 
  private final SparkMax primaryClimberMotor = new SparkMax(ClimberConstants.kPrimaryClimberMotor, MotorType.kBrushless);
  private final SparkMax secoundaryClimberMotor = new SparkMax(ClimberConstants.kSecoundaryClimberMotor, MotorType.kBrushless);

  private final SparkMaxConfig primaryClimberMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig secoundaryClimberMotorConfig = new SparkMaxConfig();

  private double idleClimberAngle;

  
  public climberSubsystem() {}

  @Override
  public void periodic() {
   
  }
}
