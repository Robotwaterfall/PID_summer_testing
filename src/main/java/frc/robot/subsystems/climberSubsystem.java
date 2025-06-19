// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class climberSubsystem extends SubsystemBase {
 
  private final SparkMax primaryClimberMotor = new SparkMax(ClimberConstants.kPrimaryClimberMotor, MotorType.kBrushless);
  private final SparkMax secoundaryClimberMotor = new SparkMax(ClimberConstants.kSecoundaryClimberMotor, MotorType.kBrushless);

  private final SparkMaxConfig primaryClimberMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig secoundaryClimberMotorConfig = new SparkMaxConfig();

  private final PIDController climberController = new PIDController
  (ClimberConstants.kClimberKp,
  ClimberConstants.kClimberKi,
  ClimberConstants.kClimberKd);

  private double climbSetpoint_Degress;

  
  public climberSubsystem() {

    climberController.enableContinuousInput(0, 360);

    secoundaryClimberMotorConfig.follow(ClimberConstants.kPrimaryClimberMotor, true);

    primaryClimberMotorConfig.absoluteEncoder.positionConversionFactor(360);

    primaryClimberMotor.configure(primaryClimberMotorConfig, ResetMode.kNoResetSafeParameters, 
    PersistMode.kNoPersistParameters);

    secoundaryClimberMotor.configure(secoundaryClimberMotorConfig, ResetMode.kNoResetSafeParameters, 
    PersistMode.kNoPersistParameters);

  }

  public SparkMax getPrimaryClimberMotor(){
    return primaryClimberMotor;
  }

  public double getClimberPositionDegress(){
    return primaryClimberMotor.getAbsoluteEncoder().getPosition();
  }

  public PIDController getClimberController(){
    return climberController;
  }

  public double getClimbSetpoint_Degress(){
    return climbSetpoint_Degress;
  }

  public double setClimbSetpoint_Degress(){
    return climbSetpoint_Degress;
  }

  

  
}
