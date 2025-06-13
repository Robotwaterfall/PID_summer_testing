// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PitcherConstants;

public class intake_PitcherSubsystem extends SubsystemBase {

  private SparkMax intakePitcherMotor = new SparkMax(PitcherConstants.kPitcherMotorPort, MotorType.kBrushless);
  private SparkMaxConfig intakePitcherMotorConfig = new SparkMaxConfig();

  private PIDController intakePitcherPIDController = new PIDController(
  PitcherConstants.kPitcherkp, 
  PitcherConstants.kPitcherki, 
  PitcherConstants.kPitcherkd);

  private double intakePitcherSetpoint_Degress = 0.0;
  
  public intake_PitcherSubsystem() {}

  @Override
  public void periodic() {
    intakePitcherPIDController.enableContinuousInput(0,360);

    intakePitcherMotorConfig.absoluteEncoder.positionConversionFactor(360);

    intakePitcherMotor.configure(intakePitcherMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, 
    PersistMode.kNoPersistParameters);

  }

  public SparkMax getIntakePitcherMotor(){
    return intakePitcherMotor;
  }

  public PIDController getIntakePitcherController(){
    return intakePitcherPIDController;
  }

  public double getIntakePitcherSetpoint_Degress(){
    return intakePitcherSetpoint_Degress;
  }

  public void setIntakePitcherSetpoint_Degrees(double Setpoint_Degress){
    intakePitcherSetpoint_Degress = Setpoint_Degress;
  }


}
