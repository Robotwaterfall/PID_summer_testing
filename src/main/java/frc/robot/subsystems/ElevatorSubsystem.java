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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax primaryElevatorMotor = new SparkMax(ElevatorConstants.kPrimaryElevatorMotorPort, MotorType.kBrushless);
  private SparkMax secondaryElevatorMotor = new SparkMax(ElevatorConstants.kSecondaryElevatorMotorPort, MotorType.kBrushless);

  private SparkMaxConfig primaryElevatorConfig = new SparkMaxConfig();
  private SparkMaxConfig secondaryElevatorConfig = new SparkMaxConfig();

  private PIDController elevatorPidController = new PIDController(
    ElevatorConstants.kElevatorkp, 
    ElevatorConstants.kElevatorki, 
    ElevatorConstants.kElevatorkd
  );

  private double intakeHeightToGround_Meters = ElevatorConstants.kElevatorIntakeHeightToGround_Meters;

  public ElevatorSubsystem() {
    
    secondaryElevatorConfig.follow(ElevatorConstants.kPrimaryElevatorMotorPort, true);

    primaryElevatorConfig.encoder.positionConversionFactor(ElevatorConstants.elevatorMotorRotationToMeters);

    secondaryElevatorMotor.configure(secondaryElevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public SparkMax getPrimaryElevatorMotor(){
    return primaryElevatorMotor;
  }

  public SparkMax getSecoundaryElevatorMotor(){
    return secondaryElevatorMotor;
  }

  public double getPrimaryElevatorPosition(){
    return primaryElevatorMotor.getEncoder().getPosition();
  }

  public PIDController getElevatorController(){
    return elevatorPidController;
  }

  public double getIntakeHeightToGround_Meters() {
    return intakeHeightToGround_Meters;
  }

  public void setIntakeHeightToGround_Meters(double intakeHeightToGround_Meters) {
    this.intakeHeightToGround_Meters = intakeHeightToGround_Meters;
  }

  public void resetElevatorEncoders(){
    primaryElevatorMotor.getEncoder().setPosition(0);
  }
}
