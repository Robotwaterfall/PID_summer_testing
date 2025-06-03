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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

public class armSubsystem extends SubsystemBase {

  private final SparkMax armMotor = new SparkMax (armConstants.kArmMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  
  private boolean isHoldPosition;
  private double armSetPoint_degress;

  private PIDController pidController = new PIDController(
    armConstants.kArmkp, 
    armConstants.kArmki, 
    armConstants.kArmkd
    );


  public armSubsystem() {
    pidController.enableContinuousInput(0, 360);

        /*sets the dismount arm encoder units.  */
        armMotorConfig.absoluteEncoder.positionConversionFactor(360);
        /*applies dimount arm motor configuration to the dismount arm motor. */
        armMotor.configure(armMotorConfig,ResetMode.kResetSafeParameters,
         PersistMode.kNoPersistParameters);
  }

  public SparkMax getarmMotor(){
    return armMotor;
  }

  public PIDController getArmPidController(){
    return pidController;
  }

  public void getIsHoldPosition(boolean state){
    isHoldPosition = state;
  }

  public double getArmSetPoint(){
    return armSetPoint_degress;
  }

 public void setArmSetPoint(double newSetPoint_degress){
  armSetPoint_degress = newSetPoint_degress;
 }

  
}
