// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PitcherConsumerConstants;

public class pitcher_ConsumerMotorSubsystem extends SubsystemBase {
  
  SparkMax pitcher_ConsumerMotor = new SparkMax(PitcherConsumerConstants.kPitcherConsumerMotor, MotorType.kBrushless);

  public SparkMax getConsumerMotor(){
    return pitcher_ConsumerMotor;
  }

  
}
