// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pitcher_ConsumerMotorSubsystem;


public class powerConsumerCommand extends Command {

  private final pitcher_ConsumerMotorSubsystem consumerSub;
  private final SparkMax consumerMotor;
  private final Supplier<Double> intakeConsumerSpeedSupplier;
  private final Supplier<Double> outtakeConsumerSpeedSupplier;
  
  public powerConsumerCommand(pitcher_ConsumerMotorSubsystem consumerSub,
  Supplier<Double> intakeConsumerSpeedSupplier, 
  Supplier<Double> outtakeConsumerSpeedSupplier) {
    this.consumerSub = consumerSub;
    this.consumerMotor = consumerSub.getConsumerMotor();
    this.intakeConsumerSpeedSupplier = intakeConsumerSpeedSupplier;
    this.outtakeConsumerSpeedSupplier = outtakeConsumerSpeedSupplier;
  }

  
  @Override
  public void initialize() {
    consumerMotor.set(0);
    consumerMotor.stopMotor();

    SmartDashboard.putBoolean("isConsumerMotorRunning", true);
  }


  @Override
  public void execute() {
    consumerMotor.set((intakeConsumerSpeedSupplier.get() *0.3)
    - (outtakeConsumerSpeedSupplier.get() * 0.3));
  }

  
  @Override
  public void end(boolean interrupted) {
    consumerMotor.set(0);
    consumerMotor.stopMotor();

    SmartDashboard.putBoolean("isConsumerMotorRunning", false);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
