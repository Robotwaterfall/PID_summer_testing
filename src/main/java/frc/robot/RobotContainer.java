// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static edu.wpi.first.units.Units.Meters;

import java.time.Instant;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PitcherConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.armCommand;
import frc.robot.commands.dismountCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.dismountMotorSubsystem;
import frc.robot.subsystems.intake_PitcherSubsystem;



public class RobotContainer {

  private armSubsystem armSub = new armSubsystem();
  private dismountMotorSubsystem dismountSub = new dismountMotorSubsystem();
  private ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
  private intake_PitcherSubsystem pitcherSub = new intake_PitcherSubsystem();
  
  private Joystick Controller1 = new Joystick(OIConstants.kControllerPort);


  public RobotContainer() {
    
    configureBindings();
  }



  private void configureBindings(){

    Command dismountAlgaelvl2CMD = new SequentialCommandGroup(

    new InstantCommand(() -> {
      armSub.setIsHoldPosition(!armSub.getIsHoldPosition());
    }),
    new ConditionalCommand(
      new InstantCommand(() -> {
      armSub.setArmSetPoint(90);
      dismountSub.getdismountMotor().set(1);
       }),
     new InstantCommand(() -> {
      armSub.setArmSetPoint(0);
      dismountSub.getdismountMotor().set(0);
       }),
       () -> {
        return armSub.getIsHoldPosition();
       }
       )
    );
    Command scoreCoralLvl2CMD = new SequentialCommandGroup(

      new InstantCommand(() -> {
        
        elevatorSub.setIntakeHeightToGround_Meters(ElevatorConstants.kElevatorScoreLVL2CoralHeight_Meters);

        pitcherSub.setIntakePitcherSetpoint_Degrees(PitcherConstants.kPitcherSetpoint_DegreesLVL2);
      }
    )
    );
    
    new JoystickButton(Controller1, 1).onTrue(dismountAlgaelvl2CMD);

    new JoystickButton(Controller1, 2).onTrue(scoreCoralLvl2CMD);
  }


  public Command getAutonomousCommand() {
    return null;
  }
}