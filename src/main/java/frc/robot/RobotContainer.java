// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.armCommand;
import frc.robot.commands.dismountCommand;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.dismountMotorSubsystem;



public class RobotContainer {

  private armSubsystem armSub = new armSubsystem();
  private dismountMotorSubsystem dismountSub = new dismountMotorSubsystem();
  
  private Joystick Controller1 = new Joystick(OIConstants.kControllerPort);


  public RobotContainer() {
    
    configureBindings();
  }



  private void configureBindings(){

    Command dismountAlgaelvl2CMD = new SequentialCommandGroup(

    new InstantCommand(() -> {
      armSub.setArmSetPoint(90);
      dismountSub.getdismountMotor().set(1);
  })
    );
    
    new JoystickButton(Controller1, 1).onTrue(dismountAlgaelvl2CMD);
  }


  public Command getAutonomousCommand() {
    return null;
  }
}