// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static class armConstants{

    public final static double kMaxArmPower = 0.5; // Maximum power for the arm motor

    public final static int kArmMotorPort = 1; // Port number for the arm motor
 
    // PID
    public final static double kArmkp = 0;
    public final static double kArmki = 0;
    public final static double kArmkd = 0;
  }

  public final static class OIConstants{
    public final static int kControllerPort = 0;
  }

  public final static class dismountConstants{
    public final static int kDismountMotorPort = 2;
  }
  
  }

