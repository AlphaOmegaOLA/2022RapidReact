// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This project is under version control using Alpha Omega's Github
// account at https://github.com/AlphaOmegaOLA/2022RapidReact

// Alpha Omega 2022 competition code updated 4/9/22.
// FRC WORLD CHAMPIONSHIPS EDITION!!! HOUSTON HERE WE COME!

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// Countdown Timers
import edu.wpi.first.wpilibj.Timer;
// XBox joystick
import edu.wpi.first.wpilibj.XboxController;
// Try the fancy Mecanum Drive this year
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// Drive System Motor Controller
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Intake Wheel Motor Controller
import edu.wpi.first.wpilibj.motorcontrol.Talon;
// Webcam Server
import edu.wpi.first.cameraserver.CameraServer;
// USB Cameras
import edu.wpi.first.cscore.UsbCamera;
// Slew rate limiter to make drive system less jumpy.
// Currently not in use.
import edu.wpi.first.math.filter.SlewRateLimiter;
// Driver station info for game data
import edu.wpi.first.wpilibj.DriverStation;
// Dashboard for autonomous selection
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot 
{
  // Drive System Motor Controllers
  private final Spark rightRearMotor = new Spark(0);
  private final Spark rightFrontMotor = new Spark(1);
  private final Spark leftFrontMotor = new Spark(2);
  private final Spark leftRearMotor = new Spark(3);

  private static final String defaultAuto = "Shoot-Roll Back";
  private static final String optionOneAuto = "Shoot-Wait-Roll Back";

  // Mecanum Drive System
  private MecanumDrive robotDrive;

  // Climber Motor Controller
  private final Spark climberMotor = new Spark(4);

  // Intake Arm Motor Controller
  private final Spark intakeArmMotor = new Spark(5);

  // Intake Wheels Motor Controller
  private final Talon intakeWheelsMotor = new Talon(6);

  // LED Lights Controller
  private final Spark lights = new Spark(7);

  // XBox drive Controller
  private final XboxController drive_xbox = new XboxController(0);

  // XBox drive Controller
  private final XboxController game_xbox = new XboxController(1);

  // Top Camera
  UsbCamera topcam;
  UsbCamera bottomcam;

  // Slew rater limiter to make joystick less jumpy.
  // Currently not in use.
  SlewRateLimiter filter = new SlewRateLimiter(0.5);

  // Timer for autonomous
  Timer autoTimer = new Timer();

  // Game timer for Teleop
  Timer gameTimer = new Timer();
    
  // Dampens the speed on the drive motors for testing. Set to 1.0
  // to go full speed for competition
  double speed = 0.6;

  String autoName;

  // AUTONOMOUS CODE
  @Override
  public void autonomousInit() 
  {
    // Timer reset
    autoTimer.reset();
    autoTimer.start();

    // Autonomous mode selection - go with the default if not set
    autoName = SmartDashboard.getString("Auto Selector", defaultAuto);

    // Turn on the lights for autonomous based on our alliance color
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    {
      // Breath, Blue
      lights.set(-0.15); 
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      // Breath, Red
      lights.set(-0.17);
    }
    else
    {
      // Color waves, Rainbow Palette
      lights.set(-0.45);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
    // Autonmous operations

    // 1. Fire a ball into the pit to get points.
    // 2. Back out of the tarmac to get taxi points.
    
    // Make sure all unused motors are set to 0 to
    // prevent stuttering.
    climberMotor.set(0);
    intakeArmMotor.set(0);

    robotDrive.feed();
    
    switch(autoName)
    {
      case defaultAuto:
        // Our original autonomous:
        //  1. Fire the ball at 3 secs for 3 secs
        //  2. Roll back at 6.5 secs for 1.5 secs 
        if (autoTimer.get() > 3.0 && autoTimer.get() < 6.0) 
        {
          // Release the ball
          intakeWheelsMotor.set(-1.0);
        }
        else if (autoTimer.get() > 6.5 && autoTimer.get() < 8.0) 
        { 
          // Taxi backwards off the tarmac
          rightFrontMotor.set(-.5);
          rightRearMotor.set(-.5);
          leftFrontMotor.set(-.5);
          leftRearMotor.set(-.5);
        } 
        else if  (autoTimer.get() > 8.1)
        {
          // stop robot  
          rightFrontMotor.set(0);
          rightRearMotor.set(0);
          leftFrontMotor.set(0);
          leftRearMotor.set(0); 
          // Stop the launcher
          intakeWheelsMotor.set(0);
        }
        else
        {
          // stop robot  
          rightFrontMotor.set(0);
          rightRearMotor.set(0);
          leftFrontMotor.set(0);
          leftRearMotor.set(0); 
          // Stop the launcher
          intakeWheelsMotor.set(0);
        }

      case optionOneAuto:
        // Team 118 Robonauts alliance auto:
        //  1. Fire the ball at 3 secs
        //  2. Roll back at 10 secs for 4 secs.
        if (autoTimer.get() > 3.0 && autoTimer.get() < 6.0) 
        {
          // Release the ball
          intakeWheelsMotor.set(-1.0);
        }
        else if (autoTimer.get() > 10.0 && autoTimer.get() < 14.0) 
        {
            // Taxi backwards off the tarmac
            rightFrontMotor.set(-.5);
            rightRearMotor.set(-.5);
            leftFrontMotor.set(-.5);
            leftRearMotor.set(-.5);
        } 
        else if (autoTimer.get() > 14.0) 
        {
          // stop robot  
          rightFrontMotor.set(0);
          rightRearMotor.set(0);
          leftFrontMotor.set(0);
          leftRearMotor.set(0); 
          // Stop the launcher
          intakeWheelsMotor.set(0);
        }
        else
        {
          // stop robot  
          rightFrontMotor.set(0);
          rightRearMotor.set(0);
          leftFrontMotor.set(0);
          leftRearMotor.set(0); 
          // Stop the launcher
          intakeWheelsMotor.set(0);
        }
    }
  }
  
  @Override
  public void robotInit() 
  {
    // Autonomous Mode Selection
    String[] options = {defaultAuto, optionOneAuto};
    SmartDashboard.putStringArray("Auto List", options);
    SmartDashboard.setDefaultString("Auto Selector", defaultAuto);
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
    leftFrontMotor.setInverted(false);
    leftRearMotor.setInverted(false);

    // The Mecanum Drive requires all 4 motors to operate independently
    robotDrive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
    robotDrive.feed();
    // Send camera feeds to dashboard.
    // Lift hook camera
    topcam = CameraServer.startAutomaticCapture(0);
    // Drive camera
    bottomcam = CameraServer.startAutomaticCapture(1);
    
    // Turn on the lights for teleop
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    {
      // Breath, Blue
      lights.set(-0.15); 
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      // Breath, Red
      lights.set(-0.17);
    }
    else
    {
      // Color Waves, Rainbow Palette
      lights.set(-0.45);
    }
  }

  // MANUAL OPERATION
  @Override
  public void teleopPeriodic() 
  {
    // Lights go crazy in the last 10 seconds of the match
    // when we should be hanging.
    boolean endGame = false;
    if ((gameTimer.get()) > 130.0 && !endGame)
    {
      endGame = true;
      // Color Waves, Party Palette
      lights.set(-0.43);
    }
    
    // DRIVE SYSTEM
    
    // Mecanum drive.  The last argument is "gyroangle" to set field-oriented
    // vs. driver oriented steering.  Field-oriented allows the robot to spin
    // while driving in any direction. For driver-oriented you set it to 0.0;
    // We are using driver oriented and not using the gyro.

    // Mecanum driving - note Y is first and multiplied by a negative  
    robotDrive.driveCartesian(-1*drive_xbox.getLeftY(), drive_xbox.getLeftX(), drive_xbox.getRightX());

    // CLIMBER SYSTEM

    // Pull the climbing hook down so the robot climbs.    
    if (game_xbox.getYButton())
    {
      System.out.println("Y Button (Triangle) - Resetting Hook!");
      climberMotor.set(.5);
    }
    // Lower the robot by unwinding the spool
    else if (game_xbox.getAButton())
    {
      System.out.println("A Button (X Button) - End Game Climb!");
      climberMotor.set(-.65);
    } 

    // GAME PIECE INTAKE SYSTEM
    // Retrieve game pieces - Square
    else if (game_xbox.getXButton())
    {
      System.out.println("X Button (Square) - Intake retrieving!");
      intakeWheelsMotor.set(.6);
    }
    // Release game pieces - Circle
    else if (game_xbox.getBButton())
    {
      System.out.println("B Button (Circle) - Fast Intake Release!");
      intakeWheelsMotor.set(-1.0);
    }
    else if (game_xbox.getRightX() > 0)
    {
      System.out.println("Operator Right Joystick - Slow Intake Release!");
      intakeWheelsMotor.set(-0.4);
    }
    // INTAKE ARM SYSTEM
    // Raise the arm - Right trigger
    else if (game_xbox.getRightBumper())
    {
      intakeArmMotor.set(.7);
      System.out.println("Right Bumper Button - Raise Intake Arm");
    }
    // Lower the Arm - Left trigger
    else if (game_xbox.getLeftBumper())
    {
      intakeArmMotor.set(-.4);
      System.out.println("Left Bumper Button - Lower Intake Arm");
    }
    else
    {
      // Stop any other motors
      climberMotor.set(0);
      intakeWheelsMotor.set(0);
      intakeArmMotor.set(0);
    }
  }
}
