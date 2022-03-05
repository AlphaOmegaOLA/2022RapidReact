// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This project is under version control using Alpha Omega's Github
// account at https://github.com/AlphaOmegaOLA/2022RapidReact

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
// Webcam
import edu.wpi.first.cameraserver.CameraServer;
// Gyro
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// Limit switch
import edu.wpi.first.wpilibj.DigitalInput;
// Slew rate limiter to make drive system less jumpy
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Robot extends TimedRobot 
{
  // Drive System Motor Controllers
  private final Spark rightRearMotor = new Spark(0);
  private final Spark rightFrontMotor = new Spark(1);
  private final Spark leftFrontMotor = new Spark(2);
  private final Spark leftRearMotor = new Spark(3);

  // Climber Motor Controller
  private final Spark climberMotor = new Spark(4);

  // Intake Arm Motor Controller
  private final Spark intakeArmMotor = new Spark(5);

  // Intake Wheels Motor Controller
  private final Talon intakeWheelsMotor = new Talon(6);

  // LED Lights Controller
  private final Spark lights = new Spark(7);

  // Mecanum Drive System
  private MecanumDrive robotDrive;

  // XBox Controller
  private final XboxController xbox = new XboxController(0);

  // The gyro
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Limit switch - for stoping the intake arm from
  //                going down too far
  DigitalInput armLimitSwitch = new DigitalInput(0);

  // Slew rater limiter to make joystick less jumpy
  SlewRateLimiter filter = new SlewRateLimiter(.5);

  // Timer for autonomous
  Timer autoTimer = new Timer();

  // Game timer for Teleop
  Timer gameTimer = new Timer();

  // Stores the gyro angle so we don't print it more than we need to.
  double angle = gyro.getAngle();
    
  // Dampens the speed on the drive motors for testing. Set to 1.0
  // to go full speed for competition
  double speed = .3;

  // Flag to see if the ball was
  // launched in autonomous mode
  boolean ballLaunched = false;

  // AUTONOMOUS CODE
  @Override
  public void autonomousInit() 
  {
    // autonomous init code goes here
    autoTimer.reset();
    autoTimer.start();

    // Turn on the lights for autonomous
    lights.set(-0.51);
  }

  @Override
  public void autonomousPeriodic()
  {
    // Autonmous operations

    // 1. Fire a ball into the pit to get points.
    // 2. Lower the arm
    // 3. Back out of the tarmac to get taxi points.
    //    Drive for 2 seconds
    
    // Fire the ball. Note: in testing
    // make sure we don't need to rais the arm
    if (!ballLaunched)
    {
      ballLaunched = true;
      if (autoTimer.get() < 5.0)
      {
        intakeWheelsMotor.set(-1.0);
      }
      else
      {
        intakeWheelsMotor.stopMotor();
      }

    }
    
    if (autoTimer.get() < 8.0) 
    {
        robotDrive.driveCartesian(-0.5, 0.0, 0.0); // drive backwards half speed
    } 
    else 
    {
      // stop robot  
      robotDrive.stopMotor(); 
    }
  }

  @Override
  public void robotInit() 
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 
    rightFrontMotor.setInverted(false);
    rightRearMotor.setInverted(false);
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    // The Mecanum Drive requires all 4 motors to operate independently
    robotDrive = new MecanumDrive(rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor);

    // Send camera feed to dashboard
    CameraServer.startAutomaticCapture();
  }

  // MANUAL OPERATION
  @Override
  public void teleopPeriodic() 
  {
    
    // Lights go crazy in the last 10 seconds of the match
    // when we should be hanging.
    // Set to "Color Waves, Party Palette" 
    boolean endGame = false;
    if ((gameTimer.get()) > 130.0 && !endGame)
    {
      endGame = true;
      lights.set(-0.43);
    }
    
    // DRIVE SYSTEM
    
    // Mecanum drive.  The last argument is "gyroangle" to set field-oriented
    // vs. driver oriented steering.  Field-oriented allows the robot to spin
    // while driving in any direction. For driver-oriented you set it to 0.0;
    
    // Gyro angle readout
    double newAngle = gyro.getAngle();
    if (newAngle != angle)
    {
      angle = newAngle;
      System.out.print("Gyro angle: ");
      System.out.println(angle);
    }
    System.out.print("Gyro Angle: ");
    System.out.println(gyro.getAngle());

    // Directional lights
    if (xbox.getLeftY() > 0)
    {
      // Moving forward is solid green
      lights.set(.77);
    }
    else if (xbox.getLeftY() < 0)
    {
      // Moving backwards is solid red
      lights.set(.61);
    }

    // Mecanum driving  
    robotDrive.driveCartesian(filter.calculate(xbox.getLeftY()*speed), filter.calculate(xbox.getLeftX()*speed), filter.calculate(xbox.getRightX()*speed), gyro.getAngle());

    // CLIMBER SYSTEM

    // First we check if the intake arm is coming down and
    // hitting the switch...
    if (armLimitSwitch.get() && xbox.getLeftBumper())
    {
      // The intake arm has gone far enough. Stop it.
      System.out.println("Intake Arm Limit Switch Triggered");
      intakeArmMotor.stopMotor();
      // Set lights to strobe gold
      lights.set(-0.07);
    }
    // Pull the climbing hook down so the robot climbs.    
    else if (xbox.getYButton())
    {
      System.out.println("Y Button (Triangle) - Climbing up!");
      climberMotor.set(.5);
      // Set lights to "large fire"
      lights.set(-0.57);
    }
    // Lower the robot by unwinding the spool
    else if (xbox.getAButton())
    {
      System.out.println("A Button (X Button) - Climbing down!");
      climberMotor.set(-.5);
      // Set lights to rainbow twinkles
      lights.set(-0.55);
    } 

    // GAME PIECE INTAKE SYSTEM
    // Retrieve game pieces - Square
    else if (xbox.getXButton())
    {
      System.out.println("X Button (Square) - Intake retrieving!");
      intakeWheelsMotor.set(1.0);
      // Set lights to "light chase blue"
      lights.set(-0.29);
    }
    // Release game pieces - Circle
    else if (xbox.getBButton())
    {
      System.out.println("B Button (Circle) - Intake releasing!");
      intakeWheelsMotor.set(-1.0);
      // Set lights to "light chase red"
      lights.set(-0.31);
    }
    // INTAKE ARM SYSTEM
    // Raise the arm - Right trigger
    else if (xbox.getRightBumper())
    {
      intakeArmMotor.set(.5);
      System.out.println("Right Bumper Button - Raise Intake Arm");
      // Set lights to solid violet
      lights.set(.91);
    }
    // Lower the Arm - Left trigger
    else if (xbox.getLeftBumper())
    {
      intakeArmMotor.set(-.5);
      System.out.println("Left Bumper Button - Lower Intake Arm");
      // Set lights to solid white
      lights.set(.93);
    }
    else
    {
      // Turn the lights off
      if (!endGame)
      {
        lights.stopMotor();
      }
      // Stop any other motors
      climberMotor.stopMotor();
      intakeWheelsMotor.stopMotor();
      intakeArmMotor.stopMotor();
    }
  }
}
