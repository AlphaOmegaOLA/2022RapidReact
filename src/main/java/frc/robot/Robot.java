// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This project is under version control using Alpha Omega's Github
// account at https://github.com/AlphaOmegaOLA/2022RapidReact

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
// XBox joystick
import edu.wpi.first.wpilibj.XboxController;
// Try the fancy Mecanum Drive this year
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// Drive System Motor Controller
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Climber/Intake Arm/Intake Wheel Motor Controller
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
// Webcam
//import edu.wpi.first.cameraserver.CameraServer;
// Gyro
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// Limit switch
import edu.wpi.first.wpilibj.DigitalInput;
// Slew rate limiter to make drive system less jumpy
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
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
  private final Spark intakeWheelsMotor = new Spark(6);

  // LED Lights Controller
  private final Spark lights = new Spark(7);

  // Mecanum Drive System
  private MecanumDrive robotDrive;
  // PS4 controller
  //private final PS4Controller ps4 = new PS4Controller(0);
  // XBox Controller
  private final XboxController xbox = new XboxController(0);
  // The gyro
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Limit switch
  DigitalInput armLimitSwitch = new DigitalInput(0);

  // Slew rater limiter to make joystick less jumpy
  SlewRateLimiter filter = new SlewRateLimiter(.5);

  // Autonomous timer
  Timer driveTimer = new Timer();



  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() 
  {
    // autonomous init code goes here
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 
    driveTimer.reset();
    driveTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    // Autonmous operations go here.

    // 1. Fire a ball into the pit to get points.
    // 2. Lower the arm
    // 3. Back out of the tarmac to get taxi points.
       // Drive for 2 seconds
    if (driveTimer.get() < 2.0) 
    {
        robotDrive.driveCartesian(0.5, 0.0, 0.0); // drive forwards half speed
    } 
    else 
    {
        robotDrive.stopMotor(); // stop robot
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
    //CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() 
  {
    // DRIVE SYSTEM
    double speed = .3;
    // Mecanum drive.  The last argument is "gyroangle" to set field-oriented
    // vs. drive oriented steering. Still need to figure that out.
    //robotDrive.driveCartesian(ps4.getLeftX(), ps4.getLeftYS(), ps4.getRightX(), 0.0);
    System.out.println(gyro.getAngle());
    robotDrive.driveCartesian(filter.calculate(xbox.getLeftY()*speed), filter.calculate(xbox.getLeftX()*speed), filter.calculate(xbox.getRightX()*speed), gyro.getAngle());

    // CLIMBER SYSTEM

    // Raise the robot climber to reach the rung by pressing the green triangle button
    if (armLimitSwitch.get() && xbox.getLeftBumper())
    {
      System.out.println("Limit Switch Triggered");
      intakeArmMotor.stopMotor();
    }
    else if (xbox.getYButton())
    {
      climberMotor.set(.5);
    }
    // Lower the robot climber to pull the robot up the rung by pressing
    // the blue X (Cross) button.
    else if (xbox.getAButton())
    {
      climberMotor.set(-.5);
    } 
    // GAME PIECE INTAKE SYSTEM
    // Retrieve game pieces - Square
    else if (xbox.getXButton())
    {
      intakeWheelsMotor.set(1.0);
    }
    // Release game pieces - Circle
    else if (xbox.getBButton())
    {
      intakeWheelsMotor.set(1.0);
    }
    // INTAKE ARM SYSTEM
    // Raise the arm - Right trigger
    else if (xbox.getRightBumper())
    {
      intakeArmMotor.set(.5);
      System.out.println("Right Bumper Button - Raise Intake Arm");
    }
    // Lower the Arm - Left trigger
    else if (xbox.getLeftBumper())
    {
      intakeArmMotor.set(-.5);
      System.out.println("Left Bumper Button - Lower Intake Arm");
    }
    else
    {
      climberMotor.stopMotor();
      intakeWheelsMotor.stopMotor();
      intakeArmMotor.stopMotor();
    }
  }
}
