// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This project is under version control using Alpha Omega's Github
// account at https://github.com/AlphaOmegaOLA/2022RapidReact

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// PS4 joystick
//import edu.wpi.first.wpilibj.PS4Controller;
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
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// Limit switch
//import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot 
{
  // Drive System Motor Controllers
  private final Spark leftFrontMotor = new Spark(0);
  private final Spark rightFrontMotor = new Spark(1);
  private final Spark leftRearMotor = new Spark(2);
  private final Spark rightRearMotor = new Spark(3);

  // Climber Motor Controller
  private final Spark climberMotor = new Spark(4);

  // Intake Arm Motor Controller
  private final Spark intakeArmMotor = new Spark(5);

  // Intake Wheels Motor Controller
  private final Spark intakeWheelsMotor = new Spark(6);

  // Mecanum Drive System
  private MecanumDrive robotDrive;
  // PS4 controller
  //private final PS4Controller ps4 = new PS4Controller(0);
  // XBox Controller
  private final XboxController xbox = new XboxController(0);
  // The gyro
  //private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() 
  {
    // autonomous init code goes here
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    // Autonmous operations go here.

    // 1. Raise the arm to the right height.
    // 2. Fire a ball into the pit to get points.
    // 3. Lower the arm
    // 4. Back out of the tarmac to get taxi points.
    // 5. Spin around to be read to grab a game piece?
  }


  @Override
  public void robotInit() 
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    // The Mecanum Drive requires all 4 motors to operate independently
    robotDrive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    // Send camera feed to dashboard
    //CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() 
  {
    // DRIVE SYSTEM

    // Mecanum drive.  The last argument is "gyroangle" to set field-oriented
    // vs. drive oriented steering. Still need to figure that out.
    //robotDrive.driveCartesian(ps4.getLeftX(), ps4.getLeftYS(), ps4.getRightX(), 0.0);
    robotDrive.driveCartesian(xbox.getLeftX(), xbox.getLeftY(), xbox.getRightX(), 0.0);

    // Raise the robot climber to reach the rung by pressing the green triangle button
    if (xbox.getYButtonPressed())
    {
      climberMotor.set(.2);
    }
    if (xbox.getYButtonPressed())
    {
      climberMotor.stopMotor();
    }

    // CLIMBER SYSTEM

    // Lower the robot climber to pull the robot up the rung by pressing
    // the blue X (Cross) button.
    if (xbox.getAButtonPressed())
    {
      climberMotor.set(-.2);
    }
    if (xbox.getAButtonReleased())
    {
      climberMotor.stopMotor();
    }

    // GAME PIECE INTAKE SYSTEM

    // Retrieve game pieces - Square
    if (xbox.getXButtonPressed())
    {
      intakeWheelsMotor.set(1.0);
    }
    if (xbox.getXButtonReleased())
    {
      intakeWheelsMotor.stopMotor();
    }

    // Release game pieces - Circle
    if (xbox.getBButtonPressed())
    {
      intakeWheelsMotor.set(1.0);
    }
    if (xbox.getBButtonReleased())
    {
      intakeWheelsMotor.stopMotor();
    }

    // INTAKE ARM SYSTEM

    // Raise the arm - Right trigger
    if (xbox.getRightBumperPressed())
    {
      intakeWheelsMotor.set(.2);
    }
    if (xbox.getRightBumperReleased())
    {
      intakeWheelsMotor.stopMotor();
    }

    // Lower the Arm - Left trigger
    if (xbox.getLeftBumperPressed())
    {
      intakeArmMotor.set(-.2);
    }
    if (xbox.getLeftBumperReleased())
    {
      intakeArmMotor.stopMotor();
    }
  }
}
