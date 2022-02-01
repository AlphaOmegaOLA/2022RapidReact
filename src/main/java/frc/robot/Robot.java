// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This project is under version control using Alpha Omega's Github
// account at https://github.com/AlphaOmegaOLA/2022RapidReact

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// Xbox joystick
import edu.wpi.first.wpilibj.XboxController;
// Try the fancy Mecanum Drive this year
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// Drive System Motor Controller
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Climber/Intake Arm/Intake Wheel Motor Controller
import edu.wpi.first.wpilibj.motorcontrol.Talon;
// Webcam
import edu.wpi.first.cameraserver.CameraServer;
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
  private final Talon climberMotor = new Talon(4);

  // Intake Arm Motor Controller
  private final Talon intakeArmMotor = new Talon(5);

  // Intake Wheels Motor Controller
  private final Talon intakeWheelsMotor = new Talon(6)

  // Mecanum Drive System
  private MecanumDrive robotDrive;
  // XBox controller
  private final XboxController xbox = new XboxController(0);
  // The gyro
  //private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();


  @Override
  public void robotInit() 
  {
    // The Mecanum Drive requires all 4 motors to operate independently
    robotDrive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    // Send camera feed to dashboard
    CameraServer.startAutomaticCapture();

    // Limit switch for the hanger
    //DigitalInput toplimitSwitch = new DigitalInput(0);
  }

  @Override
  public void teleopPeriodic() 
  {
    // Mecanum drive.  The last argument is "gyroangle" to set field-oriented
    // vs. drive oriented steering. Still need to figure that out.
    robotDrive.driveCartesian(xbox.getLeftX(), xbox.getLeftY(), xbox.getRightX(), 0.0);

    

    /*
    if (speed > 0) 
    {
      if (toplimitSwitch.get()) 
      {
        // We are going up and top limit is tripped so stop
        motor.set(0);
      } 
      else 
      {
        // We are going up but top limit is not tripped so go at joystic speed
        motor.set(speed);
      }
    }
    */
  }
}
