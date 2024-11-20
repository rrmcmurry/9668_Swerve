// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;




public class Robot extends TimedRobot {

  // Drive command variables
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();

  // The driver's controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  
  // Network Tables
  NetworkTable NetworkController;
  DoubleSubscriber networkcontroller_leftJoyX;
  DoubleSubscriber networkcontroller_leftJoyY;
  DoubleSubscriber networkcontroller_rightJoyX;


  @Override
  public void robotInit() {
    NetworkTableInstance.getDefault().startServer();
    NetworkController = NetworkTableInstance.getDefault().getTable("NetworkController");
    networkcontroller_leftJoyX = NetworkController.getDoubleTopic("leftJoyX").subscribe(0.00);
    networkcontroller_leftJoyY = NetworkController.getDoubleTopic("leftJoyY").subscribe(0.00);
    networkcontroller_rightJoyX = NetworkController.getDoubleTopic("rightJoyX").subscribe(0.00);
  }

  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {  
    // Initially using field relative with rate limits
    fieldRelative = true;
    rateLimit = false;
  }

  @Override
  public void autonomousPeriodic() {

    // Get control values from network tables
    strafe = MathUtil.applyDeadband(networkcontroller_leftJoyX.get(), OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(networkcontroller_leftJoyY.get(), OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(networkcontroller_rightJoyX.get(), OIConstants.kDriveDeadband);

    // Send control values to swerve drive
    swerveDrive.drive(forward,strafe,rotate, fieldRelative, rateLimit);

  }

  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    fieldRelative = true;
    rateLimit = false;

  }

  @Override
  public void teleopPeriodic() {
    
    // Right bumper - sets wheels in an X formation
    if (controller.getRightBumperPressed() ) {
      swerveDrive.setX();
    }

    // Y button - Resets gyroscope heading
    if (controller.getYButtonPressed() ) {
      swerveDrive.zeroHeading();
    }
    
    // B button - Resets pose
    if (controller.getBButtonPressed() ) {
      Pose2d newpose = new Pose2d();
      swerveDrive.resetOdometry(newpose);
    }
    
    // X button - Toggles field relative
    if (controller.getXButtonPressed()) {
      fieldRelative = !fieldRelative;      
    }
    
    // Start button - Toggles the use of rate limits
    if (controller.getStartButtonPressed()) {
      rateLimit = !rateLimit;
    }

    // Get control values from the controller
    strafe = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    forward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
    rotate = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);

    // Send control values to swerve drive
    swerveDrive.drive(forward,strafe,rotate, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}