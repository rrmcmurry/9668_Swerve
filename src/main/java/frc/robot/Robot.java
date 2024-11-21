// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
  boolean teleautonomous;

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
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    fieldRelative = true;
    rateLimit = false;
    teleautonomous = false;

  }

  @Override
  public void teleopPeriodic() {
    
    // X button - sets wheels in an X formation
    if (controller.getXButtonPressed() ) {
      swerveDrive.setX();
    }

    // Y button - Resets heading and sets pose to 0,5
    if (controller.getYButtonPressed() ) {
      swerveDrive.zeroHeading();      
      swerveDrive.setPose(0,5,0);
    }
    
    // Back button - Toggles field relative    
    if (controller.getBackButtonPressed()) {
      fieldRelative = !fieldRelative;      
    }
    
    // Start button - Toggles autonomous mode
    if (controller.getStartButtonPressed()) {      
      teleautonomous = !teleautonomous;
    }

    // Get control values from the controller
    strafe = MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(controller.getLeftY() * -1.0, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);

    if (teleautonomous) {
      // Combine with controller values from network tables
      strafe = strafe + MathUtil.applyDeadband(networkcontroller_leftJoyX.get(), OIConstants.kDriveDeadband);
      forward = forward + MathUtil.applyDeadband(networkcontroller_leftJoyY.get(), OIConstants.kDriveDeadband);
      rotate = rotate + MathUtil.applyDeadband(networkcontroller_rightJoyX.get(), OIConstants.kDriveDeadband);
    }
    

    // Send controller values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}