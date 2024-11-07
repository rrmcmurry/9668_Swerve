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
  Double x;
  Double y;
  Double z;
  boolean fieldRelative;
  boolean rateLimit;

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();

  // The driver's controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  
  // Network Tables
  NetworkTable visiontable;
  DoubleSubscriber visionsubx;
  DoubleSubscriber visionsuby;
  DoubleSubscriber visionsubz;


  @Override
  public void robotInit() {
    // Subscribe to Network Table Vision 
    visiontable = NetworkTableInstance.getDefault().getTable("Vision");
    visionsubx = visiontable.getDoubleTopic("X_Axis").subscribe(0.00);
    visionsuby = visiontable.getDoubleTopic("Y_Axis").subscribe(0.00);
    visionsubz = visiontable.getDoubleTopic("Z-Axis").subscribe(0.00);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {

    // Get control values from network tables
    x = MathUtil.applyDeadband(visionsubx.get(), OIConstants.kDriveDeadband);
    y = MathUtil.applyDeadband(visionsuby.get(), OIConstants.kDriveDeadband);
    z = MathUtil.applyDeadband(visionsubz.get(), OIConstants.kDriveDeadband);

    // Send controls to swerve drive
    swerveDrive.drive(y,x,z, false, true);

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
    
    // X button - Toggles field relative
    if (controller.getXButtonPressed()) {
      fieldRelative = !fieldRelative;      
    }
    
    // Start button - Toggles the use of rate limits
    if (controller.getStartButtonPressed()) {
      rateLimit = !rateLimit;
    }

    // Get control values from the controller
    x = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    y = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
    z = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);

    // Send controls to swerve drive
    swerveDrive.drive(y,x,z, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}