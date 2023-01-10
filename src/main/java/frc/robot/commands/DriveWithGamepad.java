// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Allows the robot to Drive with the Gamepad */

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveWithGamepad extends CommandBase {

  private boolean isFieldRelative;
  private boolean isCounterRotationOn;
  Drive drive;
  ProfiledPIDController rotationController;
  double currentAngle;
  double previousGyroAngle;
  final double trainingWheels = 0.85;

  /**
   * Command for controlling the robot with the gamepad.
   * 
   * Runs indefinitely.
   */
  public DriveWithGamepad(Drive drive,boolean isFieldRelative, boolean isCounterRotationOn) {
    this.drive = drive;
    addRequirements(drive);
    rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
        Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
        new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    // rotationController.enableContinuousInput(-180, 180);
    this.isFieldRelative = isFieldRelative;
    this.isCounterRotationOn = isCounterRotationOn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = drive.getIMUAngle().getDegrees();
    previousGyroAngle = currentAngle;
    rotationController.reset(currentAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightStickX = -RobotContainer.getLeftStickY() * .5;
    double leftStickY = -RobotContainer.getLeftStickX() * .5;
    double leftStickX = -RobotContainer.getRightStickX() * .5;

    double rotationOutput = rightStickX;

    if (isCounterRotationOn && Math.abs(rotationOutput) < 0.05) {
      if (Math.abs(drive.getIMUAngle().getDegrees() - previousGyroAngle) < 1.8) {
        rotationOutput = rotationController.calculate(drive.getIMUAngle().getDegrees(), currentAngle);
      } else {
        currentAngle = drive.getIMUAngle().getDegrees();
        rotationController.reset(currentAngle);
      }
    } else {
      currentAngle = drive.getIMUAngle().getDegrees();
      rotationController.reset(currentAngle);
      rotationOutput *= Constants.DRIVE_MAX_ANGULAR_VELOCITY;
    }
    
    previousGyroAngle = drive.getIMUAngle().getDegrees();
    double xVel = -leftStickY * Constants.MAX_VELOCITY * trainingWheels;
    double yVel = leftStickX * Constants.MAX_VELOCITY * trainingWheels;

    Translation2d corrections = new Translation2d(xVel, yVel);

    if(isFieldRelative){
      drive.driveFieldRelative(new ChassisSpeeds(corrections.getX(), corrections.getY(), rotationOutput));
    }else{
      drive.driveRobotRelative(new ChassisSpeeds(corrections.getX(), corrections.getY(), rotationOutput));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}