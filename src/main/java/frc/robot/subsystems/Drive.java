// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithGamepad;

public class Drive extends SubsystemBase {
  SwerveDriveOdometry odometry;
  SwerveDriveKinematics kinematics;
  Swerve_Module [] modules;
  ADIS16470_IMU imu;

  /** Creates a new Drive. */
  public Drive() {
    //construct each of the four swerve modules, as objects in an array
    modules = new Swerve_Module[]{
      new Swerve_Module(Constants.FRONT_LEFT_DRIVE_ID, Constants.FRONT_LEFT_ROTATION_ID, 0),
      new Swerve_Module(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_ROTATION_ID, 1),
      new Swerve_Module(Constants.REAR_LEFT_DRIVE_ID, Constants.REAR_LEFT_ROTATION_ID, 2),
      new Swerve_Module(Constants.REAR_RIGHT_DRIVE_ID, Constants.REAR_RIGHT_ROTATION_ID, 3), 
    };

    //use the known positions of the swerve modules around the center of the robot, to make kinematics class
    kinematics = new SwerveDriveKinematics(Constants.FRONT_LEFT_OFFSET, Constants.FRONT_RIGHT_OFFSET, 
      Constants.REAR_LEFT_OFFSET, Constants.REAR_RIGHT_OFFSET);

    imu = new ADIS16470_IMU();

    //TODO: The SwerveDriveOdometry constructor needs an array of SwerveModulePositions third argument. SwerveModulePositions are objects that should be pulled from the SwerveModule objects, TODO below and in swerve module explains. 
    odometry = new SwerveDriveOdometry(kinematics, getIMUAngle());

    //Send a button to the SmartDashboard that will run the swerve module zeroing command
    SmartDashboard.putData("ZeroModules", zeroModulesCommand());

    //The following is a command to set the drive motor outputs on all the modules for testing, it is written inline
    // setDefaultCommand(new RunCommand(() -> {
    //   for(Swerve_Module module : modules){
    //     module.setDrivePercentOutput(-RobotContainer.getLeftStickY());
    //   }
    // }, this));

    //The following is a simplified command for driving, it is written inline
    setDefaultCommand(
    new RunCommand(() -> {
      driveFieldRelative(new ChassisSpeeds(
        -RobotContainer.getLeftStickY() * .5,
        -RobotContainer.getLeftStickX() * .5,
        -RobotContainer.getRightStickX() * .5
      ));
    }, this) );
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    //TODO:use the getPositions() rather than getStates() once getPositions() is written
    odometry.update(getIMUAngle(), getStates());
    
    //put the swerve modules rotational positions on the SmartDashboard
    for(Swerve_Module module : modules){
      SmartDashboard.putNumber("Swerve#"+module.module_id,module.getAbsoluteRotation().getDegrees());
    }
  }

  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This is an object that represents the rotation orientation 
   * of the robot.
   * 
   * @return a Rotation2d object
   */
  public Rotation2d getIMUAngle(){
    return Rotation2d.fromDegrees(imu.getAngle());
  }

  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void zeroIMUAngle(){
    imu.reset();
  }

  /**
   * Constructs an array of SwerveModuleStates, representing 
   * the current SwerveModuleState of each module
   * 
   * 
   * @return
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for(Swerve_Module module : modules){
      states[module.module_id] = module.getState();
    }
    return states;
  }

  //TODO:using the same design/format as getStates() method, create getPositions() method, that returns an array of SwerveModulePosition[] insead of SwerveModuleState[]

  /**
   * Sets current position in the odometry class
   * 
   * @param pose new current position as a Pose2d object
   */
  public void resetOdometry(Pose2d pos) {
    //TODO:as of 2023, this method requires things in a different order and requires getPositions() as well
    odometry.resetPosition(pos, getIMUAngle());//The order should be getIMUAngle(), getPositions(), pos
  }

  /**
   * Pull the current Position from the odometry 
   * class, this should be in meters.
   * 
   * @return a Pose2d representing the current position
   */
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  /**
   * set each module based on the SwerveModuleState array input.
   * 
   * @param states
   */
  public void setModuleStates(SwerveModuleState[] states){
    for(Swerve_Module module : modules){
      module.setState(states[module.module_id],false);//TODO: Change to true once PID fixed
    }
  }

  /**
   * set all modules to a specific angle.
   * @param angle a rotational position in degrees
   */
  public void setModuleAngle(double angle){
    for(Swerve_Module module : modules){
      module.setRotationPosition(angle);
    }
  }

  /**
   * Drive the robot so that all directions are independent 
   * of the robots orientation (rotation) 
   * 
   * @param fieldRelativeSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
    //the following takes fieldrelative ChassiSpeeds and converts to robot relative ChassiSpeeds
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fieldRelativeSpeeds.vxMetersPerSecond, 
      fieldRelativeSpeeds.vyMetersPerSecond, 
      fieldRelativeSpeeds.omegaRadiansPerSecond, 
      getIMUAngle());
    
    //put converted robot relative ChassiSpeeds to driveRobotRelative
    driveRobotRelative(speeds);
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   *
   * @param robotRelativeSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    //convert the speed we want the robot to drive, to what speeds and positons the modules need to be
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);

    //desaturate(normalize) so that no module is driven above maximum velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.MAX_VELOCITY);

    //set the modules to the required positions and speeds
    setModuleStates(targetStates);
  }

  /**
   * a method that generates a command group. The command group 
   * gives the driver the control over individual module rotation 
   * one at a time. The RightJoystick allows the driver to slowwly
   * rotate a module, once pressed, that position will be the new 
   * zero position of the module. and control over the next module 
   * will be given.
   * 
   * @return a command
   */
  public CommandBase zeroModulesCommand() {
    SequentialCommandGroup zeroGroup = new SequentialCommandGroup();

    for(Swerve_Module module : modules){
      zeroGroup.addCommands(
        new InstantCommand(() -> {
          module.stopMotors();// a command to stop all the motors
        }),
        new RunCommand(() -> {
          module.setRotationPercentOutput(RobotContainer.getRightStickX() / 2);//a command that allows a motors rotation
        }).raceWith(new WaitUntilCommand(RobotContainer::getRightStickPressed)),//a command, racing with previous, to move on when button pressed
        new InstantCommand(() -> {
          module.setRotationPercentOutput(0);
          module.zeroRotation();
          module.stopMotors();
        })//a command that stops the rotation, then zeros the encoder on that module
      );
    }

    zeroGroup.addRequirements(this);
    return zeroGroup;
  }
}
