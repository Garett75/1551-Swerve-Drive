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
    modules = new Swerve_Module[]{
      new Swerve_Module(Constants.FRONT_LEFT_DRIVE_ID, Constants.FRONT_LEFT_ROTATION_ID, 0),
      new Swerve_Module(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_ROTATION_ID, 1),
      new Swerve_Module(Constants.REAR_LEFT_DRIVE_ID, Constants.REAR_LEFT_ROTATION_ID, 2),
      new Swerve_Module(Constants.REAR_RIGHT_DRIVE_ID, Constants.REAR_RIGHT_ROTATION_ID, 3), 
    };
    kinematics = new SwerveDriveKinematics(Constants.FRONT_LEFT_OFFSET, Constants.FRONT_RIGHT_OFFSET, 
      Constants.REAR_LEFT_OFFSET, Constants.REAR_RIGHT_OFFSET);

    imu = new ADIS16470_IMU();

    odometry = new SwerveDriveOdometry(kinematics, getIMUAngle());

    SmartDashboard.putData("ZeroModules", zeroModulesCommand());

    // setDefaultCommand(new RunCommand(() -> {
    //   for(Swerve_Module module : modules){
    //     module.setDrivePercentOutput(-RobotContainer.getLeftStickY());
    //   }
    // }, this));
    setDefaultCommand(
    new RunCommand(() -> {
      driveFieldRelative(new ChassisSpeeds(
        -RobotContainer.getLeftStickY() * .5,
        -RobotContainer.getLeftStickX() * .5,
        -RobotContainer.getRightStickX() * .5
      ));
    }, this) );
 }

  @Override
  public void periodic() {
    odometry.update(getIMUAngle(), getStates());
    // This method will be called once per scheduler run
    for(Swerve_Module module : modules){
      SmartDashboard.putNumber("Swerve#"+module.module_id,module.getAbsoluteRotation().getDegrees());
    }
  }

  public Rotation2d getIMUAngle(){
    return Rotation2d.fromDegrees(imu.getAngle());
  }

  public void zeroIMUAngle(){
    imu.reset();
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for(Swerve_Module module : modules){
      states[module.module_id] = module.getState();
    }
    return states;
  }

  public void resetOdometry(Pose2d pos) {
    odometry.resetPosition(pos, getIMUAngle());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] states){
    for(Swerve_Module module : modules){
      module.setState(states[module.module_id],false);//TODO: Change to true once PID fixed
    }
  }

  
  public void setModuleAngle(double angle){
    for(Swerve_Module module : modules){
      module.setRotationPosition(angle);
    }
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fieldRelativeSpeeds.vxMetersPerSecond, 
      fieldRelativeSpeeds.vyMetersPerSecond, 
      fieldRelativeSpeeds.omegaRadiansPerSecond, 
      getIMUAngle());
    
    driveRobotRelative(speeds);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.MAX_VELOCITY);

    setModuleStates(targetStates);
  }

  public CommandBase zeroModulesCommand() {
    SequentialCommandGroup zeroGroup = new SequentialCommandGroup();

    for(Swerve_Module module : modules){
      zeroGroup.addCommands(
        new InstantCommand(() -> {
          module.stopMotors();
        }),
        new RunCommand(() -> {
          module.setRotationPercentOutput(RobotContainer.getRightStickX() / 2);
        }).raceWith(new WaitUntilCommand(RobotContainer::getRightStickPressed)),
        new InstantCommand(() -> {
          module.setRotationPercentOutput(0);
          module.zeroRotation();
          module.stopMotors();
        })
      );
    }

    zeroGroup.addRequirements(this);
    return zeroGroup;
  }
}
