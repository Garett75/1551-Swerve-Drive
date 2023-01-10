// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve_Module {
  public final TalonFX drive_motor;
  public final TalonSRX rotation_motor;
  public final int module_id;

  public double lastRotation; 
  public double rotationOffset;

  /** Creates a new ExampleSubsystem. */
  public Swerve_Module(int driveID, int rotationID, int moduleID) {
    drive_motor = new TalonFX(driveID);
    rotation_motor = new TalonSRX(rotationID);
    module_id = moduleID;
    rotationOffset = Preferences.getDouble(moduleID + "rotationOffset", 0);

    drive_motor.configFactoryDefault();
    drive_motor.setSensorPhase(true);
    drive_motor.config_kP(0, Constants.SWERVE_MODULE_DRIVE_P);
    drive_motor.config_kI(0, Constants.SWERVE_MODULE_DRIVE_I);
    drive_motor.config_kD(0, Constants.SWERVE_MODULE_DRIVE_D);
    drive_motor.config_kF(0, 1023.0/Constants.MAX_VELOCITY/Constants.METERS_PER_PULSE);
    drive_motor.setNeutralMode(NeutralMode.Coast);

    rotation_motor.configFactoryDefault();
    rotation_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
    rotation_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rotation_motor.configFeedbackNotContinuous(true, 10);
    rotation_motor.setInverted(false);
    rotation_motor.setSensorPhase(false);
    rotation_motor.config_kP(0, Constants.SWERVE_MODULE_ROTATION_P);
    rotation_motor.config_kI(0, Constants.SWERVE_MODULE_ROTATION_I);
    rotation_motor.config_kD(0, Constants.SWERVE_MODULE_ROTATION_D);
    rotation_motor.config_IntegralZone(0, Constants.SWERVE_MODULE_ROTATION_I_ZONE);
    rotation_motor.setNeutralMode(NeutralMode.Coast);

    lastRotation = getAbsoluteRotation().getDegrees();
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // @Override
  // public void simulationPeriodic() {
  //   // This method will be called once per scheduler run during simulation
  // }

public SwerveModuleState getState() {
  return new SwerveModuleState(getDriveVelocity(), getAbsoluteRotation());
}

public double getDriveVelocity(){
  return drive_motor.getSelectedSensorVelocity() * Constants.METERS_PER_PULSE * 10;
}

public Rotation2d getAbsoluteRotation(){
  double rot = (rotation_motor.getSelectedSensorPosition(1) * Constants.DEGREES_PER_PULSE) -180 - rotationOffset;
  // if(rot < -180){
  //   rot += 360;
  // }else if(rot > 180){
  //   rot -= 360;
  // }
  return Rotation2d.fromDegrees(rot);
}

public double getRelativeRotation(){
  return rotation_motor.getSelectedSensorPosition(0) * Constants.DEGREES_PER_PULSE;
  // return Rotation2d.fromDegrees(rotation_motor.getSelectedSensorPosition(0) * Constants.DEGREES_PER_PULSE);
}

public void setState(SwerveModuleState state, boolean velocityMode){
  
  if(Math.abs(state.speedMetersPerSecond) < 0.1){
    stopMotors();
    return;
  }
  
  double angle = state.angle.getDegrees();
  SwerveModuleState targetState = SwerveModuleState.optimize(
    new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(angle)), getState().angle);
 
  if(velocityMode){
    drive_motor.set(ControlMode.Velocity, targetState.speedMetersPerSecond/Constants.METERS_PER_PULSE/10);
  }else{
    drive_motor.set(ControlMode.PercentOutput, targetState.speedMetersPerSecond);
  }

  setRotationPosition(targetState.angle.getDegrees());
}
/**
 * Sets the rotation of the swerve module to a specified angle in degrees
 * 
 * @param degrees the angle the module will be set to
 */
public void setRotationPosition(double degrees) {
  double currentPosAbs = getAbsoluteRotation().getDegrees();
  double currentPosRel = getRelativeRotation();
  double delta = degrees - currentPosAbs;
  if (delta > 180) {
      delta -= 360;
  } else if (delta < -180) {
      delta += 360;
  }
  rotation_motor.set(ControlMode.Position, (currentPosRel + delta) / Constants.DEGREES_PER_PULSE);
}

public void zeroRotation(){
  rotationOffset = (rotation_motor.getSelectedSensorPosition(1)*Constants.DEGREES_PER_PULSE) - 180;//getAbsoluteRotation().getDegrees();
  // if(rotationOffset < 0){
  //   rotationOffset += 360;
  // }else if(rotationOffset > 360){
  //   rotationOffset -= 360;
  // }
  Preferences.setDouble(module_id + "rotationOffset", rotationOffset);
}

public void setRotationPercentOutput(double percent){
  rotation_motor.set(ControlMode.PercentOutput, percent);
}

public void setDrivePercentOutput(double percent){
  drive_motor.set(ControlMode.PercentOutput, percent);
}

public void stopMotors(){
  rotation_motor.set(ControlMode.PercentOutput, 0);
  drive_motor.set(ControlMode.PercentOutput, 0);
}

}
