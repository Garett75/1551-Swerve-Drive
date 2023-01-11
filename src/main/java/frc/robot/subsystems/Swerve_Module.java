// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//TODO: as this program has been updated for 2023 the support for third party libraries(CTRE in this case) must be reinstalled. directions here: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html
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

    drive_motor.configFactoryDefault();//returns motor to factory defaults
    drive_motor.setSensorPhase(true);//Note from Rob-I believe this is backwards, if I am right, this makes positive power out result in negative speed and distance, explains why PID fails
    drive_motor.config_kP(0, Constants.SWERVE_MODULE_DRIVE_P);
    drive_motor.config_kI(0, Constants.SWERVE_MODULE_DRIVE_I);
    drive_motor.config_kD(0, Constants.SWERVE_MODULE_DRIVE_D);
    drive_motor.config_kF(0, 1023.0/Constants.MAX_VELOCITY/Constants.METERS_PER_PULSE);
    drive_motor.setNeutralMode(NeutralMode.Coast);//sets motor to coast mode

    rotation_motor.configFactoryDefault();//returns motor to factory defaults
    rotation_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);//This sets up sensor 1 as an Absolute encoder 0-360 range
    rotation_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);//This sets up sensor 0 as an Relative encoder
    rotation_motor.configFeedbackNotContinuous(true, 10);
    rotation_motor.setInverted(false);
    rotation_motor.setSensorPhase(false);
    rotation_motor.config_kP(0, Constants.SWERVE_MODULE_ROTATION_P);
    rotation_motor.config_kI(0, Constants.SWERVE_MODULE_ROTATION_I);
    rotation_motor.config_kD(0, Constants.SWERVE_MODULE_ROTATION_D);
    rotation_motor.config_IntegralZone(0, Constants.SWERVE_MODULE_ROTATION_I_ZONE);
    rotation_motor.setNeutralMode(NeutralMode.Coast);//sets motor to coast mode

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

  /**
   * Returns the current state of the swerve module 
   * as a SwerveModuleState. The speed of the module 
   * should be in m/s and the rotational position is 
   * in the form of a Rotation2d object.
   * 
   * This is no longer used in SwerveOdometry
   * 
   * @return a SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAbsoluteRotation());
  }

  //TODO:create a public method that returns SwerveModulePosition, and takes no arguement. 
  //SwerveModulePosition is constructed of the current DriveMotor encoder position and the getAbsoluteRotation()
  //i.e. new SwerveModulePosition( driveMotor.getSelected sensor position, getAbsoluteRotation())

  /**
   * get the current velocity of the drive motor
   * @return the drive motor's speed in m/s
   */
  public double getDriveVelocity(){
    return drive_motor.getSelectedSensorVelocity() * Constants.METERS_PER_PULSE * 10;
  }

  /**
   * The Mag Encoder reads the absolute rotational position
   * of the module. This method returns that positon in a 
   * Rotation2d object.
   * 
   * @return the position of the module as a Rotation2d object
   */
  public Rotation2d getAbsoluteRotation(){
    double rot = (rotation_motor.getSelectedSensorPosition(1) * Constants.DEGREES_PER_PULSE) -180 - rotationOffset;
    // if(rot < -180){
    //   rot += 360;
    // }else if(rot > 180){
    //   rot -= 360;
    // }
    return Rotation2d.fromDegrees(rot);
  }

   /**
     * This is a method meant for testing by getting the relative 
     * count from the Mag Encoder. This is relative, and is not 
     * easily translated to a specific rotational position of the 
     * swerve module. It is continuous, and does not loop back 
     * around.
     * 
     * @return the encoder count in degrees from boot position
     */
  public double getRelativeRotation(){
    return rotation_motor.getSelectedSensorPosition(0) * Constants.DEGREES_PER_PULSE;
    // return Rotation2d.fromDegrees(rotation_motor.getSelectedSensorPosition(0) * Constants.DEGREES_PER_PULSE);
  }

  /**
   * The method to set the module to a position and speed. 
   * This method does the opitimization internally. The 
   * speed should be from -1.0 to 1.0 if isVeloMode is false, 
   * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
   * isVeloMode is true.
   * 
   * @param state SwerveModuleState, target for the module
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void setState(SwerveModuleState state, boolean velocityMode){

    //check if the requested speed is below a threshold
    if(Math.abs(state.speedMetersPerSecond) < 0.1){
      // if speed is small, force the module to not rotate
      stopMotors();
      return;
    }
    
    //get the target angle for this module in degrees
    double angle = state.angle.getDegrees();

    //take the target angle and check if things would be faster if you swap the direction of the motor
    SwerveModuleState targetState = SwerveModuleState.optimize(
      new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(angle)), getState().angle);
  
    //set the motor to either a dutyCycle or Velocity. the later requires tuned PID on the drive motor.
    if(velocityMode){
      drive_motor.set(ControlMode.Velocity, targetState.speedMetersPerSecond/Constants.METERS_PER_PULSE/10);
    }else{
      drive_motor.set(ControlMode.PercentOutput, targetState.speedMetersPerSecond);
    }

    //set the pid controller for the target angle
    setRotationPosition(targetState.angle.getDegrees());
  }

  /**
   * Sets the rotation of the swerve module to a specified angle in degrees
   * 
   * @param degrees the angle the module will be set to
   */
  public void setRotationPosition(double degrees) {
    //First we geet the current absolute rotational position of the module
    double currentPosAbs = getAbsoluteRotation().getDegrees();

    //Next we get the current reading of the relative encodder
    double currentPosRel = getRelativeRotation();

    //next we get the difference between the targeted angle and the current absolute rotational position
    double delta = degrees - currentPosAbs;

    //Now we check to see if detla is rotating the motor more than 180  which is the wrong way around the circle
    if (delta > 180) {
        delta -= 360;
    } else if (delta < -180) {
        delta += 360;
    }

    //set the motor's PID controller to from the current position by delta
    rotation_motor.set(ControlMode.Position, (currentPosRel + delta) / Constants.DEGREES_PER_PULSE);
  }

  /**
   * This method dictates the zero position as the 
   * current position of the module. This is 
   * accomplished by taking the current value as an 
   * offset, and then saving that offset in WPILibs 
   * Preferences system. This is done because the 
   * mechanical zero is hard to move on a MagEncoder
   * setup.
   */
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

  /**
   * Set the speed of the drive motor in percent duty cycle, good for testing
   * 
   * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
   *                  percent duty cycle
   */
  public void setDrivePercentOutput(double percent){
    drive_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * This method is used to stop the module completely. The drive 
   * motor is switched to percent voltage and and output of 0.0 
   * percent volts. Both motor's control mode is set to 
   * DutyCyclevoltage, and output of 0.0% output.
   */
  public void stopMotors(){
    rotation_motor.set(ControlMode.PercentOutput, 0);
    drive_motor.set(ControlMode.PercentOutput, 0);
  }

}
