// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Swerve_Module;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive();
  private static PS4Controller controller = new PS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // drive.setDefaultCommand(new DriveWithGamepad(drive, true, true));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller, PS4Controller.Button.kCross.value).whenActive(new InstantCommand(() -> {
      drive.setModuleAngle(0);
    }, drive));
    new JoystickButton(controller, PS4Controller.Button.kCircle.value).whenActive(new InstantCommand(() -> {
      drive.setModuleAngle(45);
    }, drive));
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value).whenActive(new DriveZeroGyro(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public static double getRightStickX(){
    return controller.getRightX();
  }

  public static double getLeftStickX(){
    return controller.getLeftX();
  }

  public static double getLeftStickY(){
    return controller.getLeftY();
  }

  public static boolean getRightStickPressed(){
    return controller.getR3ButtonPressed();
  }
}
