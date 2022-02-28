// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.conveyorCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.lowerClimber;
import frc.robot.commands.lowerIntake;
import frc.robot.commands.raiseClimber;
import frc.robot.commands.raiseIntake;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.sushiCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController primaryController = new XboxController(0);

  //Pneumatics
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);


  //Subsystems
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final climberSubsystem climbSubsystem = new climberSubsystem();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(primaryController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(primaryController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(primaryController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));


    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(conveyorSubsystem);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(climbSubsystem);
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    //new Button(primaryController::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new Button(primaryController::getAButton).whileHeld(new intakeCommand(intakeSubsystem));
    new Button(primaryController::getBButton).whileHeld(new conveyorCommand(conveyorSubsystem));
    new Button(primaryController::getLeftBumper).whileHeld(new sushiCommand(feederSubsystem));
    new Button(primaryController::getRightBumper).whenPressed(new shooterCommand(shooterSubsystem));
    new Button(primaryController::getXButton).whileHeld(new lowerIntake(intakeSubsystem));
    new Button(primaryController::getYButton).whileHeld(new raiseIntake(intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
