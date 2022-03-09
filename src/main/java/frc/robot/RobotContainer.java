// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AuxArmMoveCommand;
import frc.robot.commands.ConveyorAutomationCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ShooterHighCloseCommand;
import frc.robot.commands.ShooterHighFarCommand;
import frc.robot.commands.ShooterLowCloseCommand;
import frc.robot.commands.ShooterLowFarCommand;
import frc.robot.commands.SushiMoveCommand;
import frc.robot.commands.TestShooterSpeed;
import frc.robot.commands.climberMoveCommand;
import frc.robot.commands.intakeBallCommand;
import frc.robot.commands.outtakeBallCommand;
import frc.robot.commands.toggleAngleCommand;
import frc.robot.commands.toggleIntakeFlipCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Limelight.LedMode;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxController primaryController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  //Pneumatics
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  


  //Subsystems
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final ClimberSubsystem climbSubsystem = new ClimberSubsystem();
  private final AuxArmSubsystem auxArmSubsystem = new AuxArmSubsystem();
  private VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    pcmCompressor.enableAnalog(80, 120);
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> modifyAxis(primaryController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> modifyAxis(primaryController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> modifyAxis(primaryController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));


    //SLOW MODE WHEN SET TO .3, CHANGE TO A PRECISION MODE LATER, THIS IS FOR LIMELIGHT SHOOTING
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> modifyAxis(primaryController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 1,
      () -> modifyAxis(primaryController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 1,
      () -> ((modifyAxis(primaryController.getRightX()) + visionSubsystem.getLimelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * 1
    ));
    //pcmCompressor.disable();

    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(conveyorSubsystem);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(climbSubsystem);
    CommandScheduler.getInstance().registerSubsystem(auxArmSubsystem);

    // Shooter Subsystem 
    CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, new TestShooterSpeed(shooterSubsystem));
    CommandScheduler.getInstance().setDefaultCommand(auxArmSubsystem, new AuxArmMoveCommand(auxArmSubsystem, () -> -modifyAxis(primaryController.getRightTriggerAxis()), () -> -modifyAxis(primaryController.getLeftTriggerAxis())));
    CommandScheduler.getInstance().setDefaultCommand(conveyorSubsystem, new ConveyorAutomationCommand(conveyorSubsystem, shooterSubsystem, feederSubsystem,intakeSubsystem));

    CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new SushiMoveCommand(feederSubsystem, () -> -modifyAxis(operatorController.getLeftY())));
    CommandScheduler.getInstance().setDefaultCommand(climbSubsystem, new climberMoveCommand(climbSubsystem, () -> -modifyAxis(operatorController.getRightTriggerAxis()), () -> -modifyAxis(operatorController.getLeftTriggerAxis())));
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    //new Button(primaryController::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new Button(operatorController::getLeftBumper).whileHeld(new outtakeBallCommand(intakeSubsystem));
    new Button(operatorController::getRightBumper).whileHeld(new intakeBallCommand(intakeSubsystem));
    new Button(operatorController::getStartButton).toggleWhenPressed(new toggleIntakeFlipCommand(intakeSubsystem));
    new Button(operatorController::getBackButton).toggleWhenPressed(new toggleAngleCommand(shooterSubsystem));

    new Button(operatorController::getYButton).whenPressed(new ShooterHighFarCommand(shooterSubsystem, visionSubsystem, conveyorSubsystem, feederSubsystem, intakeSubsystem));
    new Button(operatorController::getXButton).whenPressed(new ShooterLowFarCommand(shooterSubsystem, visionSubsystem, conveyorSubsystem, feederSubsystem, intakeSubsystem));
    new Button(operatorController::getBButton).whenPressed(new ShooterHighCloseCommand(shooterSubsystem, visionSubsystem, conveyorSubsystem, feederSubsystem, intakeSubsystem));
    new Button(operatorController::getAButton).whenPressed(new ShooterLowCloseCommand(shooterSubsystem, visionSubsystem, conveyorSubsystem, feederSubsystem, intakeSubsystem));

    new Button(primaryController::getRightBumper).whenPressed(new RunCommand(visionSubsystem::setLimelightOffset).beforeStarting(new InstantCommand(() -> visionSubsystem.setLedMode(LedMode.ON))));
    new Button(primaryController::getLeftBumper).whenPressed(new RunCommand(() -> visionSubsystem.setLimelightOffset(0)).beforeStarting(new InstantCommand(() -> visionSubsystem.setLedMode(LedMode.OFF))));
    //new Button(primaryController::getXButton).whenPressed(new rotateToVisionTargetCommand(m_drivetrainSubsystem, VisionSubsystem,)))
    // new Button(operatorController::getBButton).whileHeld(new conveyorCommand(conveyorSubsystem));
    // new Button(operatorController::getLeftBumper).whileHeld(new sushiCommand(feederSubsystem));
    // new Button(operatorController::getRightBumper).whenPressed(new shooterCommand(shooterSubsystem));
    // new Button(operatorController::getXButton).whileHeld(new lowerIntakeCommand(intakeSubsystem));
    // new Button(operatorController::getYButton).whileHeld(new raiseIntakeCommand(intakeSubsystem));
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
