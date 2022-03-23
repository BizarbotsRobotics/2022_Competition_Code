// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AuxArmMoveCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraight1Ball;
import frc.robot.commands.DriveStraight2Ball;
import frc.robot.commands.ShooterHighCloseCommand;
import frc.robot.commands.ShooterHighFarCommand;
import frc.robot.commands.ShooterLowCloseCommand;
import frc.robot.commands.ShooterLowFarCommand;
import frc.robot.commands.alignCommand;
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
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  //private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final ClimberSubsystem climbSubsystem = new ClimberSubsystem();
  private final AuxArmSubsystem auxArmSubsystem = new AuxArmSubsystem();
  private VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  
  private IntakeFeeder intakeFeeder = new IntakeFeeder();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands
   */
  public RobotContainer() {
    //pcmCompressor.enableAnalog(80, 120);
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
    visionSubsystem.setLedMode(LedMode.ON);
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> modifyAxis(primaryController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 1,
      () -> modifyAxis(primaryController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 1,
      () -> ((modifyAxis(primaryController.getRightX()) + visionSubsystem.getLimelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * 1
    ));
    //pcmCompressor.disable();
    //CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    //CommandScheduler.getInstance().registerSubsystem(conveyorSubsystem);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    //CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(climbSubsystem);
    CommandScheduler.getInstance().registerSubsystem(auxArmSubsystem);

    CommandScheduler.getInstance().registerSubsystem(intakeFeeder);
    // Shooter Subsystem 
    //CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, new TestShooterSpeed(shooterSubsystem));
    CommandScheduler.getInstance().setDefaultCommand(auxArmSubsystem, new AuxArmMoveCommand(auxArmSubsystem, () -> -modifyAxis(primaryController.getRightTriggerAxis()), () -> -modifyAxis(primaryController.getLeftTriggerAxis())));

    //CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new SushiMoveCommand(feederSubsystem, () -> -modifyAxis(operatorController.getLeftY())));
    CommandScheduler.getInstance().setDefaultCommand(climbSubsystem, new climberMoveCommand(climbSubsystem, () -> -modifyAxis(operatorController.getRightTriggerAxis()), () -> -modifyAxis(operatorController.getLeftTriggerAxis())));
    // Configure the button bindings
    configureButtonBindings();
    configureSendableChooser();
  }

  private void configureSendableChooser() {
    m_chooser.setDefaultOption("Stay Still", new InstantCommand());
    m_chooser.setDefaultOption("Drive Straight", new DriveStraight(m_drivetrainSubsystem, visionSubsystem, shooterSubsystem, intakeFeeder));
    m_chooser.setDefaultOption("Drive Straight and score 2 balls", new DriveStraight2Ball(m_drivetrainSubsystem, visionSubsystem));
    m_chooser.setDefaultOption("Drive Straight and score 1 balls", new DriveStraight1Ball(m_drivetrainSubsystem, visionSubsystem, intakeFeeder, shooterSubsystem));
    //bm_chooser.setDefaultOption("SCARY TEST", new DriveStraight1Ball(m_drivetrainSubsystem, visionSubsystem));
    SmartDashboard.putData(m_chooser);
}

  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(operatorController::getLeftBumper).whileHeld(new outtakeBallCommand(intakeFeeder));
    new Button(operatorController::getRightBumper).whileHeld(new intakeBallCommand(intakeFeeder));
    new Button(operatorController::getStartButton).toggleWhenPressed(new toggleIntakeFlipCommand(intakeFeeder));
    new Button(operatorController::getBackButton).toggleWhenPressed(new toggleAngleCommand(shooterSubsystem));

    new Button(operatorController::getYButton).whileHeld(new ShooterHighFarCommand(shooterSubsystem, visionSubsystem, intakeFeeder));
    new Button(operatorController::getXButton).whileHeld(new ShooterLowFarCommand(shooterSubsystem, visionSubsystem, intakeFeeder));
    new Button(operatorController::getBButton).whileHeld(new ShooterHighCloseCommand(shooterSubsystem, visionSubsystem, intakeFeeder));
    new Button(operatorController::getAButton).whileHeld(new ShooterLowCloseCommand(shooterSubsystem, intakeFeeder));
    //new Button(primaryController::getBButton).whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));
    new Button(primaryController::getRightBumper).whileHeld(new alignCommand(visionSubsystem, operatorController));
    //new Button(primaryController::getXButton).whenPressed(new rotateToVisionTargetCommand(m_drivetrainSubsystem, VisionSubsystem,)))
    // new Button(operatorController::getBButton).whileHeld(new conveyorCommand(conveyorSubsystem));
    // new Button(operatorController::getLeftBumper).whileHeld(new sushiCommand(feederSubsystem));
    // new Button(operatorController::getRightBumper).whenPressed(new shooterCommand(shooterSubsystem));
    // new Button(operatorController::getXButton).whileHeld(new lowerIntakeCommand(intakeSubsystem));
    // new Button(operatorController::getYButton).whileHeld(new raiseIntakeCommand(intakeSubsystem));


    new Button(() -> operatorController.getLeftTriggerAxis() > .2).whileHeld(new InstantCommand(()->this.shooterSubsystem.shooterStop()));
    new Button(() -> operatorController.getRightTriggerAxis() > .2).whileHeld(new InstantCommand(()->this.shooterSubsystem.shooterStop()));

    new Trigger(() -> (operatorController.getRightY() > 0.2))
                .whenActive(() -> { intakeFeeder.runConveyorIn(); } )
                .whenInactive(() -> { intakeFeeder.conveyorStop(); } );

    new Trigger(() -> (operatorController.getRightY() < -0.2))
                .whenActive(() -> { intakeFeeder.runConveyorOut(); } )
                .whenInactive(() -> { intakeFeeder.conveyorStop(); } );  
                
                new Trigger(() -> (operatorController.getLeftY() > 0.2))
                .whenActive(new InstantCommand(() ->  intakeFeeder.sushiIn() )).whenInactive(() -> { intakeFeeder.sushiStop(); } ); 
                new Trigger(() -> (operatorController.getLeftY() < -0.2))
                .whenActive(new InstantCommand(() ->  intakeFeeder.sushiOut() )).whenInactive(() -> { intakeFeeder.sushiStop(); } ); 
 
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
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
