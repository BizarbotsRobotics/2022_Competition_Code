// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue2Ball extends SequentialCommandGroup {
  /** Creates a new DriveStraight1Ball. */
  private DrivetrainSubsystem drivetrainSubsystem;
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeFeeder intakeFeeder;
  public Blue2Ball(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, IntakeFeeder intakeFeeder, ShooterSubsystem shooterSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeFeeder = intakeFeeder;
    addCommands(
      new PathResetOdometryCommand(drivetrainSubsystem, "Blue 3 Ball 1"),
      new TrajectoryFollowCommand(drivetrainSubsystem, "Blue 3 Ball 1").raceWith(new InstantCommand(() -> this.intakeFeeder.intakeCargo())).andThen(new InstantCommand(() -> this.intakeFeeder.stopIntake())),
      new autoAlign(visionSubsystem).raceWith(new DefaultDriveCommand(drivetrainSubsystem, (()->0),(()->0),(() ->(visionSubsystem.getLimelightOffset() * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)))),
      new ShooterHighFarCommand(shooterSubsystem, visionSubsystem, intakeFeeder).withTimeout(1)
    );
  }
}
