// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new DriveStraight. */
  private DrivetrainSubsystem drivetrainSubsystem;
  private VisionSubsystem vision;
  private ShooterSubsystem shooter;
  private IntakeFeeder intake;
  public DriveStraight(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem vision, ShooterSubsystem shooter, IntakeFeeder intake) {
    this.vision = vision;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooter = shooter;
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new DefaultDriveCommand(drivetrainSubsystem, (()->.75),(()->0),(()->0)).withTimeout(3),
      // new DefaultDriveCommand(drivetrainSubsystem, (()->.0),(()->0),(()->0)),
      // new autoAlign(vision).raceWith(new DefaultDriveCommand(drivetrainSubsystem, (()->0),(()->0),(() ->(vision.getLimelightOffset() * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)))),
      // new WaitCommand(3),
      // new ShooterHighFarCommand(shooter, vision, intake)
      // new InstantCommand(() ->drivetrainSubsystem.driveAuto(new ChassisSpeeds(.75,0,0))).withTimeout(3),
      // new InstantCommand(() ->drivetrainSubsystem.driveAuto(new ChassisSpeeds(0,0,(vision.getLimelightOffset()) * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND))).withTimeout(3).raceWith(new InstantCommand(() -> vision.setLimelightOffset())).withTimeout(2),
      new DefaultDriveCommand(drivetrainSubsystem, (()->.75),(()->0),(()->0)).withTimeout(3),
      new DefaultDriveCommand(drivetrainSubsystem, (()->.0),(()->0),(()->0)).withTimeout(.2),
      new autoAlign(vision).raceWith(new DefaultDriveCommand(drivetrainSubsystem, (()->0),(()->0),(() ->(vision.getLimelightOffset() * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)))).withTimeout(2),
      new ShooterHighFarCommand(shooter, vision, intake).withTimeout(5)
    );
  }
}
