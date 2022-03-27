// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue3Ball extends SequentialCommandGroup {
  /** Creates a new DriveStraight2Ball. */
  private DrivetrainSubsystem drivetrainSubsystem;
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeFeeder intakeFeeder;
  public Blue3Ball(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, IntakeFeeder intakeFeeder, ShooterSubsystem shooterSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeFeeder = intakeFeeder;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new SnapToAngle(90, drivetrainSubsystem)
      new PathResetOdometryCommand(this.drivetrainSubsystem, "Blue 3 Ball 1"),
      new InstantCommand(() -> intakeFeeder.runIntake(1), intakeFeeder),
      new TrajectoryFollowCommand(this.drivetrainSubsystem, "Blue 3 Ball 1").withTimeout(2.5),
      new InstantCommand(() -> intakeFeeder.stopIntake(), intakeFeeder).withTimeout(.5),
      new autoAlign(visionSubsystem).withTimeout(3).deadlineWith(new InstantCommand(() -> drivetrainSubsystem.driveAuto(new ChassisSpeeds(0,0,visionSubsystem.getLimelightOffset() * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 1.1)))),
      new ShooterHighFarCommand(shooterSubsystem, visionSubsystem, intakeFeeder).withTimeout(2),
      new InstantCommand(() -> shooterSubsystem.setShooterSpeed(SHOOTER_DEFAULT_SPEED)).withTimeout(.5),
      //new SnapToAngle(-90, drivetrainSubsystem),
      //new PathResetOdometryCommand(drivetrainSubsystem, "Blue 3 Ball 2"),
      new InstantCommand(() -> intakeFeeder.runIntake(1), intakeFeeder),
      new TrajectoryFollowCommand(this.drivetrainSubsystem, "Blue 3 Ball 2").withTimeout(5),
      new InstantCommand(() -> intakeFeeder.stopIntake(), intakeFeeder),
      new autoAlign(visionSubsystem).withTimeout(3).deadlineWith(new InstantCommand(() -> drivetrainSubsystem.driveAuto(new ChassisSpeeds(0,0,visionSubsystem.getLimelightOffset() * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 1.1)))),
      new ShooterHighFarCommand(shooterSubsystem, visionSubsystem, intakeFeeder).withTimeout(4),
      new InstantCommand(() -> shooterSubsystem.setShooterSpeed(SHOOTER_DEFAULT_SPEED))

    );
  }
}
