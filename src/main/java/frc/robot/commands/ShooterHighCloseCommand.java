// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.attribute.PosixFilePermission;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.GOAL_TYPE;
import frc.robot.subsystems.ShooterSubsystem.POSITION;

public class ShooterHighCloseCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  /** Creates a new ShooterLowCloseCommand. */
  public ShooterHighCloseCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setShooterSpeed(this.shooter.calculateShooterSpeed(GOAL_TYPE.HIGH, POSITION.CLOSE));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
