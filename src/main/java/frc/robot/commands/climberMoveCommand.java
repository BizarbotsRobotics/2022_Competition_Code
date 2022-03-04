// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class climberMoveCommand extends CommandBase {
  /** Creates a new climberMoveCommand. */
  private final ClimberSubsystem climber;
  private DoubleSupplier upSpeed, downSpeed;
  /** Creates a new lowerClimber. */
  public climberMoveCommand(ClimberSubsystem climber, DoubleSupplier upSpeed, DoubleSupplier downSpeed) {
    this.climber = climber;
    this.upSpeed = upSpeed;
    this.downSpeed = downSpeed;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.climber.climberMove(upSpeed.getAsDouble(), downSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
