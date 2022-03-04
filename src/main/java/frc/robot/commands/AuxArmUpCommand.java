// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AuxArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class AuxArmUpCommand extends CommandBase {
  /** Creates a new AuxArmUpCommand. */
  private final AuxArmSubsystem aux;
  public AuxArmUpCommand(AuxArmSubsystem aux) {
    this.aux = aux;
    addRequirements(aux);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.aux.AuxArmUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.aux.stopAuxArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
