// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.GOAL_TYPE;
import frc.robot.subsystems.ShooterSubsystem.POSITION;
import static frc.robot.Constants.*;

public class ShooterHighFarCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private VisionSubsystem vision;
  private ConveyorSubsystem conveyor;
  private FeederSubsystem feeder;
  private IntakeSubsystem intake;
  /** Creates a new ShooterLowCloseCommand. */
  public ShooterHighFarCommand(ShooterSubsystem shooter, VisionSubsystem vision, ConveyorSubsystem conveyor, FeederSubsystem feeder, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.vision = vision;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.intake = intake;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( this.vision.isAligned()) {
      int setPoint = this.shooter.calculateShooterSpeed(GOAL_TYPE.HIGH, POSITION.FAR);
      this.shooter.setShooterSpeed(setPoint);
      if(this.shooter.checkSpeed()) {
        //init shot
        new defaultShootCommand(conveyor, feeder, intake).andThen(() -> this.cancel()).schedule();
      }
    }
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
