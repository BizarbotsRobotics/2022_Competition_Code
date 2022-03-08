// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.attribute.PosixFilePermission;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.GOAL_TYPE;
import frc.robot.subsystems.ShooterSubsystem.POSITION;

public class ShooterHighCloseCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private VisionSubsystem vision;
  private ConveyorSubsystem conveyor;
  private FeederSubsystem feeder;
  /** Creates a new ShooterLowCloseCommand. */
  public ShooterHighCloseCommand(ShooterSubsystem shooter, VisionSubsystem vision, ConveyorSubsystem conveyor, FeederSubsystem feeder) {
    this.shooter = shooter;
    this.vision = vision;
    this.conveyor = conveyor;
    this.feeder = feeder;
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
      int setPoint = this.shooter.calculateShooterSpeed(GOAL_TYPE.HIGH, POSITION.CLOSE);
      this.shooter.setShooterSpeed(setPoint);
      if(this.shooter.getShooterSpeed() < setPoint +30 && this.shooter.getShooterSpeed() > setPoint - 30 ) {
        //init shot
        new SequentialCommandGroup(
          new InstantCommand(() -> (this.conveyor.runConveyorIn())),
          new InstantCommand(() -> (this.feeder.sushiIn())),
          new WaitCommand(.5),
          new InstantCommand(() -> (this.conveyor.stop())),
          new InstantCommand(() -> (this.feeder.sushiStop())),
        );
        this.cancel();
      }
    } else {
      this.cancel();
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
