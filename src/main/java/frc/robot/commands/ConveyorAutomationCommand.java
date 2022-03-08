// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

public class ConveyorAutomationCommand extends CommandBase {
  /** Creates a new sushiCommand. */
  private final ConveyorSubsystem conveyor;
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private IntakeSubsystem intake;
  public ConveyorAutomationCommand(ConveyorSubsystem conveyor, ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake) {
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.feeder = feeder;
    this.intake = intake;
    addRequirements(conveyor, shooter,feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this.conveyor.runConveyor(this.speed.getAsDouble());

    
    // If there is a ball loaded and is the incorrect color and slot two is free we reject using the shooter.
    // FIX ME Maybe add position pid control or add check
    this.conveyor.checkBallOneStatus();
    if(this.conveyor.checkBallLoadedFront() && !this.conveyor.isBallOneCorrectColor() && !this.conveyor.checkBallLoadedBack()) {
      new SequentialCommandGroup(
        new InstantCommand(() -> this.shooter.setShooterSpeed(2500)),
        new InstantCommand(() -> this.intake.intakeCargo()),
        new InstantCommand(() -> this.conveyor.runConveyorIn()),
        new WaitCommand(1),
        new InstantCommand(() -> this.feeder.sushiIn()),
        new InstantCommand(() -> this.intake.stop()),
        new WaitCommand(1),
        new InstantCommand(() -> this.feeder.sushiStop()),
        new InstantCommand(() -> this.shooter.setShooterSpeed(SHOOTER_DEFAULT_SPEED)),
        new InstantCommand(() -> this.conveyor.stop())
      ).schedule(true);
    }
    // If there is a ball loaded and is the correct color and slot two is free we move back.
    else if(this.conveyor.checkBallLoadedFront() && !this.conveyor.checkBallLoadedBack() && this.conveyor.isBallOneCorrectColor()) {
      new SequentialCommandGroup(
        new InstantCommand(() -> this.conveyor.runConveyorIn()),
        new WaitCommand(.5),
        new InstantCommand(() -> this.conveyor.stop())
      ).schedule(true);
      this.conveyor.moveFrontBallToBackPosition();
    } 
    // // If there is a ball in the back and the new ball is the wrong color, we outtake the ball
    else if(this.conveyor.checkBallLoadedFront() && this.conveyor.checkBallLoadedBack() && !this.conveyor.isBallOneCorrectColor()) {
      new SequentialCommandGroup(
        new InstantCommand(() -> this.conveyor.runConveyorOut()),
        new InstantCommand(() -> this.intake.outtakeCargo()),
        new WaitCommand(.5),
        new InstantCommand(() -> this.conveyor.stop()),
        new InstantCommand(() -> this.intake.stop()),
        new InstantCommand(() -> this.conveyor.runConveyorIn()),
        new WaitCommand(.5),
        new InstantCommand(() -> this.conveyor.stop())
      ).schedule(true);
    }
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
