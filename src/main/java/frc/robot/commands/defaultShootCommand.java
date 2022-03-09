// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class defaultShootCommand extends SequentialCommandGroup {
  /** Creates a new defaultShootCommand. */
  private ConveyorSubsystem conveyor;
  private FeederSubsystem feeder;
  private IntakeSubsystem intake;

  public defaultShootCommand(ConveyorSubsystem conveyor, FeederSubsystem feeder, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.intake = intake;
  addCommands(
    new InstantCommand(() -> this.conveyor.runConveyorIn()),
    new InstantCommand(() -> this.intake.intakeCargo()),
    new InstantCommand(() -> this.feeder.sushiIn()),
    new WaitCommand(.5),
    //new InstantCommand(() -> this.conveyor.moveDistance(DEFAULT_CONVEYOR_DISTANCE)),
    new InstantCommand(() -> this.intake.stop()),
    new InstantCommand(() -> this.conveyor.stop()),
    new InstantCommand(() -> this.feeder.sushiStop())
  );
  }
}
