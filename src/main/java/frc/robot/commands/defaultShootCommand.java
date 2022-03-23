// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class defaultShootCommand extends SequentialCommandGroup {
  /** Creates a new defaultShootCommand. */
  private IntakeFeeder intakeFeeder;

  public defaultShootCommand(IntakeFeeder intakeFeeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.intakeFeeder = intakeFeeder;
  addCommands(
    new InstantCommand(() -> this.intakeFeeder.runConveyorIn()),
    new InstantCommand(() -> this.intakeFeeder.intakeCargo()),
    new InstantCommand(() -> this.intakeFeeder.sushiIn()),
    new WaitCommand(.5),
    //new InstantCommand(() -> this.conveyor.moveDistance(DEFAULT_CONVEYOR_DISTANCE)),
    new InstantCommand(() -> this.intakeFeeder.stopIntake()),
    new InstantCommand(() -> this.intakeFeeder.conveyorStop()),
    new InstantCommand(() -> this.intakeFeeder.sushiStop())
  );
  }
}
