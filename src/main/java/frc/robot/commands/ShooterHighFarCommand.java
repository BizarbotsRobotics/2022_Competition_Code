// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.GOAL_TYPE;
import frc.robot.subsystems.ShooterSubsystem.POSITION;
import static frc.robot.Constants.*;

public class ShooterHighFarCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private VisionSubsystem vision;
  private IntakeFeeder intakeFeeder;
  private boolean shot;
  /** Creates a new ShooterLowCloseCommand. */
  public ShooterHighFarCommand(ShooterSubsystem shooter, VisionSubsystem vision,IntakeFeeder intakeFeeder) {
    this.shooter = shooter;
    this.vision = vision;
    this.intakeFeeder = intakeFeeder;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shot = false;
    this.shooter.setShooterLock(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.closeAngle();
    this.shooter.setShooterLock(true);
    if( this.vision.isAligned()) {
      int setPoint = this.shooter.calculateShooterSpeed(GOAL_TYPE.HIGH, POSITION.FAR);
      this.shooter.setShooterSpeed(setPoint);
      if(this.shooter.checkSpeed()) {
        //init shot
        new defaultShootCommand(intakeFeeder).andThen(new InstantCommand(() -> shot = true)).schedule();
        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //this.shooter.setShooterSpeed(SHOOTER_DEFAULT_SPEED);
    this.shooter.setShooterLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shot;
  }
}
