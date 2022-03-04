// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Limelight;
import frc.robot.util.PidConstants;
import frc.robot.util.PidController;
import frc.robot.util.Vector2;

public class rotateToVisionTargetCommand extends CommandBase {
  private static final PidConstants PID_CONSTANTS = new PidConstants(0.5, 2.0, 0.025);
  private static final double ROTATION_STATIC_CONSTANT = 0.3;
  private static  final double MAXIMUM_AVERAGE_VELOCITY = 2.0;

  private final DrivetrainSubsystem drivetrain;
  private final VisionSubsystem visionSubsystem;

  private final DoubleSupplier xAxis;
  private final DoubleSupplier yAxis;

  private PidController controller = new PidController(PID_CONSTANTS);
  private double lastTime = 0.0;
  /** Creates a new rotateToVisionTargetCommand. */

  public rotateToVisionTargetCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem,
      DoubleSupplier xAxis, DoubleSupplier yAxis) {
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    addRequirements(drivetrain);
    addRequirements(visionSubsystem);

    controller.setInputRange(0.0, 2.0 * Math.PI);
    controller.setContinuous(true);
    controller.setIntegralRange(Math.toRadians(10.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTime = Timer.getFPGATimestamp();
    visionSubsystem.setCamMode(Limelight.CamMode.VISION);
    visionSubsystem.setSnapshotEnabled(true);
    controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    double dt = time - lastTime;
    lastTime = time;

    Vector2 translationalVelocity = new Vector2(xAxis.getAsDouble(), yAxis.getAsDouble());

    double rotationalVelocity = 0.0;
    if (visionSubsystem.hasTarget()) {
        double currentAngle = drivetrain.getPose().getRotation().getRadians();
        double targetAngle = visionSubsystem.getHorizontalOffset();
        controller.setSetpoint(targetAngle);
        rotationalVelocity = controller.calculate(currentAngle, dt);

        if (drivetrain.getAverageAbsoluteValueVelocity() < MAXIMUM_AVERAGE_VELOCITY) {
            rotationalVelocity += Math.copySign(
                    ROTATION_STATIC_CONSTANT / RobotController.getBatteryVoltage(),
                    rotationalVelocity
            );
        }
    }

    ChassisSpeeds cs = new ChassisSpeeds(translationalVelocity.x, translationalVelocity.y, rotationalVelocity);
    drivetrain.drive(cs);

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
