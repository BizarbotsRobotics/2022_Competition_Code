package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapToAngle extends CommandBase {

    private double angle_= 0d;
    private double angle = 0d;
    private DrivetrainSubsystem dt_;
    private PIDController thetaController_;
    private XboxController currentInput_;
    private boolean fenderAngle_ = false;

    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.00012;

    public SnapToAngle(double angle, DrivetrainSubsystem requirements) {
        thetaController_ = new PIDController(kP, kI, kD);
        angle_ = angle;
        addRequirements(requirements);
        dt_ = requirements;
    }

    @Override
    public void initialize() {
         angle_ = this.angle;
        thetaController_.setTolerance(4);
        if (angle_ > 180) angle_ -= 360;

        double gyroAngle = dt_.getGyroscopeRotationSNAP().getDegrees() % 360;
        gyroAngle +=  (gyroAngle > 180) ? -360 : 360;

        // Math to find shortest path
        double angleDiff = angle_ - gyroAngle;
        if (Math.abs(angleDiff) > 180) {
            if (angle_ > 0 && gyroAngle < 0) {
                angle_ -= 360;
            } 
            angle_ += 360;
        }
        SmartDashboard.putNumber("Snap Goal", angle_);        
        SmartDashboard.putNumber("Snap Starting", gyroAngle);


    }

    @Override
    public void execute() {
        
        double gyroAngle = dt_.getGyroscopeRotationSNAP().getDegrees() % 360;
        if (gyroAngle > 180) gyroAngle = 360 - gyroAngle;
        double angleDiff = angle_ - gyroAngle;
        SmartDashboard.putNumber("turnError", angleDiff);

        dt_.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,0,thetaController_.calculate(gyroAngle, -angle_)*.2, 
            dt_.getGyroscopeRotation() 
        ));
    }
    @Override
    public boolean isFinished() {
        double gyroAngle = dt_.getGyroscopeRotationSNAP().getDegrees() % 360;
        double angleDiff = angle_ - gyroAngle;
        return Math.abs(angleDiff) < 2;
    }
}