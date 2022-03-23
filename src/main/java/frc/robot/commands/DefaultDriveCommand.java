package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    // private final ChassisSpeeds cs;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        // this.cs = new ChassisSpeeds(
        //     m_translationXSupplier.getAsDouble(),
        //     m_translationYSupplier.getAsDouble(),
        //     m_rotationSupplier.getAsDouble()
        // ) ;                



        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // cs.vxMetersPerSecond = m_translationXSupplier.getAsDouble();
        // cs.vyMetersPerSecond = m_translationYSupplier.getAsDouble();
        // cs.omegaRadiansPerSecond = m_rotationSupplier.getAsDouble();
        // m_drivetrainSubsystem.drive(cs);
        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
