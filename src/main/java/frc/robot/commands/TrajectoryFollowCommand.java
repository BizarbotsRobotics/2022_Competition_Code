package frc.robot.commands;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollowCommand extends CommandBase {

    private String m_pathName;
    private PathPlannerTrajectory m_trajectory = null;
    private DrivetrainSubsystem drivetrainSubsystem;

    /**
     * Executes a trajectory that makes it remain still
     */
    public TrajectoryFollowCommand() {
        m_pathName = "Stay Still";
    }

    public TrajectoryFollowCommand(DrivetrainSubsystem drivetrainSubsystem, String pathName) {
        m_pathName = pathName;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public TrajectoryFollowCommand(Trajectory traj) {
        m_trajectory = (PathPlannerTrajectory) traj;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");


        if (m_trajectory == null) {
            try {
                m_trajectory = PathPlanner.loadPath(m_pathName, 2.6, 2.6); //2.9, 3
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(.45, 0, .00025,
                new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Math.pow(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new PPSwerveControllerCommand(m_trajectory,
                drivetrainSubsystem::getCurrentPose,
                drivetrainSubsystem.getKinematics(),
                new PIDController(1.34, 0, 0),
                new PIDController(1.34, 0, 0),
                thetaController,
                drivetrainSubsystem::actuateModulesAuto,
                drivetrainSubsystem)
                        .andThen(() -> drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))
                        .schedule();
        
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
