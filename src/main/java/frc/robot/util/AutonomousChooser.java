package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class AutonomousChooser {

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser() {

        autonomousModeChooser.setDefaultOption("1 Ball Auto", AutonomousMode.ONE_BALL);
        autonomousModeChooser.addOption("2 Ball Compatible", AutonomousMode.TWO_BALL);
        autonomousModeChooser.addOption("DrIvE FoRwArD", AutonomousMode.DRIVE_FORWARD);
        autonomousModeChooser.addOption("Spin Move", AutonomousMode.SPIN_MOVE);

    }



    private enum AutonomousMode {
        ONE_BALL,
        TWO_BALL,
        DRIVE_FORWARD,
        SPIN_MOVE
    }
}