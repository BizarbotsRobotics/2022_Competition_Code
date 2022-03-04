// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;

public class VisionSubsystem extends SubsystemBase {
        private static final double TARGET_HEIGHT = 104;
        private static final double LIMELIGHT_HEIGHT = 19;

        private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(30);

        private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

        private static final Limelight LIMELIGHT = new Limelight("shooter");

        private double distanceToHub = 0;

        private double horizontalOffset = 0;

        private boolean hasTarget;

        public VisionSubsystem() {

        }

        @Override
        public void periodic() {
                this.hasTarget = LIMELIGHT.hasTarget();
                if (this.hasTarget){
                        double verticalOffset = LIMELIGHT.getVerticalOffset();
                        this.horizontalOffset = LIMELIGHT.getHorizontalOffset();
                        double theta = LIMELIGHT_MOUNTING_ANGLE + verticalOffset;
                        this.distanceToHub = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(theta);
                } 
        }

        public boolean hasTarget() {
                return hasTarget;
        }

        public double getDistanceToHub(){
                return this.distanceToHub;
        }

        public double getHorizontalOffset() {
                return this.horizontalOffset;
        }

        public void setLedMode(Limelight.LedMode mode) {
                LIMELIGHT.setLedMode(mode);
        }
        
        public void setSnapshotEnabled(boolean isEnabled) {
                LIMELIGHT.setSnapshotsEnabled(isEnabled);
        }

        public void setCamMode(Limelight.CamMode mode) {
                LIMELIGHT.setCamMode(mode);
        }
}
