// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {

    private final NetworkTable table;

    private final NetworkTableEntry tv;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry ts;
    private final NetworkTableEntry tl;
    private final NetworkTableEntry tcornx;
    private final NetworkTableEntry tcorny;

    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry camMode;
    private final NetworkTableEntry pipeline;
    private final NetworkTableEntry stream;
    private final NetworkTableEntry snapshot;

    public Limelight() {
        this(NetworkTableInstance.getDefault().getTable("limelight"));
    }

    /**
     * Creates an instance of the Limelight class given the name of the Limelight. For example,
     * if the Limelight is called "limelight-cargo", pass in "cargo".
     * @param name The name of the Limelight.
     */
    public Limelight(String name) {
        this(NetworkTableInstance.getDefault().getTable("limelight-" + name));
    }

    /**
     * Creates an instance of the Limelight class given its NetworkTable.
     * @param table The NetworkTable used to create the Limelight.
     */
    public Limelight(NetworkTable table) {
        this.table = table;

        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");

        tcornx = table.getEntry("tcornx");
        tcorny = table.getEntry("tcorny");

        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        pipeline = table.getEntry("pipeline");
        stream = table.getEntry("stream");
        snapshot = table.getEntry("snapshot");
    }


    public boolean hasTarget(){
        int target = this.tv.getNumber(0).intValue();
        if(target == 0){
            return false;
        }
        return  true;
    }

    public double getHorizontalOffset(){
        return this.tx.getDouble(0);
    }

    public double getVerticalOffset(){
        return this.ty.getDouble(0);
    }

    public double getTargetArea(){
        return this.ta.getDouble(0);
    }

    public double getSkew(){
        return this.ts.getDouble(0);
    }

    public double getLatency(){
        return this.tl.getDouble(0);
    }
    /**
     * Sets the operating mode of the Limelight.
     * @param mode The operating mode the Limelight should be set to.
     */
    public void setCamMode(CamMode mode) {
        switch (mode) {
            case VISION:
                camMode.setNumber(0);
                break;
            case DRIVER:
                camMode.setNumber(1);
        }
    }

    public void setSnapshotsEnabled(boolean enabled) {
        if (enabled) {
            snapshot.setNumber(1);
        } else {
            snapshot.setNumber(0);
        }
    }

    /**
     * Sets the mode of the LED's of the Limelight.
     * @param mode The mode the LED's should be set to.
     */
    public void setLedMode(LedMode mode) {
        switch (mode) {
            case DEFAULT:
                ledMode.setNumber(0);
                break;
            case OFF:
                ledMode.setNumber(1);
                break;
            case BLINK:
                ledMode.setNumber(2);
                break;
            case ON:
                ledMode.setNumber(3);
                break;
        }
    }

    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
    }

    public enum CamMode {
        /**
         * Brings the exposure down and runs the pipeline
         */
        VISION,
        /**
         * Disables the pipeline, and brings the exposure up
         */
        DRIVER
    }
        /**
     * Represents the different LED modes of the Limelight
     */
    public enum LedMode {
        /**
         * Sets the LED's to whatever is specified in the pipeline
         */
        DEFAULT,
        /**
         * Turns the LED's on
         */
        ON,
        /**
         * Turns the LED's off
         */
        OFF,
        /**
         * Makes the LED's blink
         */
        BLINK
    }

    /**
     * Represents the different streaming modes of the camera
     */
    public enum StreamMode {
        /**
         * Side-by-side streams if a webcam is attached to Limelight
         */
        STANDARD,
        /**
         * The secondary camera stream is placed in the lower-right corner of the primary camera stream
         */
        PIP_MAIN,
        /**
         * The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        PIP_SECONDARY
    }
    
    public double[][] getCorners() {
        double[] x = tcornx.getDoubleArray(new double[]{0.0, 0.0});
        double[] y = tcorny.getDoubleArray(new double[]{0.0, 0.0});
        double[][] corners = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            corners[i][0] = x[i];
            corners[i][1] = y[i];
        }
        return corners;
    }
}
