package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Swerve.LimelightConstants;
import frc.robot.Constants.FieldConstants;

public final class Limelight extends SubsystemBase {

    private final NetworkTable m_limelightTable;
    
    public Limelight() {
        // network table where all data from limelight is automatically sent and updated
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return (m_limelightTable.getEntry("tv").getDouble(0.0) == 1);
    }

    // returns angle in x axis that target is at
    public double getTX() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    // returns angle in y axis that target is at
    public double getTY() {
        return m_limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getDistance() {
        // default array used if we can't find camTran - can test for equality between default and camTran to see if it actually finds it
        double[] defaultArray = {0.0};
        // Probably will not work - Needs testing with an actual limelight
        double[] camTran = m_limelightTable.getEntry("camtran").getDoubleArray(defaultArray);
        double unconvertedDistance = camTran[2];

        // if defaultArray and camTram reference are the same we can catch it here
        return Math.abs(unconvertedDistance * LimelightConstants.distanceConversion);
    }

    public Pose2d getPose() {
        double[] poseArray = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        Pose2d pose = new Pose2d(new Translation2d(poseArray[0]+FieldConstants.length/2, poseArray[1]+FieldConstants.width/2), Rotation2d.fromDegrees(poseArray[5]));
        return pose;
    }

    public double getLatency() {
        return (m_limelightTable.getEntry("tl").getDouble(0.0)+m_limelightTable.getEntry("cl").getDouble(0.0))/1000.0;
    }

    public void switchPipeline(int pipelineID) {
        m_limelightTable.getEntry("pipeline").setNumber(pipelineID); 
        NetworkTableInstance.getDefault().flush();
    }

    public int getPipeline() {
        return m_limelightTable.getEntry("pipeline").getNumber(0).intValue();
    }
}
