package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.HashMap;

public class ApriltagLimelightVision {
  private HashMap<Double, PoseEstimate> poseEstimates = new HashMap<>();
  private ArrayList<Double> avgAreas = new ArrayList<>();
  private PoseEstimate bestEstimate = null;
  private RobotContainer m_robotContainer;

  public ApriltagLimelightVision(RobotContainer robotContainer) {
    this.m_robotContainer = robotContainer;
  }

  public void updateVisionEstimates(Rotation2d orientation) {
    poseEstimates.clear();
    avgAreas.clear();

    LimelightHelpers.SetRobotOrientation(
        Constants.VisionConstants.limelightPoseEstimatorName,
        orientation.getDegrees(),
        0,
        0,
        0,
        0,
        0);

    PoseEstimate pos;

    pos =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.VisionConstants.limelightPoseEstimatorName);

    if (pos != null && pos.tagCount > 0) {
      poseEstimates.put(pos.avgTagArea, pos);
    }

    avgAreas.addAll(poseEstimates.keySet());
    avgAreas.sort(null);

    if (!avgAreas.isEmpty()) {
      bestEstimate = poseEstimates.get(avgAreas.get(avgAreas.size() - 1));
    }

    if (bestEstimate != null && bestEstimate.tagCount > 0) {
      Constants.VisionConstants.bestLimelightPose = bestEstimate;
    } else {

      bestEstimate = null;
    }
  }

  public void applyVisionToEstimator() {
    if (bestEstimate != null) {
      m_robotContainer.drive.addVisionMeasurement(
          bestEstimate.pose,
          Utils.fpgaToCurrentTime(bestEstimate.timestampSeconds),
          Constants.VisionConstants.visionStdDevs);
    }
  }

  public void resetPoseFromVision(Rotation2d rot) {
    if (bestEstimate != null && bestEstimate.tagCount > 0) {
      m_robotContainer.drive.setPose(new Pose2d(bestEstimate.pose.getTranslation(), rot));
    }
  }

  public Rotation2d estimateRotationFromVision() {
    updateVisionEstimates(m_robotContainer.drive.getRotation());
    return (bestEstimate != null) ? bestEstimate.pose.getRotation() : new Rotation2d();
  }

  public void periodic() {
    if (bestEstimate != null) {
      SmartDashboard.putString("Vision Pose", bestEstimate.pose.toString());
    }
  }
}
