package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionTrackingService {
    private final VisionConfig visionConfig;

    public VisionTrackingService(VisionConfig visionConfig) {
        this.visionConfig = visionConfig;
    }

    public Pose2d estimateRobotPose(Pose2d currentPose) {
        if (!visionConfig.hasTargets()) {
            return currentPose;
        }

        PhotonTrackedTarget bestTarget = visionConfig.getBestTarget();
        double distance = visionConfig.calculateDistanceToTarget(bestTarget);
        
        // Calculate pose transformation based on target
        Transform2d transform = new Transform2d(
            new Translation2d(distance, 0),
            new Rotation2d(bestTarget.getYaw())
        );

        return currentPose.transformBy(transform);
    }

    public boolean isAlignedWithTarget(double toleranceDegrees) {
        if (!visionConfig.hasTargets()) {
            return false;
        }

        PhotonTrackedTarget target = visionConfig.getBestTarget();
        return Math.abs(target.getYaw()) <= toleranceDegrees;
    }
}
