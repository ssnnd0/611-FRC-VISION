package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineType;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class VisionConfig {
    private final PhotonCamera camera;
    private NetworkTableInstance networkTableInstance;

    // Camera configuration constants
    public static final String CAMERA_NAME = "MainCamera";
    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double TARGET_HEIGHT_METERS = 2.0;
    public static final double CAMERA_PITCH_RADIANS = Math.toRadians(30);

    public VisionConfig() {
        this.camera = new PhotonCamera(CAMERA_NAME);
        this.networkTableInstance = NetworkTableInstance.getDefault();
        configurePipeline();
    }

    private void configurePipeline() {
        camera.setPipelineType(PhotonPipelineType.APRILTAG);
        camera.setDriverMode(false);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getLatestResult().getTargets();
    }

    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return hasTargets() ? getTargets().get(0) : null;
    }

    public double calculateDistanceToTarget(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            target.getPitch()
        );
    }
}
