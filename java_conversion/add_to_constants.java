public final class Constants {
    // ... existing constants ...

    public static final class VisionConstants {
        public static final int kCameraResolutionWidth = 640;
        public static final int kCameraResolutionHeight = 480;
        public static final int kCameraFPS = 30;
        public static final double kCameraFocalLength = 554.3; // Focal length in pixels
        public static final double kTagSize = 0.162; // Tag size in meters
        public static final Map<Integer, Pose2d> kTagPoses = new HashMap<>(); // Tag poses on the field
    }
}
