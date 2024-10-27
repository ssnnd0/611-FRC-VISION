public class DriveSubsystem extends SubsystemBase {
    // ... existing code ...

    private Pose2d visionPose = new Pose2d();

    public void updateVisionPose(Pose2d pose) {
        visionPose = pose;
    }

    @Override
    public void periodic() {
        // ... existing code ...

        if (visionPose != null) {
            m_odometry.resetPosition(m_gyro.getRotation2d(), 
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
                },
                visionPose);
        }
    }

    // ... existing code ...
}
