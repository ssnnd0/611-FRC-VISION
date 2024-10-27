public class RobotContainer {
    // ... existing code ...

    private final VisionSubsystem visionSubsystem;

    public RobotContainer() {
        // ... existing code ...

        visionSubsystem = new VisionSubsystem(robotDrive);
    }

    // ... existing code ...
}
