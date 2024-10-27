public class AutoAlignAmpCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AutoAlignAmpCmd(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            double distance = visionSubsystem.getTargetDistance();
            double angleToTarget = Math.atan2(visionSubsystem.getTargetX() - (Constants.VisionConstants.kCameraResolutionWidth/2.0),
                                            Constants.VisionConstants.kCameraFocalLength);

            // Drive to the target
            driveSubsystem.drive(distance * Math.cos(angleToTarget), 
                                 distance * Math.sin(angleToTarget), 
                                 0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
