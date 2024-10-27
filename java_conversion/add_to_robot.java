public class Robot extends TimedRobot {
    private VisionSystem visionSystem;

    @Override
    public void robotInit() {
        visionSystem = new VisionSystem(new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1)));
    }

    @Override
    public void autonomousInit() {
        Command autoCommand = visionSystem.createAutoAlignCommand(1).andThen(visionSystem.createAutoDriveCommand(2.0));
        autoCommand.schedule();
    }
}
