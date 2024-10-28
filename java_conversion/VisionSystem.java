package frc.robot.vision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class VisionSystem {
    // Core components
    private final AprilTagDetector tagDetector;
    private final PhotonCamera camera;
    private final NetworkTableInstance networkTable;
    private final DifferentialDrive robotDrive;
    private final PIDController alignmentPID;
    private final Timer timer;

    // Network Table entries
    private final NetworkTableEntry targetXEntry;
    private final NetworkTableEntry targetYEntry;
    private final NetworkTableEntry targetFoundEntry;
    private final NetworkTableEntry tagIDEntry;
    private final NetworkTableEntry robotPoseEntry;
    private final NetworkTableEntry telemetryEntry;

    // Constants
    private static final double CAMERA_HEIGHT_METERS = 0.5;
    private static final double TARGET_HEIGHT_METERS = 1.0;
    private static final double CAMERA_PITCH_RADIANS = 0.0;
    private static final double ALIGNMENT_TOLERANCE = 0.02;
    private static final double MAX_DRIVE_SPEED = 0.5;

    // Robot pose tracking
    private Pose2d currentPose;
    private final Field2d field;

    public VisionSystem(DifferentialDrive drive) {
        // Initialize core components
        this.robotDrive = drive;
        this.tagDetector = new AprilTagDetector();
        this.camera = new PhotonCamera("photonvision");
        this.networkTable = NetworkTableInstance.getDefault();
        this.timer = new Timer();
        this.field = new Field2d();

        // Initialize PID controller for alignment
        this.alignmentPID = new PIDController(0.1, 0.01, 0.005);
        alignmentPID.setTolerance(ALIGNMENT_TOLERANCE);

        // Setup Network Tables
        NetworkTable visionTable = networkTable.getTable("Vision");
        targetXEntry = visionTable.getEntry("targetX");
        targetYEntry = visionTable.getEntry("targetY");
        targetFoundEntry = visionTable.getEntry("targetFound");
        tagIDEntry = visionTable.getEntry("tagID");
        robotPoseEntry = visionTable.getEntry("robotPose");
        telemetryEntry = visionTable.getEntry("telemetry");

        // Initialize AprilTag detector
        tagDetector.addFamily("tag16h5");
        
        // Start vision processing
        startVisionProcessing();
    }

    private void startVisionProcessing() {
        Thread visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            camera.setFPS(30);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);
            Mat frame = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(frame) == 0) continue;

                processFrame(frame);
                outputStream.putFrame(frame);
                updateTelemetry();
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void processFrame(Mat frame) {
        ArrayList<AprilTagDetection> detections = tagDetector.detect(frame);
        
        if (!detections.isEmpty()) {
            AprilTagDetection bestTarget = detections.get(0);
            updateTargetInfo(bestTarget);
            drawTargetOverlay(frame, bestTarget);
            updateRobotPose(bestTarget);
        } else {
            targetFoundEntry.setBoolean(false);
        }
    }

    private void updateTargetInfo(AprilTagDetection target) {
        targetFoundEntry.setBoolean(true);
        targetXEntry.setDouble(target.getCenterX());
        targetYEntry.setDouble(target.getCenterY());
        tagIDEntry.setDouble(target.getId());
    }

    private void drawTargetOverlay(Mat frame, AprilTagDetection target) {
        Point[] corners = new Point[4];
        for (int i = 0; i < 4; i++) {
            corners[i] = new Point(target.getCornerX(i), target.getCornerY(i));
        }

        Scalar color = new Scalar(0, 255, 0);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(frame, corners[i], corners[(i + 1) % 4], color, 2);
        }

        String text = String.format("ID: %d Dist: %.2fm", target.getId(), 
            calculateTargetDistance(target));
        Imgproc.putText(frame, text, new Point(10, 30), 
            Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
    }

    private double calculateTargetDistance(AprilTagDetection target) {
        double targetPitch = Math.atan2(target.getCenterY() - 240, 554.3);
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / 
               Math.tan(CAMERA_PITCH_RADIANS + targetPitch);
    }

    private void updateRobotPose(AprilTagDetection target) {
        // Calculate robot pose based on AprilTag position
        Transform2d cameraToPose = new Transform2d(
            new Translation2d(calculateTargetDistance(target), new Rotation2d(target.getYaw())),
            new Rotation2d()
        );
        
        currentPose = new Pose2d().plus(cameraToPose);
        field.setRobotPose(currentPose);
        
        double[] poseArray = {
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getDegrees()
        };
        robotPoseEntry.setDoubleArray(poseArray);
    }

    private void updateTelemetry() {
        Map<String, Double> telemetryData = new HashMap<>();
        telemetryData.put("timestamp", timer.get());
        telemetryData.put("robotX", currentPose.getX());
        telemetryData.put("robotY", currentPose.getY());
        telemetryData.put("robotHeading", currentPose.getRotation().getDegrees());
        telemetryData.put("targetFound", targetFoundEntry.getBoolean(false) ? 1.0 : 0.0);
        
        telemetryEntry.setString(telemetryData.toString());
    }

    // Autonomous Navigation Methods
    public Command createAutoAlignCommand(int targetTagId) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Starting alignment to tag " + targetTagId)),
            new FunctionalCommand(
                // Init
                () -> alignmentPID.reset(),
                // Execute
                () -> {
                    if (targetFoundEntry.getBoolean(false) && 
                        tagIDEntry.getDouble(0) == targetTagId) {
                        double error = targetXEntry.getDouble(0) - 320; // Center X
                        double rotation = alignmentPID.calculate (error);
                        robotDrive.arcadeDrive(0, rotation);
                    } else {
                        robotDrive.arcadeDrive(0, 0);
                    }
                },
                // End
                () -> {
                    robotDrive.arcadeDrive(0, 0);
                    System.out.println("Alignment complete");
                },
                // Interrupt
                () -> {
                    robotDrive.arcadeDrive(0, 0);
                    System.out.println("Alignment interrupted");
                }
            )
        );
    }

    public Command createAutoDriveCommand(double distance) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Starting drive to distance " + distance)),
            new FunctionalCommand(
                // Init
                () -> {},
                // Execute
                () -> {
                    double error = distance - currentPose.getX();
                    double speed = Math.min(MAX_DRIVE_SPEED, Math.abs(error));
                    robotDrive.arcadeDrive(speed, 0);
                },
                // End
                () -> {
                    robotDrive.arcadeDrive(0, 0);
                    System.out.println("Drive complete");
                },
                // Interrupt
                () -> {
                    robotDrive.arcadeDrive(0, 0);
                    System.out.println("Drive interrupted");
                }
            )
        );
    }
}
