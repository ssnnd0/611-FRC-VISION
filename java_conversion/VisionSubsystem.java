package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;

public class VisionSubsystem extends SubsystemBase {
    private final AprilTagDetector detector;
    private final NetworkTable visionTable;
    private final DriveSubsystem driveSubsystem;
    
    // Network Table entries
    private final NetworkTableEntry targetXEntry;
    private final NetworkTableEntry targetYEntry;
    private final NetworkTableEntry targetFoundEntry;
    private final NetworkTableEntry tagIDEntry;
    private final NetworkTableEntry distanceEntry;

    private AprilTagDetection currentTarget = null;
    private Thread visionThread;
    private volatile boolean running = true;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        
        // Initialize AprilTag detector
        detector = new AprilTagDetector();
        detector.addFamily("tag16h5");

        // Initialize Network Tables
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        targetXEntry = visionTable.getEntry("targetX");
        targetYEntry = visionTable.getEntry("targetY");
        targetFoundEntry = visionTable.getEntry("targetFound");
        tagIDEntry = visionTable.getEntry("tagID");
        distanceEntry = visionTable.getEntry("distance");

        startVisionThread();
    }

    private void startVisionThread() {
        visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(VisionConstants.kCameraResolutionWidth, 
                               VisionConstants.kCameraResolutionHeight);
            camera.setFPS(VisionConstants.kCameraFPS);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Detected", 
                VisionConstants.kCameraResolutionWidth, 
                VisionConstants.kCameraResolutionHeight);

            Mat mat = new Mat();

            while (running) {
                if (cvSink.grabFrame(mat) == 0) continue;

                ArrayList<AprilTagDetection> detections = detector.detect(mat);
                processDetections(detections, mat);
                outputStream.putFrame(mat);
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void processDetections(ArrayList<AprilTagDetection> detections, Mat image) {
        if (detections.isEmpty()) {
            targetFoundEntry.setBoolean(false);
            currentTarget = null;
            return;
        }

        currentTarget = detections.get(0); // Get closest/best target
        targetFoundEntry.setBoolean(true);
        
        updateNetworkTables(currentTarget);
        drawDetection(currentTarget, image);
    }

    private void drawDetection(AprilTagDetection detection, Mat image) {
        Point[] corners = new Point[4];
        for (int i = 0; i < 4; i++) {
            corners[i] = new Point(detection.getCornerX(i), detection.getCornerY(i));
        }

        Scalar color = new Scalar(0, 255, 0);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(image, corners[i], corners[(i + 1) % 4], color, 2);
        }

        Point center = new Point(detection.getCenterX(), detection.getCenterY());
        String text = String.format("ID: %d D: %.2fm", detection.getId(), 
            calculateDistance(detection));
        Imgproc.putText(image, text, 
            new Point(center.x - 20, center.y - 20),
            Imgproc.FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }

    private void updateNetworkTables(AprilTagDetection detection) {
        targetXEntry.setDouble(detection.getCenterX());
        targetYEntry.setDouble(detection.getCenterY());
        tagIDEntry.setDouble(detection.getId());
        
        double distance = calculateDistance(detection);
        distanceEntry.setDouble(distance);
    }

    private double calculateDistance(AprilTagDetection detection) {
        // Calculate distance using tag size and focal length
        double focalLength = VisionConstants.kCameraFocalLength;
        double tagSize = VisionConstants.kTagSize;
        double pixelWidth = detection.getCornerX(1) - detection.getCornerX(0);
        
        return (tagSize * focalLength) / pixelWidth;
    }

    @Override
    public void periodic() {
        if (currentTarget != null) {
            // Update pose estimation
            Pose2d estimatedPose = calculateRobotPose(currentTarget);
            driveSubsystem.updateVisionPose(estimatedPose);
            
            // Update SmartDashboard
            SmartDashboard.putNumber("Tag Distance", calculateDistance(currentTarget));
            SmartDashboard.putNumber("Tag ID", currentTarget.getId());
            SmartDashboard.putBoolean("Target Found", true);
        } else {
            SmartDashboard.putBoolean("Target Found", false);
        }
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection) {
        // Calculate robot pose based on AprilTag detection
        double distance = calculateDistance(detection);
        double angleToTarget = Math.atan2(detection.getCenterX() - (VisionConstants.kCameraResolutionWidth/2.0),
                                        VisionConstants.kCameraFocalLength);
        
        // Get the field position of the detected AprilTag
        Pose2d tagPose = VisionConstants.kTagPoses.get(detection.getId());
        if (tagPose == null) return driveSubsystem.getPose();

        // Calculate robot position relative to tag
        double robotX = tagPose.getX() - (distance * Math.cos(angleToTarget));
        double robotY = tagPose.getY() - (distance * Math.sin(angleToTarget));
        Rotation2d robotRotation = tagPose.getRotation().minus(new Rotation2d(angleToTarget));

        return new Pose2d(robotX, robotY, robotRotation);
    }

    public boolean hasTarget() {
        return currentTarget != null;
    }

    public double getTargetDistance() {
        return currentTarget != null ? calculateDistance(currentTarget) : 0.0;
    }

    public int getTargetId() {
        return currentTarget != null ? currentTarget.getId() : -1;
    }

    public void close() {
        running = false;
        if (visionThread != null) {
            try {
                visionThread.join(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        detector.close();
    }
}
