package frc.robot.vision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import javax.servlet.http.*;
import org.eclipse.jetty.server.*;
import org.eclipse.jetty.servlet.*;
import com.google.gson.Gson;
import java.util.*;
import java.io.*;

public class EnhancedVisionSystem {
    private final AprilTagDetector tagDetector;
    private final PhotonCamera photonCamera;
    private final NetworkTableInstance networkTable;
    private final DifferentialDrive robotDrive;
    private final PIDController alignmentPID;
    private final Timer timer;
    private final AIVideoDetector aiDetector;
    private final WebServer webServer;
    private double lastFrameTimestamp = 0;
    private double lastMetricsUpdate = 0;
    private int frameCount = 0;
    private double lastProcessingLatency = 0;
    private boolean saveFrames = false;
    private WebSocketServer webSocketServer;

    private final NetworkTableEntry targetXEntry, targetYEntry, targetFoundEntry, tagIDEntry, robotPoseEntry, telemetryEntry;

    private List<GameObject> detectedObjects;
    private List<Obstacle> detectedObstacles;
    private Mat currentFrame;

    private static final double CAMERA_HEIGHT_METERS = 0.5;
    private static final double TARGET_HEIGHT_METERS = 1.0;
    private static final double CAMERA_PITCH_RADIANS = 0.0;
    private static final double ALIGNMENT_TOLERANCE = 0.02;
    private static final double MAX_DRIVE_SPEED = 0.5;

    private Pose2d currentPose;
    private final Field2d field;

    public EnhancedVisionSystem(DifferentialDrive drive) {
        this.robotDrive = drive;
        this.tagDetector = new AprilTagDetector();
        this.photonCamera = new PhotonCamera("photonvision");
        this.networkTable = NetworkTableInstance.getDefault();
        this.timer = new Timer();
        this.field = new Field2d();
        this.aiDetector = new AIVideoDetector();
        this.webServer = new WebServer(5800);

        this.alignmentPID = new PIDController(0.1, 0.01, 0.005);
        alignmentPID.setTolerance(ALIGNMENT_TOLERANCE);

        NetworkTable visionTable = networkTable.getTable("Vision");
        targetXEntry = visionTable.getEntry("targetX");
        targetYEntry = visionTable.getEntry("targetY");
        targetFoundEntry = visionTable.getEntry("targetFound");
        tagIDEntry = visionTable.getEntry("tagID");
        robotPoseEntry = visionTable.getEntry("robotPose");
        telemetryEntry = visionTable.getEntry("telemetry");

        initializeSystem();
    }

    private void initializeSystem() {
        tagDetector.addFamily("tag16h5");
        setupVisionProcessing();
        webServer.start();
    }

    private void setupVisionProcessing() {
        Thread visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            camera.setFPS(30);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);
            currentFrame = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(currentFrame) == 0) continue;
                processFrame(currentFrame);
                outputStream.putFrame(currentFrame);
                updateTelemetry();
                updateWebInterface();
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void processFrame(Mat frame) {
        ArrayList<AprilTagDetection> tags = tagDetector.detect(frame);
        if (!tags.isEmpty()) {
            AprilTagDetection bestTarget = tags.get(0);
            updateTargetInfo(bestTarget);
            drawTargetOverlay(frame, bestTarget);
            updateRobotPose(bestTarget);
        } else {
            targetFoundEntry.setBoolean(false);
        }

        List<DetectedObject> detectedObjects = aiDetector.detectObjects(frame);
        updateDetectedObjects(detectedObjects);
        drawDetections(frame);
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

        String text = String.format("ID: %d Dist: %.2fm", target.getId(), calculateTargetDistance(target));
        Imgproc.putText(frame, text, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
    }

    private double calculateTargetDistance(AprilTagDetection target) {
        double targetPitch = Math.atan2(target.getCenterY() - 240, 554.3);
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(CAMERA_PITCH_RADIANS + targetPitch);
    }

    private void updateRobotPose(AprilTagDetection target) {
        double[] botposeTargetSpace = getBotPoseTargetSpace();
        Pose3d botPositionFromTag = new Pose3d(botposeTargetSpace[0], botposeTargetSpace[1], botposeTargetSpace[2],
                new Rotation3d(botposeTargetSpace[3], botposeTargetSpace[4], botposeTargetSpace[5]));
        currentPose = botPositionFromTag.toPose2d();
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

    private void updateWebInterface() {
        Map<String, Object> visionData = new HashMap<>();
        
        // Camera and frame data
        visionData.put("timestamp", Timer.getFPGATimestamp());
        visionData.put("fps", photonCamera.getPipelineResult().getLatencyMillis());
        visionData.put("frameCount", photonCamera.getPipelineResult().getFrameNumber());
        
        // Robot pose data
        Map<String, Double> poseData = new HashMap<>();
        poseData.put("x", currentPose.getX());
        poseData.put("y", currentPose.getY());
        poseData.put("rotation", currentPose.getRotation().getDegrees());
        visionData.put("robotPose", poseData);
    
        // Target data
        List<Map<String, Object>> targetsData = new ArrayList<>();
        for (DetectedObject obj : detectedObjects) {
            Map<String, Object> targetInfo = new HashMap<>();
            targetInfo.put("id", obj.id);
            targetInfo.put("type", obj.label);
            targetInfo.put("confidence", obj.confidence);
            targetInfo.put("x", obj.x);
            targetInfo.put("y", obj.y);
            targetInfo.put("area", obj.boundingBox.area());
            targetInfo.put("yaw", Math.atan2(obj.x - (currentFrame.width() / 2.0), currentFrame.width() / 2.0));
            targetInfo.put("pitch", Math.atan2(obj.y - (currentFrame.height() / 2.0), currentFrame.height() / 2.0));
            targetsData.add(targetInfo);
        }
        visionData.put("detectedTargets", targetsData);
    
        // Pipeline data
        Map<String, Object> pipelineData = new HashMap<>();
        pipelineData.put("activeModel", customModelPredictor != null ? "custom" : "default");
        pipelineData.put("3dModelEnabled", model3DPredictor != null);
        pipelineData.put("aprilTagEnabled", tagDetector != null);
        pipelineData.put("exposureTime", photonCamera.getPipelineResult().getLatencyMillis());
        pipelineData.put("threshold", 0.5); // Detection threshold
        visionData.put("pipeline", pipelineData);
    
        // System diagnostics
        Map<String, Object> diagnostics = new HashMap<>();
        diagnostics.put("processingLatency", Timer.getFPGATimestamp() - lastFrameTimestamp);
        diagnostics.put("cpuLoad", getCPULoad());
        diagnostics.put("ramUsage", getRAMUsage());
        diagnostics.put("temperature", getSystemTemp());
        visionData.put("diagnostics", diagnostics);
    
        // Update NetworkTables
        NetworkTable visionTable = networkTable.getTable("VisionSystem");
        visionTable.getEntry("data").setString(new Gson().toJson(visionData));
    
        // Update web socket clients if connected
        if (webSocketServer != null && webSocketServer.hasConnections()) {
            try {
                webSocketServer.broadcast(new Gson().toJson(visionData));
            } catch (Exception e) {
                System.err.println("Failed to broadcast vision data: " + e.getMessage());
            }
        }
    
        // Save frame if enabled
        if (saveFrames) {
            String timestamp = String.format("%d", System.currentTimeMillis());
            Imgcodecs.imwrite("vision_frames/" + timestamp + ".jpg", currentFrame);
        }
    
        // Update performance metrics
        lastFrameTimestamp = Timer.getFPGATimestamp();
        frameCount++;
        if (frameCount % 30 == 0) { // Update metrics every 30 frames
            updatePerformanceMetrics();
        }
    }
    
    private double getCPULoad() {
        try {
            return ManagementFactory.getOperatingSystemMXBean().getSystemLoadAverage();
        } catch (Exception e) {
            return -1;
        }
    }
    
    private long getRAMUsage() {
        return Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
    }
    
    private double getSystemTemp() {
        try {
            String temp = new String(Files.readAllBytes(Paths.get("/sys/class/thermal/thermal_zone0/temp")));
            return Double.parseDouble(temp) / 1000.0;
        } catch (Exception e) {
            return -1;
        }
    }
    
    private void updatePerformanceMetrics() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsed = currentTime - lastMetricsUpdate;
        double fps = frameCount / elapsed;
        
        NetworkTable metricsTable = networkTable.getTable("VisionMetrics");
        metricsTable.getEntry("fps").setDouble(fps);
        metricsTable.getEntry("latency").setDouble(lastProcessingLatency * 1000);
        metricsTable.getEntry("uptime").setDouble(Timer.getFPGATimestamp());
        
        frameCount = 0;
        lastMetricsUpdate = currentTime;
    }

    private void updateDetectedObjects(List<DetectedObject> detectedObjects) {
        this.detectedObjects = new ArrayList<>();
        this.detectedObstacles = new ArrayList<>();

        for (DetectedObject obj : detectedObjects) {
            if (obj.label.equals("game_object")) {
                GameObject gameObj = new GameObject();
                gameObj.id = obj.id;
                gameObj.x = obj.x;
                gameObj.y = obj.y;
                this.detectedObjects.add(gameObj);
            } else if (obj.label.equals("obstacle")) {
                Obstacle obstacle = new Obstacle();
                obstacle.id = obj.id;
                obstacle.x = obj.x;
                obstacle.y = obj.y;
                this.detectedObstacles.add(obstacle);
            }
        }
    }

    private void drawDetections(Mat frame) {
        for (DetectedObject obj : aiDetector.getLastDetectedObjects()) {
            Scalar color;
            if (obj.label.equals("game_object")) {
                color = new Scalar(255, 0, 0); // Blue for game objects
            } else if (obj.label.equals("obstacle")) {
                color = new Scalar(0, 0, 255); // Red for obstacles
            } else {
                color = new Scalar(0, 255, 0); // Green for other objects
            }

            Imgproc.rectangle(frame, obj.boundingBox.tl(), obj.boundingBox.br(), color, 2);
            Imgproc.putText(frame, obj.label, obj.boundingBox.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }

    private class AIVideoDetector {
        private Net net;
        private List<String> classes;
        private List<DetectedObject> lastDetectedObjects;

        public AIVideoDetector() {
            // Load pre-trained model and classes
            net = Dnn.readNetFromDarknet("path/to/yolov3.cfg", "path/to/yolov3.weights");
            classes = loadClasses("path/to/coco.names");
            lastDetectedObjects = new ArrayList<>();
        }

        private List<String> loadClasses(String filename) {
            List<String> classes = new ArrayList<>();
            try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
                String line;
                while ((line = br.readLine()) != null) {
                    classes.add(line.trim());
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
            return classes;
        }

        public List<DetectedObject> detectObjects(Mat frame) {
            Mat blob = Dnn.blobFromImage(frame, 1 / 255.0, new Size(416, 416), new Scalar(0), true, false);
            net.setInput(blob);

            List<Mat> result = new ArrayList<>();
            List<String> outBlobNames = net.getUnconnectedOutLayersNames();
            net.forward(result, outBlobNames);

            List<DetectedObject> detectedObjects = postprocess(frame, result);
            lastDetectedObjects = detectedObjects;
            return detectedObjects;
        }

        private List<DetectedObject> postprocess(Mat frame, List<Mat> outs) {
            List<DetectedObject> detectedObjects = new ArrayList<>();
            for (int i = 0; i < outs.size(); ++i) {
                Mat level = outs.get(i);
                for (int j = 0; j < level.rows(); ++j) {
                    Mat row = level.row(j);
                    Mat scores = row.colRange(5, level.cols());
                    Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
                    float confidence = (float) mm.maxVal;
                    Point classIdPoint = mm.maxLoc;
                    if (confidence > 0.5) {
                        int centerX = (int) (row.get(0, 0)[0] * frame.cols());
                        int centerY = (int) (row.get(0, 1)[0] * frame.rows());
                        int width = (int) (row.get(0, 2)[0] * frame.cols());
                        int height = (int) (row.get(0, 3)[0] * frame.rows());
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;

                        DetectedObject obj = new DetectedObject();
                        obj.id = detectedObjects.size();
                        obj.label = classes.get((int) classIdPoint.x);
                        obj.confidence = confidence;
                        obj.boundingBox = new Rect(left, top, width, height);
                        obj.x = centerX;
                        obj.y = centerY;
                        detectedObjects.add(obj);
                    }
                }
            }
            return detectedObjects;
        }

        public List<DetectedObject> getLastDetected Objects() {
            return lastDetectedObjects;
        }
    }

    private class DetectedObject {
        public int id;
        public String label;
        public float confidence;
        public Rect boundingBox;
        public int x;
        public int y;
    }

    private class GameObject {
        public int id;
        public int x;
        public int y;
    }

    private class Obstacle {
        public int id;
        public int x;
        public int y;
    }

    private class WebServer {
        private Server server;

        public WebServer(int port) {
            server = new Server(port);
            ServletContextHandler context = new ServletContextHandler(ServletContextHandler.SESSIONS);
            context.setContextPath("/");
            server.setHandler(context);

            context.addServlet(new ServletHolder(new WebInterfaceServlet()), "/interface");
        }

        public void start() {
            try {
                server.start();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private class WebInterfaceServlet extends HttpServlet {
        @Override
        protected void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
            resp.setContentType("text/html");
            resp.setStatus(HttpServletResponse.SC_OK);

            PrintWriter writer = resp.getWriter();
            writer.println("<html><body>");
            writer.println("<h1>Vision System Interface</h1>");
            writer.println("<p>Detected Objects:</p>");
            writer.println("<ul>");
            for (GameObject obj : detectedObjects) {
                writer.println("<li>ID: " + obj.id + ", X: " + obj.x + ", Y: " + obj.y + "</li>");
            }
            writer.println("</ul>");
            writer.println("<p>Detected Obstacles:</p>");
            writer.println("<ul>");
            for (Obstacle obstacle : detectedObstacles) {
                writer.println("<li>ID: " + obstacle.id + ", X: " + obstacle.x + ", Y: " + obstacle.y + "</li>");
            }
            writer.println("</ul>");
            writer.println("</body></html>");
        }
    }
}
