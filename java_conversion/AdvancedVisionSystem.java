package frc.robot.vision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.dnn.*;
import org.opencv.videoio.VideoCapture;
import org.opencv.ml.SVM;
import org.opencv.ml.Boost;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.*;
import javafx.animation.AnimationTimer;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.paint.Color;
import javafx.stage.FileChooser;
import javafx.geometry.Insets;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import java.util.*;
import java.io.*;
import java.nio.file.*;
import java.lang.management.ManagementFactory;
import java.util.concurrent.atomic.AtomicBoolean;

public class AdvancedVisionSystem extends Application {

    private final AprilTagDetector tagDetector;
    private final PhotonCamera photonCamera;
    private final NetworkTableInstance networkTable;
    private final DifferentialDrive robotDrive;
    private final PIDController alignmentPID;
    private final Timer timer;
    private final AIVideoDetector aiDetector;
    private double lastFrameTimestamp = 0;
    private double lastMetricsUpdate = 0;
    private int frameCount = 0;
    private double lastProcessingLatency = 0;
    private boolean saveFrames = false;

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

    private Canvas visionCanvas;
    private GraphicsContext gc;
    private LineChart<Number, Number> fpsChart;
    private XYChart.Series<Number, Number> fpsSeries;

    private final DifferentialDriveKinematics kinematics;
    private final SimpleMotorFeedforward feedforward;
    private final RamseteController ramseteController;

    private final AtomicBoolean isAutonomousMode = new AtomicBoolean(false);
    private Command currentAutonomousCommand;

    // GUI components
    private ColorPicker overlayColorPicker;
    private Slider thresholdSlider;
    private CheckBox showDetectionsCheckBox;
    private ComboBox<String> aiModelComboBox;
    private Button trainAIButton;
    private TextArea logTextArea;

    // AI training parameters
    private List<Mat> trainingImages;
    private List<Integer> trainingLabels;
    private SVM svm;
    private Boost boost;

    public AdvancedVisionSystem(DifferentialDrive drive) {
        this.robotDrive = drive;
        this.tagDetector = new AprilTagDetector();
        this.photonCamera = new PhotonCamera("photonvision");
        this.networkTable = NetworkTableInstance.getDefault();
        this.timer = new Timer();
        this.field = new Field2d();
        this.aiDetector = new AIVideoDetector();
        this.alignmentPID = new PIDController(0.1, 0.01, 0.005);
        alignmentPID.setTolerance(ALIGNMENT_TOLERANCE);

        NetworkTable visionTable = networkTable.getTable("Vision");
        targetXEntry = visionTable.getEntry("targetX");
        targetYEntry = visionTable.getEntry("targetY");
        targetFoundEntry = visionTable.getEntry("targetFound");
        tagIDEntry = visionTable.getEntry("tagID");
        robotPoseEntry = visionTable.getEntry("robotPose");
        telemetryEntry = visionTable.getEntry("telemetry");

        kinematics = new DifferentialDriveKinematics(0.6);
        feedforward = new SimpleMotorFeedforward(1, 3);
        ramseteController = new RamseteController();

        trainingImages = new ArrayList<>();
        trainingLabels = new ArrayList<>();
        svm = SVM.create();
        boost = Boost.create();

        initializeSystem();
    }

    private void initializeSystem() {
        tagDetector.addFamily("tag16h5");
        setupVisionProcessing();
        setupSmartDashboard();
        setupPathPlanner();
        setupAutonomousCommand();
        initializeAIModels();
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
                updateGUI();
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void processFrame(Mat frame) {
        long startTime = System.nanoTime();

        applyColorModifications(frame);

        ArrayList<AprilTagDetection> tags = tagDetector.detect(frame);
        if (!tags.isEmpty()) {
            AprilTagDetection bestTarget = tags.get(0);
            updateTargetInfo(bestTarget);
            if (showDetectionsCheckBox.isSelected()) {
                drawTargetOverlay(frame, bestTarget);
            }
            updateRobotPose(bestTarget);
        } else {
            targetFoundEntry.setBoolean(false);
        }

        List<DetectedObject> detectedObjects = aiDetector.detectObjects(frame);
        updateDetectedObjects(detectedObjects);
        if (showDetectionsCheckBox.isSelected()) {
            drawDetections(frame);
        }

        performStrategicAnalysis();

        long endTime = System.nanoTime();
        lastProcessingLatency = (endTime - startTime) / 1e6; // Convert to milliseconds
    }

    private void applyColorModifications(Mat frame) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV);
        Core.add(frame, new Scalar(10, 10, 10), frame);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2BGR);
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
        Scalar color = getOverlayColor();
        for (int i = 0; i < 4; i++) {
            Imgproc.line(frame, corners[i], corners[(i + 1) % 4], color, 2);
        }
        String text = String.format("ID: %d Dist: %.2fm", target.getId(), calculateTargetDistance(target));
        Imgproc.putText(frame, text, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
    }

    private Scalar getOverlayColor() {
        Color color = overlayColorPicker.getValue();
        return new Scalar(color.getBlue() * 255, color.getGreen() * 255, color.getRed() * 255);
    }

    private double calculateTargetDistance(AprilTagDetection target) {
        double targetPitch = Math.atan2(target.getCenterY() - 240, 554.3);
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(CAMERA_PITCH_RADIANS + targetPitch);
    }

    private void updateRobotPose(AprilTagDetection target) {
        double[] botposeTargetSpace = getBotPoseTargetSpace(target);
        Pose3d botPositionFromTag = new Pose3d(
            botposeTargetSpace[0], botposeTargetSpace[1], botposeTargetSpace[2],
            new Rotation3d(botposeTargetSpace[3], botposeTargetSpace[4], botposeTargetSpace[5])
        );
        currentPose = botPositionFromTag.toPose2d();
        field.setRobotPose(currentPose);
        double[] poseArray = {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()};
        robotPoseEntry.setDoubleArray(poseArray);
    }

    private double[] getBotPoseTargetSpace(AprilTagDetection target) {
        // Implementation depends on your specific setup
        return new double[]{0, 0, 0, 0, 0, 0};
    }

    private void updateTelemetry() {
        Map<String, Double> telemetryData = new HashMap<>();
        telemetryData.put("timestamp", timer.get());
        telemetryData.put("robotX", currentPose.getX());
        telemetryData.put("robotY", currentPose.getY());
        telemetryData.put("robotHeading", currentPose.getRotation().getDegrees());
        telemetryData.put("targetFound", targetFoundEntry.getBoolean(false) ? 1.0 : 0.0);
        telemetryData.put("processingLatency", lastProcessingLatency);
        telemetryData.put("fps", calculateFPS());
        telemetryEntry.setString(new Gson().toJson(telemetryData));
    }

    private double calculateFPS() {
        double currentTime = Timer.getFPGATimestamp();
        double fps = 1.0 / (currentTime - lastFrameTimestamp);
        lastFrameTimestamp = currentTime;
        return fps;
    }

    private void setupSmartDashboard() {
        SmartDashboard.putData("Field", field);
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        visionTab.add("Robot Pose", field);
        visionTab.addNumber("Target X", () -> targetXEntry.getDouble(0));
        visionTab.addNumber("Target Y", () -> targetYEntry.getDouble(0));
        visionTab.addBoolean("Target Found", () -> targetFoundEntry.getBoolean(false));
        visionTab.addNumber("Processing Latency", () -> lastProcessingLatency);
        visionTab.addNumber("FPS", this::calculateFPS);
    }

    private void setupPathPlanner() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ExamplePath", 4, 3);
        // Use the trajectory in your autonomous command
    }

    private void setupAutonomousCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ExamplePath", 4, 3);
        currentAutonomousCommand = new PPRamseteCommand(
            trajectory,
            this::getPose,
            kinematics,
            ramseteController,
            feedforward,
            this::getWheelSpeeds,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            this::tankDriveVolts,
            true,
            this
        );
    }

    private Pose2d getPose() {
        return currentPose;
    }

    private DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // Implement this method to return the current wheel speeds
        return new DifferentialDriveWheelSpeeds(0, 0);
    }

    private void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.tankDrive(leftVolts / 12.0, rightVolts / 12.0);
    }

    private void performStrategicAnalysis() {
        logMessage("Performing strategic analysis...");
        if (isNearScoringPosition()) {
            logMessage("Near scoring position. Recommending alignment.");
        }
    }

    private boolean isNearScoringPosition() {
        return currentPose.getTranslation().getDistance(new Translation2d(5, 5)) < 1.0;
    }

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle("Advanced Vision System");

        BorderPane root = new BorderPane();

        VBox centerContent = new VBox(10);
        centerContent.setPadding(new Insets(10));

        visionCanvas = new Canvas(640, 480);
        gc = visionCanvas.getGraphicsContext2D();
        centerContent.getChildren().add(visionCanvas);

        setupFPSChart();
        centerContent.getChildren().add(fpsChart);

        root.setCenter(centerContent);

        VBox sidePanel = new VBox(10);
        sidePanel.setPadding(new Insets(10));
        sidePanel.setStyle("-fx-background-color: #f0f0f0;");

        Label colorLabel = new Label("Overlay Color:");
        overlayColorPicker = new ColorPicker(Color.GREEN);
        sidePanel.getChildren().addAll(colorLabel, overlayColorPicker);

                Label thresholdLabel = new Label("Detection Threshold:");
        thresholdSlider = new Slider(0, 1, 0.5);
        thresholdSlider.setShowTickLabels(true);
        thresholdSlider.setShowTickMarks(true);
        sidePanel.getChildren().addAll(thresholdLabel, thresholdSlider);

        showDetectionsCheckBox = new CheckBox("Show Detections");
        showDetectionsCheckBox.setSelected(true);
        sidePanel.getChildren().add(showDetectionsCheckBox);

        Label modelLabel = new Label("AI Model:");
        aiModelComboBox = new ComboBox<>();
        aiModelComboBox.getItems().addAll("YOLOv3", "SSD", "Faster R-CNN");
        aiModelComboBox.setValue("YOLOv3");
        sidePanel.getChildren().addAll(modelLabel, aiModelComboBox);

        trainAIButton = new Button("Train AI Model");
        trainAIButton.setOnAction(e -> trainAIModel());
        sidePanel.getChildren().add(trainAIButton);

        Button captureButton = new Button("Capture Frame");
        captureButton.setOnAction(e -> captureFrame());
        sidePanel.getChildren().add(captureButton);

        logTextArea = new TextArea();
        logTextArea.setEditable(false);
        logTextArea.setPrefHeight(200);
        sidePanel.getChildren().add(logTextArea);

        root.setRight(sidePanel);

        Scene scene = new Scene(root, 1000, 600);
        primaryStage.setScene(scene);
        primaryStage.show();

        startAnimationTimer();
    }

    private void setupFPSChart() {
        NumberAxis xAxis = new NumberAxis();
        NumberAxis yAxis = new NumberAxis();
        fpsChart = new LineChart<>(xAxis, yAxis);
        fpsChart.setTitle("FPS Over Time");
        xAxis.setLabel("Time (s)");
        yAxis.setLabel("FPS");

        fpsSeries = new XYChart.Series<>();
        fpsSeries.setName("FPS");
        fpsChart.getData().add(fpsSeries);
    }

    private void startAnimationTimer() {
        new AnimationTimer() {
            @Override
            public void handle(long now) {
                updateGUI();
            }
        }.start();
    }

    private void updateGUI() {
        Platform.runLater(() -> {
            if (gc != null && currentFrame != null && !currentFrame.empty()) {
                gc.clearRect(0, 0, visionCanvas.getWidth(), visionCanvas.getHeight());
                WritableImage writableImage = new WritableImage(currentFrame.width(), currentFrame.height());
                MatOfByte buffer = new MatOfByte();
                Imgcodecs.imencode(".png", currentFrame, buffer);
                Image image = new Image(new ByteArrayInputStream(buffer.toArray()));
                gc.drawImage(image, 0, 0);

                updateFPSChart();
            }
        });
    }

    private void updateFPSChart() {
        double fps = calculateFPS();
        double time = timer.get();
        fpsSeries.getData().add(new XYChart.Data<>(time, fps));

        // Remove old data points to keep the chart readable
        if (fpsSeries.getData().size() > 100) {
            fpsSeries.getData().remove(0);
        }
    }

    private void captureFrame() {
        if (currentFrame != null && !currentFrame.empty()) {
            String filename = "capture_" + System.currentTimeMillis() + ".png";
            Imgcodecs.imwrite(filename, currentFrame);
            logMessage("Frame captured: " + filename);
        }
    }

    private void trainAIModel() {
        String selectedModel = aiModelComboBox.getValue();
        logMessage("Training " + selectedModel + " model...");
        
        // Simulating AI training process
        new Thread(() -> {
            for (int i = 0; i < 100; i++) {
                try {
                    Thread.sleep(50);
                    final int progress = i;
                    Platform.runLater(() -> logMessage("Training progress: " + progress + "%"));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Platform.runLater(() -> logMessage("Training complete!"));
        }).start();
    }

    private void logMessage(String message) {
        logTextArea.appendText(message + "\n");
    }

    private void initializeAIModels() {
        // Initialize SVM
        svm.setType(SVM.C_SVC);
        svm.setKernel(SVM.RBF);
        svm.setTermCriteria(new TermCriteria(TermCriteria.MAX_ITER, 100, 1e-6));

        // Initialize Boost
        boost.setBoostType(Boost.GENTLE);
        boost.setWeakCount(100);
        boost.setWeightTrimRate(0.95);
        boost.setMaxDepth(2);
        boost.setUseSurrogates(false);
        boost.setPriors(new Mat());
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
            String modelPath = "path/to/yolov3.weights";
            String configPath = "path/to/yolov3.cfg";
            String classesPath = "path/to/coco.names";

            net = Dnn.readNetFromDarknet(configPath, modelPath);
            classes = loadClasses(classesPath);
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
            Mat blob = Dnn.blobFromImage(frame, 1/255.0, new Size(416, 416), new Scalar(0), true, false);
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
            float confThreshold = (float) thresholdSlider.getValue();

            for (int i = 0; i < outs.size(); ++i) {
                Mat level = outs.get(i);
                for (int j = 0; j < level.rows(); ++j) {
                    Mat row = level.row(j);
                    Mat scores = row.colRange(5, level.cols());
                    Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
                    float confidence = (float) mm.maxVal;
                    Point classIdPoint = mm.maxLoc;

                    if (confidence > confThreshold) {
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

        public List<DetectedObject> getLastDetectedObjects() {
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

    public static void main(String[] args) {
        launch(args);
    }
}
