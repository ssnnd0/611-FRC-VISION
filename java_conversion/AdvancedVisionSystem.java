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
import org.opencv.objdetect.CascadeClassifier;

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
import javafx.scene.control.Alert.AlertType;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import java.util.*;
import java.io.*;
import java.nio.file.*;
import java.lang.management.ManagementFactory;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

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

    private CascadeClassifier faceCascade;
    private Mat lastProcessedFrame;
    private boolean isRecording = false;
    private VideoWriter videoWriter;
    private String recordingFilename;

    private TabPane tabPane;
    private Tab visionTab;
    private Tab analysisTab;
    private Tab settingsTab;

    private LineChart<Number, Number> objectCountChart;
    private XYChart.Series<Number, Number> gameObjectSeries;
    private XYChart.Series<Number, Number> obstacleSeries;

    private Slider brightnessSlider;
    private Slider contrastSlider;
    private Slider saturationSlider;

    private CheckBox enableFaceDetectionCheckBox;
    private CheckBox enableObjectTrackingCheckBox;

    private ComboBox<String> cameraSourceComboBox;
    private List<String> availableCameras;

    private Button startRecordingButton;
    private Button stopRecordingButton;

    private TextArea consoleOutput;

    private Canvas fieldMapCanvas;
    private GraphicsContext fieldMapGc;
    private Map<Integer, Point2D> gameObjectPositions;
    private Point2D robotPosition;

    private String appName = "Advanced Vision System";
    private boolean isDarkTheme = false;
    private Color robotColor = Color.BLUE;
    private Color gameObjectColor = Color.GREEN;
    private Color obstacleColor = Color.RED;

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

        faceCascade = new CascadeClassifier();
        faceCascade.load("path/to/haarcascade_frontalface_default.xml");

        lastProcessedFrame = new Mat();
        
        availableCameras = new ArrayList<>();
        
        gameObjectPositions = new HashMap<>();
        robotPosition = new Point2D(0, 0);

        initializeSystem();
    }

    private void initializeSystem() {
        tagDetector.addFamily("tag16h5");
        setupVisionProcessing();
        setupSmartDashboard();
        setupPathPlanner();
        setupAutonomousCommand();
        initializeGUI();
        setupAdditionalCharts();
        loadAvailableCameras();
        initializeFieldMap();
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

        if (enableFaceDetectionCheckBox.isSelected()) {
            detectAndDrawFaces(frame);
        }

        if (enableObjectTrackingCheckBox.isSelected()) {
            trackObjects(frame);
        }

        applyImageAdjustments(frame);

        if (isRecording && videoWriter.isOpened()) {
            videoWriter.write(frame);
        }

        frame.copyTo(lastProcessedFrame);

        performStrategicAnalysis();

        updateRobotPosition();
        updateGameObjectPositions();
        updateFieldMap();

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

    private void initializeGUI() {
        tabPane = new TabPane();
        visionTab = new Tab("Vision");
        analysisTab = new Tab("Analysis");
        settingsTab = new Tab("Settings");
        Tab fieldMapTab = new Tab("Field Map");

        tabPane.getTabs().addAll(visionTab, analysisTab, settingsTab, fieldMapTab);

        visionTab.setContent(createVisionTabContent());
        analysisTab.setContent(createAnalysisTabContent());
        settingsTab.setContent(createSettingsTabContent());
        fieldMapTab.setContent(createFieldMapTabContent());
    }

    private VBox createVisionTabContent() {
        VBox content = new VBox(10);
        content.setPadding(new Insets(10));

        visionCanvas = new Canvas(640, 480);
        gc = visionCanvas.getGraphicsContext2D();

        setupFPSChart();

        content.getChildren().addAll(visionCanvas, fpsChart);

        return content;
    }

    private VBox createAnalysisTabContent() {
        VBox content = new VBox(10);
        content.setPadding(new Insets(10));

        content.getChildren().addAll(objectCountChart);

        return content;
    }

    private VBox createSettingsTabContent() {
        VBox content = new VBox(10);
        content.setPadding(new Insets(10));

        Label brightnessLabel = new Label("Brightness:");
        brightnessSlider = new Slider(0, 100, 50);
        brightnessSlider.setShowTickLabels(true);
        brightnessSlider.setShowTickMarks(true);

        Label contrastLabel = new Label("Contrast:");
        contrastSlider = new Slider(0, 100, 50);
        contrastSlider.setShowTickLabels(true);
        contrastSlider.setShowTickMarks(true);

        Label saturationLabel = new Label("Saturation:");
        saturationSlider = new Slider(0, 100, 50);
        saturationSlider.setShowTickLabels(true);
        saturationSlider.setShowTickMarks(true);

        enableFaceDetectionCheckBox = new CheckBox("Enable Face Detection");
        enableObjectTrackingCheckBox = new CheckBox("Enable Object Tracking");

        Label cameraSourceLabel = new Label("Camera Source:");
        cameraSourceComboBox = new ComboBox<>();
        cameraSourceComboBox.setOnAction(e -> switchCamera(cameraSourceComboBox.getValue()));

        startRecordingButton = new Button("Start Recording");
        startRecordingButton.setOnAction(e -> startRecording());

        stopRecordingButton = new Button("Stop Recording");
        stopRecordingButton.setOnAction(e -> stopRecording());
        stopRecordingButton.setDisable(true);

        consoleOutput = new TextArea();
        consoleOutput.setEditable(false);
        consoleOutput.setPrefRowCount(10);

        CheckBox darkThemeCheckBox = new CheckBox("Dark Theme");
        darkThemeCheckBox.setSelected(isDarkTheme);
        darkThemeCheckBox.setOnAction(e -> toggleTheme(darkThemeCheckBox.isSelected()));

        Label robotColorLabel = new Label("Robot Color:");
        ColorPicker robotColorPicker = new ColorPicker(robotColor);
        robotColorPicker.setOnAction(e -> {
            robotColor = robotColorPicker.getValue();
            updateFieldMap();
        });

        Label gameObjectColorLabel = new Label("Game Object Color:");
        ColorPicker gameObjectColorPicker = new ColorPicker(gameObjectColor);
        gameObjectColorPicker.setOnAction(e -> {
            gameObjectColor = gameObjectColorPicker.getValue();
            updateFieldMap();
        });

        Label obstacleColorLabel = new Label("Obstacle Color:");
        ColorPicker obstacleColorPicker = new ColorPicker(obstacleColor);
        obstacleColorPicker.setOnAction(e -> {
            obstacleColor = obstacleColorPicker.getValue();
            updateFieldMap();
        });

        content.getChildren().addAll(
            brightnessLabel, brightnessSlider,
            contrastLabel, contrastSlider,
            saturationLabel, saturationSlider,
            enableFaceDetectionCheckBox, enableObjectTrackingCheckBox,
            cameraSourceLabel, cameraSourceComboBox,
            startRecordingButton, stopRecordingButton,
            new Label("Console Output:"),
            consoleOutput,
            darkThemeCheckBox,
            robotColorLabel, robotColorPicker,
            gameObjectColorLabel, gameObjectColorPicker,
            obstacleColorLabel, obstacleColorPicker
        );

        return content;
    }

    private VBox createFieldMapTabContent() {
        VBox content = new VBox(10);
        content.setPadding(new Insets(10));

        content.getChildren().add(fieldMapCanvas);

        return content;
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

    private void setupAdditionalCharts() {
        NumberAxis xAxis = new NumberAxis();
        NumberAxis yAxis = new NumberAxis();
        objectCountChart = new LineChart<>(xAxis, yAxis);
        objectCountChart.setTitle("Detected Objects Over Time");
        xAxis.setLabel("Time (s)");
        yAxis.setLabel("Count");

        gameObjectSeries = new XYChart.Series<>();
        gameObjectSeries.setName("Game Objects");
        obstacleSeries = new XYChart.Series<>();
        obstacleSeries.setName("Obstacles");

        objectCountChart.getData().addAll(gameObjectSeries, obstacleSeries);
    }

    private void loadAvailableCameras() {
        availableCameras.clear();
        availableCameras.add("Default Camera");

        // This is a simplified way to detect cameras. In practice, you'd use platform-specific methods.
        for (int i = 0; i < 10; i++) {
            VideoCapture cap = new VideoCapture(i);
            if (cap.isOpened()) {
                availableCameras.add("Camera " + i);
                cap.release();
            }
        }

        Platform.runLater(() -> {
            cameraSourceComboBox.getItems().clear();
            cameraSourceComboBox.getItems().addAll(availableCameras);
            cameraSourceComboBox.setValue("Default Camera");
        });
    }

    private void switchCamera(String cameraSource) {
        // Implementation to switch between cameras
        logMessage("Switching to camera: " + cameraSource);
        // You would need to modify your video capture setup here
    }

    private void startRecording() {
        if (!isRecording) {
            recordingFilename = "recording_" + System.currentTimeMillis() + ".avi";
            videoWriter = new VideoWriter(recordingFilename, VideoWriter.fourcc('M','J','P','G'), 10, new Size(640, 480));
            isRecording = true;
            startRecordingButton.setDisable(true);
            stopRecordingButton.setDisable(false);
            logMessage("Started recording: " + recordingFilename);
        }
    }

    private void stopRecording() {
        if (isRecording) {
            isRecording = false;
            videoWriter.release();
            startRecordingButton.setDisable(false);
            stopRecordingButton.setDisable(true);
            logMessage("Stopped recording: " + recordingFilename);
        }
    }

    private void detectAndDrawFaces(Mat frame) {
        MatOfRect faces = new MatOfRect();
        Mat grayFrame = new Mat();
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);
        faceCascade.detectMultiScale(grayFrame, faces);

        for (Rect rect : faces.toArray()) {
            Imgproc.rectangle(frame, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);
        }
    }

    private void trackObjects(Mat frame) {
        // Implement object tracking algorithm here
        // This is a placeholder for demonstration
        logMessage("Object tracking in progress...");
    }

    private void applyImageAdjustments(Mat frame) {
        double alpha = contrastSlider.getValue() / 50.0; // Contrast control (1.0-3.0)
        int beta = (int) (brightnessSlider.getValue() - 50); // Brightness control (0-100)

        frame.convertTo(frame, -1, alpha, beta);

        if (saturationSlider.getValue() != 50) {
            Mat hsvImage = new Mat();
            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);
            List<Mat> hsvChannels = new ArrayList<>();
            Core.split(hsvImage, hsvChannels);
            Core.multiply(hsvChannels.get(1), saturationSlider.getValue() / 50.0, hsvChannels.get(1));
            Core.merge(hsvChannels, hsvImage);
            Imgproc.cvtColor(hsvImage, frame, Imgproc.COLOR_HSV2BGR);
        }
    }

    private void updateObjectCountChart() {
        double time = timer.get();
        int gameObjectCount = detectedObjects.size();
        int obstacleCount = detectedObstacles.size();

        Platform.runLater(() -> {
            gameObjectSeries.getData().add(new XYChart.Data<>(time, gameObjectCount));
            obstacleSeries.getData().add(new XYChart.Data<>(time, obstacleCount));

            // Remove old data points to keep the chart readable
            if (gameObjectSeries.getData().size() > 100) {
                gameObjectSeries.getData().remove(0);
                obstacleSeries.getData().remove(0);
            }
        });
    }

    private void logMessage(String message) {
        Platform.runLater(() -> {
            consoleOutput.appendText(message + "\n");
            // Autoscroll to the bottom
            consoleOutput.setScrollTop(Double.MAX_VALUE);
        });
    }

    private void initializeFieldMap() {
        fieldMapCanvas = new Canvas(300, 300);
        fieldMapGc = fieldMapCanvas.getGraphicsContext2D();
        updateFieldMap();
    }

    private void updateRobotPosition() {
        // This is a placeholder. In a real application, you would calculate the robot's position
        // based on sensor data, vision processing results, or odometry.
        robotPosition = new Point2D(
            currentPose.getX() * 10, // Scale the position to fit the field map
            currentPose.getY() * 10
        );
    }

    private void updateGameObjectPositions() {
        // This is a placeholder. In a real application, you would update game object positions
        // based on vision processing results.
        for (GameObject obj : detectedObjects) {
            gameObjectPositions.put(obj.id, new Point2D(obj.x * 10, obj.y * 10));
        }
    }

    private void updateFieldMap() {
        fieldMapGc.clearRect(0, 0, fieldMapCanvas.getWidth(), fieldMapCanvas.getHeight());

        // Draw field boundaries
        fieldMapGc.setStroke(Color.BLACK);
        fieldMapGc.strokeRect(0, 0, fieldMapCanvas.getWidth(), fieldMapCanvas.getHeight());

        // Draw robot
        fieldMapGc.setFill(robotColor);
        fieldMapGc.fillOval(robotPosition.getX() - 5, robotPosition.getY() - 5, 10, 10);

        // Draw game objects
        fieldMapGc.setFill(gameObjectColor);
        for (Point2D objPos : gameObjectPositions.values()) {
            fieldMapGc.fillRect(objPos.getX() - 3, objPos.getY() - 3, 6, 6);
        }

        // Draw obstacles
        fieldMapGc.setFill(obstacleColor);
        for (Obstacle obstacle : detectedObstacles) {
            fieldMapGc.fillRect(obstacle.x * 10 - 3, obstacle.y * 10 - 3, 6, 6);
        }
    }

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle(appName);

        BorderPane root = new BorderPane();

        MenuBar menuBar = createMenuBar();
        root.setTop(menuBar);
        root.setCenter(tabPane);

        Scene scene = new Scene(root, 1200, 800);
        scene.getStylesheets().add(getClass().getResource("/styles.css").toExternalForm());
        primaryStage.setScene(scene);
        primaryStage.show();

        startAnimationTimer();
    }

    private MenuBar createMenuBar() {
        MenuBar menuBar = new MenuBar();

        Menu fileMenu = new Menu("File");
        MenuItem exitItem = new MenuItem("Exit");
        exitItem.setOnAction(e -> System.exit(0));
        fileMenu.getItems().add(exitItem);

        Menu editMenu = new Menu("Edit");
        MenuItem renameItem = new MenuItem("Rename App");
        renameItem.setOnAction(e -> renameApp());
        editMenu.getItems().add(renameItem);

        Menu helpMenu = new Menu("Help");
        MenuItem aboutItem = new MenuItem("About");
        aboutItem.setOnAction(e -> showAboutDialog());
        helpMenu.getItems().add(aboutItem);

        menuBar.getMenus().addAll(fileMenu, editMenu, helpMenu);

        return menuBar;
    }

        private void renameApp() {
        TextInputDialog dialog = new TextInputDialog(appName);
        dialog.setTitle("Rename App");
        dialog.setHeaderText("Enter new app name:");
        dialog.setContentText("Name:");

        Optional<String> result = dialog.showAndWait();
        result.ifPresent(name -> {
            appName = name;
            Stage stage = (Stage) tabPane.getScene().getWindow();
            stage.setTitle(appName);
        });
    }

    private void showAboutDialog() {
        Alert alert = new Alert(AlertType.INFORMATION);
        alert.setTitle("About " + appName);
        alert.setHeaderText(null);
        alert.setContentText("This is an advanced vision system for FRC robots.\nVersion: 1.0\nDeveloped by: Your Team");
        alert.showAndWait();
    }

    private void toggleTheme(boolean isDark) {
        isDarkTheme = isDark;
        Scene scene = tabPane.getScene();
        if (isDarkTheme) {
            scene.getStylesheets().remove("/styles.css");
            scene.getStylesheets().add("/dark-styles.css");
        } else {
            scene.getStylesheets().remove("/dark-styles.css");
            scene.getStylesheets().add("/styles.css");
        }
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
                updateObjectCountChart();
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

    private void performStrategicAnalysis() {
        // Implement game-specific strategic analysis here
        // This could involve analyzing the positions of game objects, obstacles, and the robot
        // to make decisions about autonomous actions or provide driver assistance
        logMessage("Performing strategic analysis...");
        if (isNearScoringPosition()) {
            logMessage("Near scoring position. Recommending alignment.");
        }
    }

    private boolean isNearScoringPosition() {
        // Implement logic to determine if the robot is near a scoring position
        return currentPose.getTranslation().getDistance(new Translation2d(5, 5)) < 1.0;
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

        public void trainModel(List<Mat> trainingImages, List<Integer> labels) {
            // Implement model training here
            logMessage("Training AI model...");
            // This is a placeholder for demonstration
            try {
                Thread.sleep(2000); // Simulate training time
                logMessage("AI model training complete!");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
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
