package frc.robot.vision;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.stage.Stage;

public class VisionDashboard extends Application {
    private VisionConfig visionConfig;
    private VisionTrackingService trackingService;

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle("PhotonVision Dashboard 2025");
        
        VBox mainLayout = new VBox(10);
        mainLayout.setPadding(new Insets(15));

        // Target Information Section
        TitledPane targetInfoPane = createTargetInfoPane();
        
        // Camera Configuration Section
        TitledPane cameraConfigPane = createCameraConfigPane();
        
        // Tracking Metrics
        TitledPane trackingMetricsPane = createTrackingMetricsPane();

        Accordion accordion = new Accordion(
            targetInfoPane, 
            cameraConfigPane, 
            trackingMetricsPane
        );

        mainLayout.getChildren().add(accordion);

        Scene scene = new Scene(mainLayout, 600, 800);
        scene.getStylesheets().add(getClass().getResource("/dark-theme.css").toExternalForm());
        
        primaryStage.setScene(scene);
        primaryStage.show();

        // Start periodic updates
        startPeriodicUpdates();
    }

    private TitledPane createTargetInfoPane() {
        GridPane grid = new GridPane();
        grid.setHgap(10);
        grid.setVgap(10);

        // Add target information labels and values
        return new TitledPane("Target Information", grid);
    }

    private TitledPane createCameraConfigPane() {
        VBox configBox = new VBox(10);
        // Add camera configuration controls
        return new TitledPane("Camera Configuration", configBox);
    }

    private TitledPane createTrackingMetricsPane() {
        GridPane metricsGrid = new GridPane();
        // Add tracking metrics
        return new TitledPane("Tracking Metrics", metricsGrid);
    }

    private void startPeriodicUpdates() {
        // Implement periodic UI updates
    }

    public static void main(String[] args) {
        launch(args);
    }
}
