import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.highgui.HighGui;

import java.util.Scanner;

public class ObjectDetectionApp {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private static final double FOCAL_LENGTH = 708.897;
    private static final int PROBABILITY_THRESHOLD = 75;

    public static void main(String[] args) {
        // Open a video capture
        VideoCapture cap = new VideoCapture(2);
        if (!cap.isOpened()) {
            System.out.println("Error: Camera not found.");
            return;
        }

        Mat firstFrame = new Mat();
        cap.read(firstFrame);
        Size frameShape = firstFrame.size();

        // Load the TensorFlow model and setup
        PredictionHandling.load();
        Networking.initialize();

        Scanner scanner = new Scanner(System.in);

        while (true) {
            Mat frame = new Mat();
            cap.read(frame);

            // Resize frame
            frame = VisionCalculation.resizeDownTo1600MaxDim(frame);

            // Feed frame into model
            double[] predictions = PredictionHandling.predict(frame);

            // Find best index
            int bestIndex = VisionCalculation.findBestProbabilityIndex(predictions);

            // Find object corners
            ObjectCorners corners = VisionCalculation.findObjectCorners(predictions, bestIndex, PROBABILITY_THRESHOLD, frameShape);
            double distanceToCamera = VisionCalculation.getDistanceToCamera(11, FOCAL_LENGTH, corners.height);

            // Draw object outline and distance data to screen
            showFrame(frame, corners.topLeft, corners.bottomRight, distanceToCamera, frameShape);

            // Program end clause
            if (scanner.hasNext() && scanner.next().equals("x")) {
                cap.release();
                HighGui.destroyAllWindows();
                break;
            }
        }
    }

    // Draw object outline and vision data to screen, then show it
    private static void showFrame(Mat frame, Point objTopLeft, Point objBottomRight, double distance, Size frameShape) {
        Imgproc.rectangle(frame, objTopLeft, objBottomRight, new Scalar(0, 255, 0), 2);

        String distanceString = "Distance: " + distance;
        Imgproc.putText(frame, distanceString, new Point(frameShape.width - 200, frameShape.height - 200), 
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.75, new Scalar(255, 255, 255), 2);

        HighGui.imshow("Image Processing", frame);
        HighGui.waitKey(500);
    }
}
