import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Map;

public class VisionCalculation {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static int findBestProbabilityIndex(List<Map<String, Object>> predictions) {
        int bestIndex = 0;
        for (int x = 0; x < predictions.size(); x++) {
            if ((double) predictions.get(x).get("probability") > (double) predictions.get(bestIndex).get("probability")) {
                bestIndex = x;
            }
        }
        return bestIndex;
    }

    public static Object[] findObjectCorners(List<Map<String, Object>> predictions, int bestIndex, double probabilityThreshold, int[] frameShape) {
        if (predictions.size() > 0) {
            if ((double) predictions.get(bestIndex).get("probability") > probabilityThreshold) {
                System.out.println(predictions.get(bestIndex));

                // Get the bounding box of the object
                Map<String, Object> bBox = (Map<String, Object>) predictions.get(bestIndex).get("boundingBox");

                // Convert the decimal position of the left side of the bounding box to an x-coordinate
                int left = (int) ((double) bBox.get("left") * frameShape[1]);
                // Convert the decimal position of the top side of the bounding box to a y-coordinate
                int top = (int) ((double) bBox.get("top") * frameShape[0]);
                // Convert the decimal width of the bounding box to pixels
                int width = (int) ((double) bBox.get("width") * frameShape[1]);
                // Convert the decimal height of the bounding box to pixels
                int height = (int) ((double) bBox.get("height") * frameShape[0]);

                int[] topLeftCorner = {left, top};
                int[] bottomRightCorner = {left + width, top + height};

                return new Object[]{topLeftCorner, bottomRightCorner, width, height};
            }
        }
        return new Object[]{new int[]{0, 0}, new int[]{0, 0}, 0, 0};
    }

    public static double getDistanceToCamera(double knownHeight, double knownFocal, double heightPixels) {
        if (heightPixels > 0) {
            return (knownHeight * knownFocal) / heightPixels;
        }
        return -1;
    }

    public static Mat resizeDownTo1600MaxDim(Mat image) {
        int h = image.rows();
        int w = image.cols();
        if (h < 1600 && w < 1600) {
            return image;
        }

        Size newSize;
        if (h > w) {
            newSize = new Size(1600 * w / h, 1600);
        } else {
            newSize = new Size(1600, 1600 * h / w);
        }
        Mat resizedImage = new Mat();
        Imgproc.resize(image, resizedImage, newSize, 0, 0, Imgproc.INTER_LINEAR);
        return resizedImage;
    }
}
