import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetection {

    private static final double[][] ANCHORS = {
            {0.573, 0.677}, {1.87, 2.06}, {3.34, 5.47}, {7.88, 3.53}, {9.77, 9.17}
    };
    private static final double IOU_THRESHOLD = 0.45;

    private String[] labels;
    private double probThreshold;
    private int maxDetections;

    public ObjectDetection(String[] labels, double probThreshold, int maxDetections) {
        if (labels.length < 1) {
            throw new IllegalArgumentException("At least 1 label is required");
        }
        this.labels = labels;
        this.probThreshold = probThreshold;
        this.maxDetections = maxDetections;
    }

    private double[] logistic(double x) {
        return x > 0 ? new double[]{1 / (1 + Math.exp(-x))} : new double[]{Math.exp(x) / (1 + Math.exp(x))};
    }

    private List<BoundingBox> nonMaximumSuppression(double[][] boxes, double[][] classProbs, int maxDetections) {
        int numBoxes = boxes.length;
        List<BoundingBox> selectedBoxes = new ArrayList<>();
        double[] maxProbs = new double[numBoxes];
        int[] maxClasses = new int[numBoxes];

        for (int i = 0; i < numBoxes; i++) {
            maxProbs[i] = Double.NEGATIVE_INFINITY;
            for (int j = 0; j < classProbs[i].length; j++) {
                if (classProbs[i][j] > maxProbs[i]) {
                    maxProbs[i] = classProbs[i][j];
                    maxClasses[i] = j;
                }
            }
        }

        while (selectedBoxes.size() < maxDetections) {
            int i = -1;
            double maxProb = Double.NEGATIVE_INFINITY;
            for (int j = 0; j < maxProbs.length; j++) {
                if (maxProbs[j] > maxProb) {
                    maxProb = maxProbs[j];
                    i = j;
                }
            }

            if (maxProb < probThreshold) {
                break;
            }

            selectedBoxes.add(new BoundingBox(boxes[i], maxClasses[i], maxProb));

            // Remove overlapping boxes
            for (int j = 0; j < numBoxes; j++) {
                if (i != j) {
                    double iou = calculateIoU(boxes[i], boxes[j]);
                    if (iou > IOU_THRESHOLD) {
                        maxProbs[j] = 0; // Suppress this box
                    }
                }
            }
        }

        return selectedBoxes;
    }

    private double calculateIoU(double[] boxA, double[] boxB) {
        double x1 = Math.max(boxA[0], boxB[0]);
        double y1 = Math.max(boxA[1], boxB[1]);
        double x2 = Math.min(boxA[0] + boxA[2], boxB[0] + boxB[2]);
        double y2 = Math.min(boxA[1] + boxA[3], boxB[1] + boxB[3]);

        double intersectionArea = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
        double boxAArea = boxA[2] * boxA[3];
        double boxBArea = boxB[2] * boxB[3];

        return intersectionArea / (boxAArea + boxBArea - intersectionArea);
    }

    private double[][] extractBoundingBoxes(double[][] predictionOutput) {
        // Implement the logic to extract bounding boxes from prediction output
        // This is a placeholder for the actual implementation
        return new double[0][0];
    }

    public List<Prediction> predictImage(BufferedImage image) {
        BufferedImage preprocessedImage = preprocess(image);
        double[][] predictionOutputs = predict(preprocessedImage);
        return postprocess(predictionOutputs);
    }

    private BufferedImage preprocess(BufferedImage image) {
        // Resize image to (416, 416)
        BufferedImage resizedImage = new BufferedImage(416, 416 , BufferedImage.TYPE_INT_RGB);
        // Implement image resizing logic
        return resizedImage;
    }

    private double[][] predict(BufferedImage preprocessedImage) {
        // Implement the logic to evaluate the model and get the output
        // This is a placeholder for the actual implementation
        return new double[0][0];
    }

    private List<Prediction> postprocess(double[][] predictionOutputs) {
        double[][] boxes = extractBoundingBoxes(predictionOutputs);
        double[][] classProbs = new double[boxes.length][labels.length];

        // Calculate class probabilities
        for (int i = 0; i < boxes.length; i++) {
            for (int j = 0; j < labels.length; j++) {
                classProbs[i][j] = calculateClassProbability(boxes[i], j);
            }
        }

        List<BoundingBox> selectedBoxes = nonMaximumSuppression(boxes, classProbs, maxDetections);

        List<Prediction> predictions = new ArrayList<>();
        for (BoundingBox box : selectedBoxes) {
            predictions.add(new Prediction(box.probability, box.tagId, labels[box.tagId], box.boundingBox));
        }

        return predictions;
    }

    private double calculateClassProbability(double[] box, int classId) {
        // Implement the logic to calculate class probability
        // This is a placeholder for the actual implementation
        return 0.0;
    }

    private static class BoundingBox {
        double[] box;
        int tagId;
        double probability;

        public BoundingBox(double[] box, int tagId, double probability) {
            this.box = box;
            this.tagId = tagId;
            this.probability = probability;
        }

        public double getLeft() {
            return box[0];
        }

        public double getTop() {
            return box[1];
        }

        public double getWidth() {
            return box[2];
        }

        public double getHeight() {
            return box[3];
        }

        public double getProbability() {
            return probability;
        }

        public int getTagId() {
            return tagId;
        }

        public String getTagName() {
            return labels[tagId];
        }

        public double[] getBoundingBox() {
            return box;
        }
    }

    private static class Prediction {
        double probability;
        int tagId;
        String tagName;
        double[] boundingBox;

        public Prediction(double probability, int tagId, String tagName, double[] boundingBox) {
            this.probability = probability;
            this.tagId = tagId;
            this.tagName = tagName;
            this.boundingBox = boundingBox;
        }
    }
}
