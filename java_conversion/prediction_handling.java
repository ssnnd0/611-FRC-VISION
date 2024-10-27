import org.tensorflow.Graph;
import org.tensorflow.Session;
import org.tensorflow.Tensor;
import org.tensorflow.SavedModelBundle;
import org.tensorflow.ndarray.Shape;
import org.tensorflow.ndarray.NdArray;
import org.tensorflow.ndarray.NdArrays;
import org.tensorflow.types.TFloat32;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PredictionHandling {

    private static final String MODEL_FILENAME = "../model.pb";
    private static final String LABELS_FILENAME = "../labels.txt";
    private static TFObjectDetection odModel;

    public static void load() throws IOException {
        // Load a TensorFlow model
        SavedModelBundle model = SavedModelBundle.load(MODEL_FILENAME, "serve");
        List<String> labels = loadLabels(LABELS_FILENAME);
        odModel = new TFObjectDetection(model, labels);
    }

    private static List<String> loadLabels(String filename) throws IOException {
        List<String> labels = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            while ((line = br.readLine()) != null) {
                labels.add(line.trim());
            }
        }
        return labels;
    }

    public static List<Prediction> predict(BufferedImage frame) {
        BufferedImage rgbFrame = new BufferedImage(frame.getWidth(), frame.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgbFrame.getGraphics().drawImage(frame, 0, 0, null);
        return odModel.predictImage(rgbFrame);
    }

    private static class TFObjectDetection extends ObjectDetection {
        private final SavedModelBundle model;

        public TFObjectDetection(SavedModelBundle model, List<String> labels) {
            super(labels.toArray(new String[0]), 0.10, 20);
            this.model = model;
        }

        public double[][] predict(BufferedImage preprocessedImage) {
            // Convert BufferedImage to float array
            float[][][] inputs = new float[1][416][416][3]; // Assuming input size is 416x416
            for (int y = 0; y < preprocessedImage.getHeight(); y++) {
                for (int x = 0; x < preprocessedImage.getWidth(); x++) {
                    int rgb = preprocessedImage.getRGB(x, y);
                    inputs[0][y][x][0] = ((rgb >> 16) & 0xFF) / 255.0f; // R
                    inputs[0][y][x][1] = ((rgb >> 8) & 0xFF) / 255.0f;  // G
                    inputs[0][y][x][2] = (rgb & 0xFF) / 255.0f;         // B
                }
            }

            // Create a Tensor from the input array
            Tensor<TFloat32> inputTensor = TFloat32.tensorOf(Shape.of(1, 416, 416, 3), NdArrays.vectorOf(inputs));
            Tensor<TFloat32> outputTensor = model.session().runner()
                    .fetch("model_outputs:0") // Adjust the output tensor name as needed
                    .feed("Placeholder:0", inputTensor)
                    .run()
                    .get(0)
                    .expect(TFloat32.DTYPE);

            // Process output tensor to extract predictions
            double[][] outputs = new double[(int) outputTensor.shape().size(0)][(int) outputTensor.shape().size(1)];
            // Convert output tensor to a 2D array (implement as needed)

            return outputs;
        }
    }
}
