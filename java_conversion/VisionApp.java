import org.opencv.core.*;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.*;
import java.net.InetSocketAddress;
import java.util.*;

import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpExchange;

public class VisionApp extends JFrame {
    private VideoCapture camera;
    private ObjectDetection odModel;
    private BufferedImage currentImage;
    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public VisionApp() {
        this.odModel = new ObjectDetection();
        this.camera = new VideoCapture(0);
        camera.set(Videoio.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
        camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
        startCamera();
        startHttpServer();
    }

    private void startCamera() {
        new Thread(() -> {
            Mat frame = new Mat();
            while (true) {
                if (camera.isOpened()) {
                    camera.read(frame);
                    if (!frame.empty()) {
                        processFrame(frame);
                    }
                }
                try {
                    Thread.sleep(30); // ~30 fps
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    private void processFrame(Mat frame) {
        currentImage = matToBufferedImage(frame);
        List<Detection> detections = odModel.detect(frame);
        drawDetections(frame, detections);
    }

    private void drawDetections(Mat frame, List<Detection> detections) {
        for (Detection detection : detections) {
            Rect box = detection.box;
            Imgproc.rectangle(frame, box, new Scalar(0, 255, 0), 2);
            String label = String.format("%s: %.2f", detection.className, detection.confidence);
            Point labelPos = new Point(box.x, box.y - 10);
            Imgproc.putText(frame, label, labelPos, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        }
    }

    private BufferedImage matToBufferedImage(Mat mat) {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        if (mat.channels() > 1) {
            type = BufferedImage.TYPE_3BYTE_BGR;
        }
        int bufferSize = mat.channels() * mat.cols() * mat.rows();
        byte[] b = new byte[bufferSize];
        mat.get(0, 0, b);
        BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
        final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
        System.arraycopy(b, 0, targetPixels, 0, b.length);
        return image;
    }

    private void startHttpServer() {
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(3000), 0);
            server.createContext("/image", new ImageHandler());
            server.setExecutor(null);
            server.start();
            System.out.println("Server started on port 3000");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private class ImageHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (currentImage != null) {
                exchange.getResponseHeaders().add("Content-Type", "image/jpeg");
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                ByteArrayOutputStream byteOutputStream = new ByteArrayOutputStream();
                ImageIO.write(currentImage, "jpg", byteOutputStream);
                byte[] imageBytes = byteOutputStream.toByteArray();
                exchange.sendResponseHeaders(200, imageBytes.length);
                OutputStream os = exchange.getResponseBody();
                os.write(imageBytes);
                os.close();
            } else {
                exchange.sendResponseHeaders(204, -1);
            }
        }
    }

    static class Detection {
        Rect box;
        float confidence;
        String className;

        Detection(Rect box, float confidence, String className) {
            this.box = box;
            this.confidence = confidence;
            this.className = className;
        }
    }

    static class ObjectDetection {
        private Net net;
        private List<String> classes;
        private static final float CONFIDENCE_THRESHOLD = 0.5f;
        private static final float NMS_THRESHOLD = 0.4f;

        public ObjectDetection() {
            try {
                // Load the model and classes
                net = Dnn.readNetFromDarknet("yolov3.cfg", "yolov3.weights");
                net.setPreferableBackend(Dnn.DNN_BACKEND_OPENCV);
                net.setPreferableTarget(Dnn.DNN_TARGET_CPU);
                
                classes = new ArrayList<>();
                BufferedReader reader = new BufferedReader(new FileReader("coco.names"));
                String line;
                while ((line = reader.readLine()) != null) {
                    classes.add(line);
                }
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public List<Detection> detect(Mat frame) {
            List<Detection> detections = new ArrayList<>();
            
            Mat blob = Dnn.blobFromImage(frame, 1/255.0, new Size(416, 416), new Scalar(0), true, false);
            net.setInput(blob);

            List<Mat> result = new ArrayList<>();
            List<String> outBlobNames = net.getUnconnectedOutLayersNames();
            net.forward(result, outBlobNames);

            List<Rect> boxes = new ArrayList<>();
            List<Float> confidences = new ArrayList<>();
            List<Integer> classIds = new ArrayList<>();

            for (Mat level : result) {
                for (int i = 0; i < level.rows(); i++) {
                    Mat row = level.row(i);
                    Mat scores = row.colRange(5, level.cols());
                    Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
                    float confidence = (float)mm.maxVal;
                    Point classIdPoint = mm.maxLoc;

                    if (confidence > CONFIDENCE_THRESHOLD) {
                        int centerX = (int)(row.get(0,0)[0] * frame.cols());
                        int centerY = (int)(row.get(0,1)[0] * frame.rows());
                        int width = (int)(row.get(0,2)[0] * frame.cols());
                        int height = (int)(row.get(0,3)[0] * frame.rows());
                        int left = centerX - width/2;
                        int top = centerY - height/2;

                        boxes.add(new Rect(left, top, width, height));
                        confidences.add(confidence);
                        classIds.add((int)classIdPoint.x);
                    }
                }
            }

            MatOfRect2d boxesMatrix = new MatOfRect2d();
            MatOfFloat confidencesMatrix = new MatOfFloat();
            
            if (!boxes.isEmpty()) {
                boxesMatrix.fromList(boxes.stream().map(b -> new Rect2d(b.x , b.y, b.width, b.height)).collect(Collectors.toList()));
                confidencesMatrix.fromList(confidences);
            }

            MatOfInt indices = new MatOfInt();
            Dnn.NMSBoxes(boxesMatrix, confidencesMatrix, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);

            for (int i = 0; i < indices.total(); i++) {
                int idx = (int)indices.get(i, 0)[0];
                Rect box = boxes.get(idx);
                float confidence = confidences.get(idx);
                int classId = classIds.get(idx);
                String className = classes.get(classId);
                detections.add(new Detection(box, confidence, className));
            }

            return detections;
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new VisionApp());
    }
}
