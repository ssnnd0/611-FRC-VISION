import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.concurrent.CountDownLatch;

public class Networking {

    private static final String SERVER_IP = "10.52.88.2";
    private static CountDownLatch connectionLatch = new CountDownLatch(1);
    private static NetworkTable visionTable;

    // Listener for connection events
    private static void connectionListener(boolean connected) {
        System.out.println("Connected: " + connected);
        if (connected) {
            connectionLatch.countDown(); // Signal that connection is established
        }
    }

    public static void initialize() {
        // Initialize NetworkTables and add a listener for connection events
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient(SERVER_IP);
        inst.addConnectionListener((table, connected) -> connectionListener(connected), true);
        visionTable = inst.getTable("SmartDashboard");
    }

    public static void waitForConnection() {
        try {
            System.out.println("Waiting for connection...");
            connectionLatch.await(); // Wait until the connection is established
            System.out.println("Connected!");
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
            System.err.println("Connection wait interrupted: " + e.getMessage());
        }
    }

    public static void putNumber(String key, double value) {
        if (visionTable != null) {
            visionTable.getEntry(key).setDouble(value);
        } else {
            System.err.println("Vision table is not initialized.");
        }
    }
}
