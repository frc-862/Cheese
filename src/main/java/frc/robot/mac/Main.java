package frc.robot.mac;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Main {
    public static void main(String[] args) {
        System.out.println("Lets go it works");
        // MacMini mac = new MacMini();

        // mac.run();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        try {
            Thread.sleep(3000);
        } catch (Exception e) {}

        System.out.println("Netowrk Tables Connected: " + nt.isConnected());
    }
}
