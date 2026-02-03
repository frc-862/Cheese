package frc.robot.mac;

public class Main {
    public static void main(String[] args) {
        MacMini mac = new MacMini();

        mac.run();

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            mac.shutdown();
        }));

        // NetworkTableInstance nt = NetworkTableInstance.create();
        // nt.setServer("10.8.62.2", 5810);
        // nt.startClient4("mac");

        // while (!nt.isConnected()) {
        //     System.out.println("Connecting");

        //     try {
        //         Thread.sleep(1000);
        //     } catch (Exception e) {
        //     }
        // }


    }
}
