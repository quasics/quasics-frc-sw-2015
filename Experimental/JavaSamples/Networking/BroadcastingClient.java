import java.io.*;
import java.net.*;
import java.util.*;

public class BroadcastingClient {

    public static void main(String[] args) throws IOException {
        broadcast("on\000", 10900);
    }

    public static void broadcast(
        String message, int port) throws IOException {
        broadcast(message, port, InetAddress.getByName("0.0.0.0"));
    }

    public static void broadcast(
        String message, int port, InetAddress address) throws IOException {
        DatagramSocket socket = new DatagramSocket();
        try {
            socket.setBroadcast(true);

            byte[] buffer = message.getBytes();

            DatagramPacket packet
                = new DatagramPacket(buffer, buffer.length, address, port);
            socket.send(packet);
        }
        finally {
            socket.close();
        }
    }
}
