import java.io.*;
import java.net.*;
import java.util.*;

public class BroadcastingClient {
 
    public static void main(String[] args) throws IOException {
        broadcast("on\000", InetAddress.getByName("0.0.0.0"));
    }
 
    public static void broadcast(
      String broadcastMessage, InetAddress address) throws IOException {
        DatagramSocket socket = new DatagramSocket();
        socket.setBroadcast(true);
 
        byte[] buffer = broadcastMessage.getBytes();
 
        DatagramPacket packet 
          = new DatagramPacket(buffer, buffer.length, address, 10900);
        socket.send(packet);
        socket.close();
    }
}
