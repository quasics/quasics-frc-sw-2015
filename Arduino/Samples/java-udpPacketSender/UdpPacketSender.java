import java.net.*;
import java.io.*;

class UdpPacketSender {
  public static void main(String [] args) throws IOException {
    if (args.length != 3) {
      System.err.println("Usage: java UdpPacketSender host_or_ip port text");
	  System.exit(1);
	}
    final String hostname = args[0];
	final int port = Integer.decode(args[1]);
	final String msg = args[2];
 
    InetAddress address = InetAddress.getByName(hostname);
    DatagramSocket socket = new DatagramSocket();
 
    final byte[] buffer = msg.getBytes();
 
    DatagramPacket request = new DatagramPacket(buffer, buffer.length, address, port);
    socket.send(request);
  }
}
