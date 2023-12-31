import java.net.*;
import java.io.*;

/**
 * A sample (Java) program to send UDP packets containing
 * textual messages to a (presumably) remote program.
 *
 * This was originally written as a way to send test data to
 * Ethernet-enabled Arduino applications (e.g., to control
 * lighting code, etc.), but could be used for other purposes.
 */
class UdpPacketSender {
  public static void main(String [] args) throws IOException {
	// Parameter sanitization
    if (args.length != 3) {
      System.err.println("Usage: java UdpPacketSender host_or_ip port text");
	  System.exit(1);
	}
    final String hostname = args[0];
	final int port = Integer.decode(args[1]);
	final String msg = args[2];
 
	// Translate the 1st parameter into an actual IP address.
    final InetAddress address = InetAddress.getByName(hostname);

	// Allocate a socket for use in sending data packets.
	//
	// Note that we don't need to specify any information about
	// IP addresses or ports here; this is the equivalent of
	// the "drop box" we'll use to put our packet onto the
	// network; the *packet* needs to be fully "addressed" for
	// delivery..
    final DatagramSocket socket = new DatagramSocket();

	// Build the "packet" to be sent on the network.
    final byte[] buffer = msg.getBytes();
    final DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, port);
 
	// Send the data.
    socket.send(packet);
  }
}
