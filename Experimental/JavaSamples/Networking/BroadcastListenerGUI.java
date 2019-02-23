import java.awt.*;
import java.awt.event.ActionListener;
import java.io.*;
import java.net.*;
import java.util.*;
import javax.swing.*;

public class BroadcastListenerGUI {

    private class Listener implements Runnable {
        private int port;
        private JTextArea text;
        private DatagramSocket sock = null;
        private boolean cancelled = false;

        Listener(int port, JTextArea text) throws SocketException {
            this.port = port;
            this.text = text;

            sock = new DatagramSocket(this.port);
            sock.setSoTimeout(10);
        }

        void cancel() {
            cancelled = true;
        }

        public void run() {
            text.setText("");
            try {
                DatagramPacket p;
                byte msg[] = new byte[1024];
                p = new DatagramPacket(msg, msg.length);

                System.out.println("Starting to listen on port " + this.port);

                while (!cancelled) {
                    try {
                        sock.receive(p);
                    } catch (SocketTimeoutException ste) {
                        continue;
                    }

                    String gotText = new String(p.getData(), 0, p.getLength());
                    text.append(gotText + '\n');
                }
            } catch (Exception e) {
                System.err.println("Caught exception: " + e);
            } finally {
                if (sock != null) {
                    sock.close();
                }
            }
            System.out.println("Stopped listening on port " + this.port);
        }
    }

    Listener l = null;

    final static int BROADCAST_PORT = 10900;

    static final String START_TEXT = "Start Listening";
    static final String STOP_TEXT = "Stop Listening";

    BroadcastListenerGUI() {
        final String title = "Broadcasting Test";

        //
        // Create our main frame
        //
        final JFrame win = new JFrame(title) {
            private static final long serialVersionUID = 1;

            public Dimension getMinimumSize() {
                return getPreferredSize();
            }

            public Dimension getMaximumSize() {
                return getPreferredSize();
            }
        };

        win.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        Container content = win.getContentPane();
        content.setLayout(new BorderLayout());

        Container portSettings = new JPanel(new GridLayout(0, 2));
        portSettings.add(new JLabel("Port"));
        JTextField portField = new JTextField(Integer.toString(BROADCAST_PORT));
        portSettings.add(portField);

        JTextArea messages = new JTextArea(10, 20);
        messages.setEditable(false);
        JScrollPane scrollPane = new JScrollPane(messages);
        content.add(scrollPane, BorderLayout.CENTER);

        JButton openClose = new JButton(START_TEXT);
        ActionListener openCloseListener = event -> {
            if (l != null) {
                l.cancel();
                l = null;
                openClose.setText(START_TEXT);
            } else {
                final int port = Integer.parseInt(portField.getText());
                try {
                    l = new Listener(port, messages);
                    new Thread(l).start();
                    openClose.setText(STOP_TEXT);
                } catch (Exception e) {
                    JOptionPane.showMessageDialog(win, "Error",
                            "Failed to start listening for traffic:\n" + e.getMessage(), JOptionPane.ERROR_MESSAGE);
                }
            }
        };

        openClose.addActionListener(openCloseListener);
        Container bottomPanel = new JPanel(new FlowLayout());
        bottomPanel.add(openClose);

        content.add(portSettings, BorderLayout.NORTH);
        content.add(scrollPane, BorderLayout.CENTER);
        content.add(bottomPanel, BorderLayout.SOUTH);

        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                win.pack();

                // Make the window show up centered on the (primary) monitor.
                Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
                win.setLocationRelativeTo(null);
                win.setLocation(dim.width / 2 - win.getSize().width / 2, dim.height / 2 - win.getSize().height / 2);
                win.setVisible(true);
            }
        });
    }

    public static void main(String args[]) {
        new BroadcastListenerGUI();
    }
}
