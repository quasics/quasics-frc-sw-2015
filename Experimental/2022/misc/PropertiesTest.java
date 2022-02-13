import java.io.*;
import java.util.*;

public class PropertiesTest {
  public static void main(String [] args) throws Exception {
	Properties props = new Properties();
	props.setProperty("key1", "value1");
	props.setProperty("bool1", Boolean.toString(true));
	props.setProperty("real1", Double.toString(6.5));
    FileOutputStream fout = new FileOutputStream("example.props");
	props.store(fout, "Example properties");
	fout.close();
  }
}
