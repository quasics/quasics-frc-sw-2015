// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.io.Writer;

/**
 * A trivial implementation of the Writer type, which handles all operations as no-ops.
 */
public class NullWriter extends Writer {
  @Override
  public void write(char[] cbuf, int off, int len) throws IOException {
    // No-op
  }

  @Override
  public void flush() throws IOException {
    // No-op
  }

  @Override
  public void close() throws IOException {
    // No-op
  }
};
