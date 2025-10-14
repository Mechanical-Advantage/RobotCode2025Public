// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Utility class to log the list of NetworkTables clients. */
public class NTClientLogger {
  private static final String tableName = "NTClients/";
  private static Set<String> lastRemoteIds = new HashSet<>();
  private static ByteBuffer intBuffer = ByteBuffer.allocate(4);

  private NTClientLogger() {}

  public static void log() {
    ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
    Set<String> remoteIds = new HashSet<>();

    // Log data for connected clients
    for (int i = 0; i < connections.length; i++) {
      lastRemoteIds.remove(connections[i].remote_id);
      remoteIds.add(connections[i].remote_id);
      Logger.recordOutput(tableName + connections[i].remote_id + "/Connected", true);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/IPAddress", connections[i].remote_ip);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/RemotePort", connections[i].remote_port);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/LastUpdate", connections[i].last_update);
      intBuffer.rewind();
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/ProtocolVersion",
          intBuffer.putInt(connections[i].protocol_version).array());
    }

    // Mark disconnected clients
    for (var remoteId : lastRemoteIds) {
      Logger.recordOutput(tableName + remoteId + "/Connected", false);
    }
    lastRemoteIds = remoteIds;
  }
}
