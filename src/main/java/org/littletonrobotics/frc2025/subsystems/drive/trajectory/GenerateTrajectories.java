// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import com.google.common.hash.Hashing;
import edu.wpi.first.math.util.Units;
import io.grpc.Grpc;
import io.grpc.InsecureChannelCredentials;
import java.io.*;
import java.math.RoundingMode;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.*;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceGrpc;

public class GenerateTrajectories {
  public static void main(String[] args) {
    Constants.disableHAL();

    // Create vehicle model
    VehicleModel model =
        VehicleModel.newBuilder()
            .setMass(67)
            .setMoi(5.8)
            .setVehicleLength(DriveConstants.trackWidthX)
            .setVehicleWidth(DriveConstants.trackWidthY)
            .setWheelRadius(Units.inchesToMeters(2.0))
            .setMaxWheelTorque(3.0)
            .setMaxWheelOmega(DriveConstants.maxLinearSpeed / Units.inchesToMeters(2.0) * 0.75)
            .build();

    // Connect to service
    var channel =
        Grpc.newChannelBuilder("127.0.0.1:56328", InsecureChannelCredentials.create()).build();
    var service = VehicleTrajectoryServiceGrpc.newBlockingStub(channel);

    Set<String> completedPaths = new HashSet<>();
    while (true) {
      // Get all paths
      var allPaths = DriveTrajectories.paths;
      boolean hasAllPaths = true;
      for (var supplier : DriveTrajectories.suppliedPaths) {
        var suppliedPaths = supplier.apply(completedPaths);
        if (suppliedPaths == null) {
          hasAllPaths = false;
        } else {
          allPaths.putAll(suppliedPaths);
        }
      }
      Set<String> originalKeys = new HashSet<>();
      originalKeys.addAll(allPaths.keySet());
      for (String name : originalKeys) {
        if (completedPaths.contains(name)) {
          allPaths.remove(name);
        } else {
          completedPaths.add(name);
        }
      }
      if (!hasAllPaths && allPaths.size() == 0) {
        throw new RuntimeException(
            "Invalid dependency tree. Not all supplied trajectories are available, but there are no trajectories to generate.");
      }

      // Check hashcodes
      Map<String, List<PathSegment>> pathQueue = new HashMap<>();
      for (Map.Entry<String, List<PathSegment>> entry : allPaths.entrySet()) {
        String hashCode = getHashCode(model, entry.getValue());
        File pathFile =
            Path.of("src", "main", "deploy", "trajectories", entry.getKey() + ".pathblob").toFile();
        if (pathFile.exists()) {
          try {
            InputStream fileStream = new FileInputStream(pathFile);
            Trajectory trajectory = Trajectory.parseFrom(fileStream);
            if (!trajectory.getHashCode().equals(hashCode)) {
              pathQueue.put(entry.getKey(), entry.getValue());
            }
          } catch (IOException e) {
            e.printStackTrace();
          }
        } else {
          pathQueue.put(entry.getKey(), entry.getValue());
        }
      }

      // Generate trajectories
      String generateEmptyFlag = System.getenv("GENERATE_EMPTY_DRIVE_TRAJECTORIES");
      boolean generateEmpty = generateEmptyFlag != null && generateEmptyFlag.length() > 0;
      for (Map.Entry<String, List<PathSegment>> entry : pathQueue.entrySet()) {
        Trajectory trajectory;
        System.out.print(entry.getKey() + " - Generating 💭");
        double startTime = System.currentTimeMillis();
        if (generateEmpty) {
          trajectory =
              Trajectory.newBuilder()
                  .addStates(TimestampedVehicleState.newBuilder().build())
                  .build();
        } else {
          // Use service for generation
          PathRequest request =
              PathRequest.newBuilder().setModel(model).addAllSegments(entry.getValue()).build();
          TrajectoryResponse response = service.generateTrajectory(request);
          String error = response.getError().getReason();
          if (error.length() > 0) {
            System.err.println(
                "Got error response for trajectory \"" + entry.getKey() + "\": " + error);
            System.exit(1);
          }
          trajectory =
              response.getTrajectory().toBuilder()
                  .setHashCode(getHashCode(model, entry.getValue()))
                  .build();
        }
        File pathFile =
            Path.of("src", "main", "deploy", "trajectories", entry.getKey() + ".pathblob").toFile();
        try {
          OutputStream fileStream = new FileOutputStream(pathFile);
          trajectory.writeTo(fileStream);
          double endTime = System.currentTimeMillis();
          System.out.println(
              "\r"
                  + entry.getKey()
                  + " - Finished in "
                  + Math.round((endTime - startTime) / 100.0) / 10.0
                  + " secs ✅");
        } catch (IOException e) {
          System.out.println("\r" + entry.getKey() + " - FAILED ⛔");
          e.printStackTrace();
        }
      }

      // Exit if all trajectories ready
      if (hasAllPaths) {
        break;
      }
    }

    // Delete old trajectories
    try {
      Files.list(Path.of("src", "main", "deploy", "trajectories"))
          .forEach(
              (path) -> {
                String filename = path.getFileName().toString();
                if (!filename.endsWith(".pathblob")) return;
                String[] components = filename.split("\\.");
                if (components.length == 2 && !completedPaths.contains(components[0])) {
                  path.toFile().delete();
                  System.out.println(components[0] + " - Deleted 💀");
                }
              });
    } catch (IOException e) {
      e.printStackTrace();
    }
    System.out.println("All trajectories up-to-date!");
  }

  // create a hashcode for the vehicle model and path segments

  private static String getHashCode(VehicleModel model, List<PathSegment> segements) {
    StringBuilder hashString = new StringBuilder();

    DecimalFormat format = new DecimalFormat("#.000000");
    format.setRoundingMode(RoundingMode.HALF_DOWN);

    hashString.append(format.format(model.getMass()));
    hashString.append(format.format(model.getMoi()));
    hashString.append(format.format(model.getVehicleLength()));
    hashString.append(format.format(model.getVehicleWidth()));
    hashString.append(format.format(model.getWheelRadius()));
    hashString.append(format.format(model.getMaxWheelTorque()));
    hashString.append(format.format(model.getMaxWheelOmega()));

    for (PathSegment segment : segements) {
      for (Waypoint waypoint : segment.getWaypointsList()) {
        hashString.append(format.format(waypoint.getX()));
        hashString.append(format.format(waypoint.getY()));
        if (waypoint.hasHeadingConstraint()) {
          hashString.append(format.format(waypoint.getHeadingConstraint()));
        }

        if (waypoint.hasSamples()) {
          hashString.append(waypoint.getSamples());
        }

        switch (waypoint.getVelocityConstraintCase()) {
          case ZERO_VELOCITY -> {
            hashString.append(format.format(0));
          }
          case VEHICLE_VELOCITY -> {
            hashString.append(format.format(waypoint.getVehicleVelocity().getVx()));
            hashString.append(format.format(waypoint.getVehicleVelocity().getVy()));
            hashString.append(format.format(waypoint.getVehicleVelocity().getOmega()));
          }
          case VELOCITYCONSTRAINT_NOT_SET -> {}
        }
      }

      if (segment.hasMaxVelocity()) {
        hashString.append(format.format(segment.getMaxVelocity()));
      }
      if (segment.hasMaxOmega()) {
        hashString.append(format.format(segment.getMaxOmega()));
      }

      hashString.append(segment.getStraightLine());
    }

    return Hashing.sha256().hashString(hashString, StandardCharsets.UTF_8).toString();
  }
}
