// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/GenerateTrajectories.h"
#include "absl/hash/hash.h"
#include "absl/hash/internal/city.h"
#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "frc/Units/Units.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "google/protobuf/util/message_differencer.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/DriveTrajectories.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.grpc.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.grpc.pb.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/message_differencer.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <iomanip>
#include <iostream>
#include <sstream>

std::string
GenerateTrajectories::GetHashCode(const VehicleModel &model,
                                  const std::vector<PathSegment> &segments) {
  std::stringstream hashString;
  hashString.precision(6);
  hashString << std::fixed;

  hashString << model.mass();
  hashString << model.moi();
  hashString << model.vehicle_length();
  hashString << model.vehicle_width();
  hashString << model.wheel_radius();
  hashString << model.max_wheel_torque();
  hashString << model.max_wheel_omega();

  for (const auto &segment : segments) {
    for (const auto &waypoint : segment.waypoints()) {
      hashString << waypoint.x();
      hashString << waypoint.y();
      if (waypoint.has_heading_constraint()) {
        hashString << waypoint.heading_constraint();
      }

      if (waypoint.has_samples()) {
        hashString << waypoint.samples();
      }

      switch (waypoint.velocity_constraint_case()) {
      case Waypoint::VelocityConstraintCase::kZeroVelocity: {
        hashString << 0.0;
        break;
      }
      case Waypoint::VelocityConstraintCase::kVehicleVelocity: {
        hashString << waypoint.vehicle_velocity().vx();
        hashString << waypoint.vehicle_velocity().vy();
        hashString << waypoint.vehicle_velocity().omega();
        break;
      }
      case Waypoint::VelocityConstraintCase::VELOCITY_CONSTRAINT_NOT_SET: {
        break;
      }
      }
    }

    if (segment.has_max_velocity()) {
      hashString << segment.max_velocity();
    }
    if (segment.has_max_omega()) {
      hashString << segment.max_omega();
    }

    hashString << segment.straight_line();
  }

  return absl::StrCat(absl::Hash(hashString.str()));
}

int main(int argc, char **argv) {
  org::littletonrobotics::frc2025::Constants::DisableHAL();

  // Create vehicle model
  VehicleModel model;
  model.set_mass(67.0);
  model.set_moi(5.8);
  model.set_vehicle_length(org::littletonrobotics::frc2025::subsystems::drive::
                               DriveConstants::trackWidthX);
  model.set_vehicle_width(org::littletonrobotics::frc2025::subsystems::drive::
                              DriveConstants::trackWidthY);
  model.set_wheel_radius(frc::Units::InchesToMeters(2.0));
  model.set_max_wheel_torque(3.0);
  model.set_max_wheel_omega(org::littletonrobotics::frc2025::subsystems::drive::
                                DriveConstants::maxLinearSpeed /
                            frc::Units::InchesToMeters(2.0) * 0.75);

  // Connect to service
  auto channel = grpc::CreateChannel("127.0.0.1:56328",
                                     grpc::InsecureChannelCredentials());
  std::unique_ptr<VehicleTrajectoryService::Stub> service(
      VehicleTrajectoryService::NewStub(channel));

  std::set<std::string> completedPaths;
  while (true) {
    // Get all paths
    auto allPaths = org::littletonrobotics::frc2025::subsystems::drive::
        trajectory::DriveTrajectories::paths;
    bool hasAllPaths = true;
    for (const auto &supplier : org::littletonrobotics::frc2025::subsystems::
             drive::trajectory::DriveTrajectories::suppliedPaths) {
      auto suppliedPaths = supplier(completedPaths);
      if (suppliedPaths.empty() && !supplier) {
        hasAllPaths = false;
      } else {
        allPaths.insert(suppliedPaths.begin(), suppliedPaths.end());
      }
    }
    std::set<std::string> originalKeys;
    for (const auto &pair : allPaths) {
      originalKeys.insert(pair.first);
    }
    for (const auto &name : originalKeys) {
      if (completedPaths.count(name)) {
        allPaths.erase(name);
      } else {
        completedPaths.insert(name);
      }
    }
    if (!hasAllPaths && allPaths.empty()) {
      std::cerr << "Invalid dependency tree. Not all supplied trajectories are "
                   "available, but there are no trajectories to generate."
                << std::endl;
      return 1;
    }

    // Check hashcodes
    std::map<std::string, std::vector<PathSegment>> pathQueue;
    for (const auto &entry : allPaths) {
      std::string hashCode =
          GenerateTrajectories::GetHashCode(model, entry.second);
      std::filesystem::path pathFile("src/main/deploy/trajectories/" +
                                     entry.first + ".pathblob");
      if (std::filesystem::exists(pathFile)) {
        try {
          std::ifstream fileStream(pathFile, std::ios::binary);
          Trajectory trajectory;
          if (trajectory.ParseFromIstream(&fileStream)) {
            if (trajectory.hash_code() != hashCode) {
              pathQueue[entry.first] = entry.second;
            }
          } else {
            pathQueue[entry.first] = entry.second;
          }
        } catch (const std::exception &e) {
          std::cerr << "Error reading file: " << e.what() << std::endl;
          pathQueue[entry.first] = entry.second;
        }
      } else {
        pathQueue[entry.first] = entry.second;
      }
    }

    // Generate trajectories
    const char *generateEmptyFlag =
        std::getenv("GENERATE_EMPTY_DRIVE_TRAJECTORIES");
    bool generateEmpty =
        generateEmptyFlag != nullptr && std::strlen(generateEmptyFlag) > 0;
    for (const auto &entry : pathQueue) {
      Trajectory trajectory;
      std::cout << entry.first << " - Generating 💭";
      auto startTime = std::chrono::high_resolution_clock::now();
      if (generateEmpty) {
        trajectory.mutable_states()->Add();
      } else {
        // Use service for generation
        PathRequest request;
        *request.mutable_model() = model;
        for (const auto &segment : entry.second) {
          *request.add_segments() = segment;
        }

        TrajectoryResponse response;
        grpc::ClientContext context;
        grpc::Status status =
            service->GenerateTrajectory(&context, request, &response);
        if (!status.ok()) {
          std::cerr << "gRPC call failed: " << status.error_message()
                    << std::endl;
          return 1;
        }
        if (response.has_error() && !response.error().reason().empty()) {
          std::cerr << "Got error response for trajectory \"" << entry.first
                    << "\": " << response.error().reason() << std::endl;
          return 1;
        }
        trajectory = response.trajectory();
        trajectory.set_hash_code(
            GenerateTrajectories::GetHashCode(model, entry.second));
      }
      std::filesystem::path pathFile("src/main/deploy/trajectories/" +
                                     entry.first + ".pathblob");
      try {
        std::ofstream fileStream(pathFile, std::ios::binary);
        trajectory.SerializeToOstream(&fileStream);
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                            endTime - startTime)
                            .count();
        std::cout << "\r" << entry.first << " - Finished in "
                  << std::round(duration / 100.0) / 10.0 << " secs ✅"
                  << std::endl;
      } catch (const std::exception &e) {
        std::cout << "\r" << entry.first << " - FAILED ⛔" << std::endl;
        std::cerr << "Error writing file: " << e.what() << std::endl;
      }
    }

    // Exit if all trajectories ready
    if (hasAllPaths) {
      break;
    }
  }

  // Delete old trajectories
  try {
    for (const auto &entry :
         std::filesystem::directory_iterator("src/main/deploy/trajectories")) {
      std::string filename = entry.path().filename().string();
      if (filename.find(".pathblob") == std::string::npos)
        continue;
      std::vector<std::string> components;
      absl::StrSplit(filename, '.', std::back_inserter(components));
      if (components.size() == 2 && !completedPaths.count(components[0])) {
        std::filesystem::remove(entry.path());
        std::cout << components[0] << " - Deleted 💀" << std::endl;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error deleting files: " << e.what() << std::endl;
  }
  std::cout << "All trajectories up-to-date!" << std::endl;
  return 0;
}
