// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

#include <condition_variable>
#include <future>
#include <mutex>
#include <numeric>
#include <vector>

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"

#include "frc/RobotController.h"
#include "frc/Threads.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/Drive.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"

bool PhoenixOdometryThread::isCANFD = ctre::phoenix6::CANBus("*").IsNetworkFD();
PhoenixOdometryThread PhoenixOdometryThread::instance;

PhoenixOdometryThread &PhoenixOdometryThread::GetInstance() { return instance; }

PhoenixOdometryThread::PhoenixOdometryThread() {
  frc::Threads::SetName("PhoenixOdometryThread");
  frc::Threads::SetDaemon(true);
}

std::queue<double> *PhoenixOdometryThread::RegisterSignal(
    ctre::phoenix6::signals::StatusSignal<units::radian_t> &signal) {
  auto queue = new std::queue<double>();
  signalsMutex.lock();
  Drive::odometryLock.lock();
  try {
    phoenixSignals.push_back(&signal);
    phoenixQueues.push_back(queue);
  } finally {
    signalsMutex.unlock();
    Drive::odometryLock.unlock();
  }
  return queue;
}

std::queue<double> *
PhoenixOdometryThread::RegisterSignal(std::function<double()> signal) {
  auto queue = new std::queue<double>();
  signalsMutex.lock();
  Drive::odometryLock.lock();
  try {
    genericSignals.push_back(signal);
    genericQueues.push_back(queue);
  } finally {
    signalsMutex.unlock();
    Drive::odometryLock.unlock();
  }
  return queue;
}

std::queue<double> *PhoenixOdometryThread::MakeTimestampQueue() {
  auto queue = new std::queue<double>();
  Drive::odometryLock.lock();
  try {
    timestampQueues.push_back(queue);
  } finally {
    Drive::odometryLock.unlock();
  }
  return queue;
}

void PhoenixOdometryThread::Run() {
  frc::Threads::SetCurrentThreadPriority(true, 99);
  while (true) {
    // Wait for updates from all signals
    signalsMutex.lock();
    try {
      if (isCANFD && !phoenixSignals.empty()) {
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
            2.0 / DriveConstants::odometryFrequency, phoenixSignals);
      } else {
        // "waitForAll" does not support blocking on multiple signals with a bus
        // that is not CAN FD, regardless of Pro licensing. No reasoning for
        // this behavior is provided by the documentation.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(1000.0 / DriveConstants::odometryFrequency)));
        if (!phoenixSignals.empty())
          ctre::phoenix6::BaseStatusSignal::RefreshAll(phoenixSignals);
      }
    } catch (const std::exception &e) {
      std::cerr << "Exception in PhoenixOdometryThread: " << e.what()
                << std::endl;
    } finally {
      signalsMutex.unlock();
    }

    // Save new data to queues
    Drive::odometryLock.lock();
    try {
      // Sample timestamp is current FPGA time minus average CAN latency
      //     Default timestamps from Phoenix are NOT compatible with
      //     FPGA timestamps, this solution is imperfect but close
      double timestamp = frc::RobotController::GetFPGATime() / 1e6;
      double totalLatency = 0.0;
      for (auto signal : phoenixSignals) {
        totalLatency += signal->GetTimestamp().GetLatency().value();
      }
      if (!phoenixSignals.empty()) {
        timestamp -= totalLatency / phoenixSignals.size();
      }

      // Add new samples to queues
      for (size_t i = 0; i < phoenixSignals.size(); i++) {
        phoenixQueues[i]->push(phoenixSignals[i]->GetValueAsDouble().value());
      }
      for (size_t i = 0; i < genericSignals.size(); i++) {
        genericQueues[i]->push(genericSignals[i]());
      }
      for (size_t i = 0; i < timestampQueues.size(); i++) {
        timestampQueues[i]->push(timestamp);
      }
    } finally {
      Drive::odometryLock.unlock();
    }
  }
}