// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#pragma once

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "thirdparty/junction/LogDataReceiver.h"
#include "thirdparty/junction/LogTable.h"
#include "util/rlog/RLOGEncoder.h"

namespace rlog {

/** Sends log data over a socket connection using the RLOG format. */
class RLOGServer : public LogDataReceiver {
public:
  RLOGServer();
  RLOGServer(int port);

  void Start();
  void End();

  void PutTable(LogTable table) override;

private:
  std::vector<uint8_t> EncodeData(const std::vector<uint8_t> &data);

  class ServerThread {
  public:
    ServerThread(int port, RLOGServer *serverInstance);
    ~ServerThread();
    void Start();
    void Close();

  private:
    void Run();
    void RunBroadcast();
    void PrintDisconnectMessage(int socket, const std::string &reason);

    static const double heartbeatTimeoutSecs;

    int serverSocket;
    std::thread serverThread;
    std::thread broadcastThread;
    std::queue<std::vector<uint8_t>> broadcastQueue;
    std::vector<int> clientSockets;
    std::vector<double> lastHeartbeats;
    std::mutex socketsMutex;
    std::mutex broadcastQueueMutex;
    std::condition_variable broadcastQueueCv;
    std::atomic<bool> running;
    RLOGServer *serverInstance;
  };

  int port;
  std::unique_ptr<ServerThread> thread;
  RLOGEncoder encoder;

  static std::mutex encoderMutex;
  static std::mutex socketsMutex;
};

} // namespace rlog