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

#include "util/rlog/RLOGServer.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace rlog {

std::mutex RLOGServer::encoderMutex;
std::mutex RLOGServer::socketsMutex;

RLOGServer::RLOGServer() : RLOGServer(5800) {}

RLOGServer::RLOGServer(int port) : port(port), thread(nullptr) {}

void RLOGServer::Start() {
  thread = std::make_unique<ServerThread>(port, this);
  thread->Start();
  std::cout << "[AdvantageKit] RLOG server started on port " << port
            << std::endl;
}

void RLOGServer::End() {
  if (thread) {
    thread->Close();
    thread.reset();
  }
}

void RLOGServer::PutTable(LogTable table) {
  if (thread) {
    std::vector<uint8_t> data;
    {
      std::lock_guard<std::mutex> lock(encoderMutex);
      encoder.EncodeTable(table, false);
      data = EncodeData(encoder.GetOutput());
    }
    thread->broadcastQueue.push(data);
    thread->broadcastQueueCv.notify_one();
  }
}

std::vector<uint8_t> RLOGServer::EncodeData(const std::vector<uint8_t> &data) {
  std::vector<uint8_t> lengthBytes(sizeof(int32_t));
  int32_t length = data.size();
  std::memcpy(lengthBytes.data(), &length, sizeof(int32_t));

  std::vector<uint8_t> fullData(lengthBytes.size() + data.size());
  std::memcpy(fullData.data(), lengthBytes.data(), lengthBytes.size());
  std::memcpy(fullData.data() + lengthBytes.size(), data.data(), data.size());
  return fullData;
}

const double RLOGServer::ServerThread::heartbeatTimeoutSecs = 3.0;

RLOGServer::ServerThread::ServerThread(int port, RLOGServer *serverInstance)
    : running(true), serverInstance(serverInstance) {
  serverSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (serverSocket < 0) {
    std::cerr << "Error opening socket" << std::endl;
    return;
  }

  sockaddr_in serverAddress;
  std::memset(&serverAddress, 0, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = INADDR_ANY;
  serverAddress.sin_port = htons(port);

  if (bind(serverSocket, (struct sockaddr *)&serverAddress,
           sizeof(serverAddress)) < 0) {
    std::cerr << "Error binding socket" << std::endl;
    close(serverSocket);
    serverSocket = -1;
    return;
  }
}

RLOGServer::ServerThread::~ServerThread() { Close(); }

void RLOGServer::ServerThread::Start() {
  if (serverSocket < 0)
    return;

  serverThread = std::thread(&RLOGServer::ServerThread::Run, this);
  broadcastThread = std::thread(&RLOGServer::ServerThread::RunBroadcast, this);
}

void RLOGServer::ServerThread::Close() {
  running = false;
  if (serverSocket >= 0) {
    close(serverSocket);
    serverSocket = -1;
  }
  if (serverThread.joinable())
    serverThread.join();
  if (broadcastThread.joinable())
    broadcastThread.join();
  broadcastQueueCv.notify_all();
  {
    std::lock_guard<std::mutex> lock(socketsMutex);
    for (int socket : clientSockets) {
      close(socket);
    }
  }
}

void RLOGServer::ServerThread::Run() {
  if (serverSocket < 0)
    return;

  listen(serverSocket, 5);
  while (running) {
    sockaddr_in clientAddress;
    socklen_t clientLength = sizeof(clientAddress);
    int clientSocket =
        accept(serverSocket, (struct sockaddr *)&clientAddress, &clientLength);
    if (clientSocket < 0) {
      if (running) {
        std::cerr << "Error accepting connection" << std::endl;
      }
      break;
    }

    std::vector<uint8_t> data;
    {
      std::lock_guard<std::mutex> lock(serverInstance->encoderMutex);
      data =
          serverInstance->EncodeData(serverInstance->encoder.GetNewcomerData());
    }

    if (send(clientSocket, data.data(), data.size(), 0) < 0) {
      close(clientSocket);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(socketsMutex);
      clientSockets.push_back(clientSocket);
      lastHeartbeats.push_back(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count() /
          1000.0);
    }

    char clientIP[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(clientAddress.sin_addr), clientIP, INET_ADDRSTRLEN);
    std::cout << "[AdvantageKit] Connected to RLOG client - " << clientIP
              << std::endl;
  }
}

void RLOGServer::ServerThread::RunBroadcast() {
  while (running) {
    std::unique_lock<std::mutex> lock(broadcastQueueMutex);
    broadcastQueueCv.wait_for(lock, std::chrono::milliseconds(20), [this] {
      return !broadcastQueue.empty() || !running;
    });
    if (!running && broadcastQueue.empty())
      break;

    std::queue<std::vector<uint8_t>> tempQueue;
    std::swap(tempQueue, broadcastQueue);
    lock.unlock();

    std::vector<std::vector<uint8_t>> broadcastData;
    while (!tempQueue.empty()) {
      broadcastData.push_back(tempQueue.front());
      tempQueue.pop();
    }

    {
      std::lock_guard<std::mutex> lock(socketsMutex);
      for (size_t i = 0; i < clientSockets.size(); i++) {
        int clientSocket = clientSockets[i];
        if (clientSocket < 0)
          continue;

        char buffer[4];
        ssize_t bytesRead =
            recv(clientSocket, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT);
        if (bytesRead > 0) {
          recv(clientSocket, buffer, bytesRead, 0);
          lastHeartbeats[i] =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count() /
              1000.0;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                        .count() /
                    1000.0 -
                lastHeartbeats[i] >
            heartbeatTimeoutSecs) {
          close(clientSocket);
          PrintDisconnectMessage(clientSocket, "timeout");
          clientSockets[i] = -1;
          continue;
        }

        send(clientSocket, buffer, sizeof(buffer), 0);

        for (const auto &data : broadcastData) {
          if (send(clientSocket, data.data(), data.size(), 0) < 0) {
            close(clientSocket);
            PrintDisconnectMessage(clientSocket, "IOException");
            clientSockets[i] = -1;
            break;
          }
        }
      }
    }
  }

  void RLOGServer::ServerThread::PrintDisconnectMessage(
      int socket, const std::string &reason) {
    sockaddr_in clientAddress;
    socklen_t clientLength = sizeof(clientAddress);
    if (getpeername(socket, (struct sockaddr *)&clientAddress, &clientLength) ==
        0) {
      char clientIP[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(clientAddress.sin_addr), clientIP, INET_ADDRSTRLEN);
      std::cout << "Disconnected from RLOG client (" << reason << ") - "
                << clientIP << std::endl;
    } else {
      std::cout << "Disconnected from RLOG client (" << reason << ")"
                << std::endl;
    }
  }

} // namespace rlog
