//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <map>
#include <fstream>

#include <csignal>

std::condition_variable quit_condition;
std::mutex mtx;
std::mutex csv_mutex;
std::ofstream g_point_csv;
std::ofstream g_imu_csv;

namespace {

uint64_t ExtractTimestamp(const uint8_t timestamp[8]) {
  uint64_t value = 0;
  memcpy(&value, timestamp, sizeof(value));
  return value;
}

const char* DescribePointType(uint8_t data_type) {
  switch (data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      return "CartesianHigh";
    case kLivoxLidarCartesianCoordinateLowData:
      return "CartesianLow";
    case kLivoxLidarSphericalCoordinateData:
      return "Spherical";
    case kLivoxLidarImuData:
      return "IMU";
    default:
      return "Unknown";
  }
}

void DumpPointToCsv(uint32_t handle,
                    const LivoxLidarEthernetPacket* packet,
                    const char* point_type,
                    uint16_t index,
                    int value1,
                    int value2,
                    int value3,
                    uint8_t reflectivity,
                    uint8_t tag) {
  if (!g_point_csv.is_open()) {
    return;
  }
  const uint64_t timestamp = ExtractTimestamp(packet->timestamp);
  std::lock_guard<std::mutex> lock(csv_mutex);
  g_point_csv << timestamp << ',' << handle << ',' << point_type << ',' << packet->frame_cnt << ','
              << index << ',' << value1 << ',' << value2 << ',' << value3 << ','
              << static_cast<uint32_t>(reflectivity) << ',' << static_cast<uint32_t>(tag) << '\n';
}

void DumpImuToCsv(uint32_t handle,
                  const LivoxLidarEthernetPacket* packet,
                  uint16_t index,
                  const LivoxLidarImuRawPoint& imu) {
  if (!g_imu_csv.is_open()) {
    return;
  }
  const uint64_t timestamp = ExtractTimestamp(packet->timestamp);
  std::lock_guard<std::mutex> lock(csv_mutex);
  g_imu_csv << timestamp << ',' << handle << ',' << packet->frame_cnt << ',' << index << ','
            << imu.gyro_x << ',' << imu.gyro_y << ',' << imu.gyro_z << ','
            << imu.acc_x << ',' << imu.acc_y << ',' << imu.acc_z << '\n';
}

void CloseCsvFiles() {
  std::lock_guard<std::mutex> lock(csv_mutex);
  if (g_point_csv.is_open()) {
    g_point_csv.close();
  }
  if (g_imu_csv.is_open()) {
    g_imu_csv.close();
  }
}

void PrintPointCloudData(const uint32_t handle, const LivoxLidarEthernetPacket* packet) {
  const uint64_t timestamp = ExtractTimestamp(packet->timestamp);
  const char* point_type = DescribePointType(packet->data_type);
  printf("PointCloud handle:%u dots:%u data_type:%s frame:%u time:%llu\n",
      handle, packet->dot_num, point_type, packet->frame_cnt,
      static_cast<unsigned long long>(timestamp));

  if (packet->data_type == kLivoxLidarCartesianCoordinateHighData) {
    const auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(packet->data);
    for (uint16_t i = 0; i < packet->dot_num; ++i) {
      printf("  idx:%u xyz(mm): %d %d %d refl:%u tag:%u\n", i, pts[i].x, pts[i].y, pts[i].z,
          pts[i].reflectivity, pts[i].tag);
      DumpPointToCsv(handle, packet, point_type, i, pts[i].x, pts[i].y, pts[i].z,
          pts[i].reflectivity, pts[i].tag);
    }
  } else if (packet->data_type == kLivoxLidarCartesianCoordinateLowData) {
    const auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(packet->data);
    for (uint16_t i = 0; i < packet->dot_num; ++i) {
      printf("  idx:%u xyz(cm): %d %d %d refl:%u tag:%u\n", i, pts[i].x, pts[i].y, pts[i].z,
          pts[i].reflectivity, pts[i].tag);
      DumpPointToCsv(handle, packet, point_type, i, pts[i].x, pts[i].y, pts[i].z,
          pts[i].reflectivity, pts[i].tag);
    }
  } else if (packet->data_type == kLivoxLidarSphericalCoordinateData) {
    const auto* pts = reinterpret_cast<const LivoxLidarSpherPoint*>(packet->data);
    for (uint16_t i = 0; i < packet->dot_num; ++i) {
      printf("  idx:%u depth:%u theta:%u phi:%u refl:%u tag:%u\n", i, pts[i].depth, pts[i].theta,
          pts[i].phi, pts[i].reflectivity, pts[i].tag);
      DumpPointToCsv(handle, packet, point_type, i, pts[i].depth, pts[i].theta, pts[i].phi,
          pts[i].reflectivity, pts[i].tag);
    }
  } else {
    printf("  Unsupported point data type (%u), raw dump skipped.\n", packet->data_type);
  }
}

void PrintImuData(const uint32_t handle, const LivoxLidarEthernetPacket* packet) {
  const uint64_t timestamp = ExtractTimestamp(packet->timestamp);
  printf("IMU handle:%u samples:%u frame:%u time:%llu\n", handle, packet->dot_num,
      packet->frame_cnt, static_cast<unsigned long long>(timestamp));

  const auto* imu_points = reinterpret_cast<const LivoxLidarImuRawPoint*>(packet->data);
  for (uint16_t i = 0; i < packet->dot_num; ++i) {
    printf(
        "  idx:%u gyro[deg/s]:%.3f %.3f %.3f acc[m/s2]:%.3f %.3f %.3f\n", i, imu_points[i].gyro_x,
        imu_points[i].gyro_y, imu_points[i].gyro_z, imu_points[i].acc_x, imu_points[i].acc_y,
        imu_points[i].acc_z);
    DumpImuToCsv(handle, packet, i, imu_points[i]);
  }
}

}  // namespace

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  (void)dev_type;
  (void)client_data;
  PrintPointCloudData(handle, data);
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  (void)dev_type;
  (void)client_data;
  PrintImuData(handle, data);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Start logger failed, the status :%d\n", status);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    printf("Start logger failed, the response is nullptr.\n");
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response->ret_code != 0) {
    printf("Start logger failed, the response ret_Code:%d.\n", response->ret_code);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  printf("The lidar[%u] start logger succ.\n", handle);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
}

void Stop(int signal) {
  quit_condition.notify_all();
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  const char *point_cloud_path = std::getenv("POINT_CLOUD_CSV_PATH");
  const std::string default_point_cloud_path = "point_cloud.csv";
  if (point_cloud_path == nullptr) {
    point_cloud_path = default_point_cloud_path.c_str();
  }
  const char *imu_data_path = std::getenv("IMU_DATA_CSV_PATH");
  const std::string default_imu_data_path = "imu_data.csv";
  if (imu_data_path == nullptr) {
    imu_data_path = default_imu_data_path.c_str();
  }

  g_point_csv.open(point_cloud_path, std::ios::out | std::ios::trunc);
  g_imu_csv.open(imu_data_path, std::ios::out | std::ios::trunc);
  if (!g_point_csv.is_open() || !g_imu_csv.is_open()) {
    printf("Failed to open CSV output files.\n");
    CloseCsvFiles();
    return -1;
  }
  g_point_csv << "timestamp,handle,data_type,frame,index,value1,value2,value3,reflectivity,tag\n";
  g_imu_csv << "timestamp,handle,frame,index,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z\n";

  SaveLivoxLidarSdkLoggerFile();

  //DisableLivoxSdkConsoleLogger();

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    CloseCsvFiles();
    return -1;
  }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  // capture Ctrl + C signal.
  std::signal(SIGINT, Stop);

  std::unique_lock<std::mutex> lock(mtx);
  quit_condition.wait(lock);
  printf("Deivice Logger exit.\n");


  LivoxLidarSdkUninit();
  CloseCsvFiles();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}
