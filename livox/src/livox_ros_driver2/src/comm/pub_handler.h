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

#ifndef LIVOX_DRIVER_PUB_HANDLER_H_
#define LIVOX_DRIVER_PUB_HANDLER_H_

#include <atomic>
#include <cstring>
#include <condition_variable> // std::condition_variable
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>              // std::mutex
#include <thread>

#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/ptp_time.h"
#include "run_logger.hpp"

namespace livox_ros {

class LidarPubHandler {
 public:
  LidarPubHandler();
  ~ LidarPubHandler() {}

  void PointCloudProcess(RawPacket& pkt);
  void SetLidarsExtParam(LidarExtParameter param);
  void GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds);

  uint64_t GetRecentTimeStamp();
  uint32_t GetLidarPointCloudsSize();
  uint64_t GetLidarBaseTime();
  void ClearPointClouds();

 private:
  void LivoxLidarPointCloudProcess(RawPacket & pkt);
  void ProcessCartesianHighPoint(RawPacket & pkt);
  void ProcessCartesianLowPoint(RawPacket & pkt);
  void ProcessSphericalPoint(RawPacket & pkt);
  void ProcessDoubleEchoPoint(RawPacket& pkt);
  std::vector<PointXyzlt> points_clouds_;
  ExtParameterDetailed extrinsic_ = {
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 1},
      {0, 0, 1}
    }
  };
  std::mutex mutex_;
  std::atomic_bool is_set_extrinsic_params_;
};
  
class PubHandler {
 public:
  using PointCloudsCallback = std::function<void(PointFrame*, void *)>;
  using ImuDataCallback = std::function<void(ImuData*, void*)>;

  PubHandler();

  ~ PubHandler() { Uninit(); }

  void Uninit();
  void RequestExit();
  void Init();
  void SetPointCloudConfig(const double publish_freq);
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data);
  void AddLidarsExtParam(LidarExtParameter& extrinsic_params);
  void ClearAllLidarsExtrinsicParams();
  void SetImuDataCallback(ImuDataCallback cb, void* client_data);
  void SetTimestampConfig(const TimestampConfig& config);
  void SetExpectedLidars(const std::vector<uint32_t>& handles);
  std::string HealthSummary(uint32_t handle);
  void SetHealthEvents(std::function<void(const std::string&, const std::string&)> observer) {
    std::lock_guard<std::mutex> lock(health_events_mutex_);
    health_events_ = std::move(observer);
  }
  bool IsPtpLocked();
  bool IsCurrentGeneration(uint32_t handle, SensorStream stream, uint64_t generation);
  bool PublishIfCurrent(uint32_t handle, SensorStream stream, uint64_t generation,
                        uint64_t timestamp,
                        const std::function<void()>& publish);

 private:
  void RecordArrival(uint32_t handle, SensorStream stream,
                     const LivoxLidarEthernetPacket& data, uint64_t arrival);
  void Diagnostic(const std::string& event, const std::string& payload);
  struct FlowDiagnostic {
    uint64_t arrival = 0, wire_stamp = 0, summary = 0, packets = 0;
    uint64_t rejects = 0, last_reject_log = 0, max_gap = 0, sequence_changes = 0;
    uint16_t sequence = 0;
    uint64_t accepted_arrival = 0;
    TimestampError last_error = TimestampError::kNone;
    std::map<TimestampError, uint64_t> reasons;
  };
  std::unique_ptr<odom_logging::RunLogger> run_logger_;
  std::mutex diagnostic_mutex_;
  std::map<std::pair<uint32_t, SensorStream>, FlowDiagnostic> diagnostics_;
  std::function<void(const std::string&, const std::string&)> health_events_;
  std::mutex health_events_mutex_;
  uint64_t health_timeout_ns_ = 500000000ULL;
  //thread to process raw data
  void RawDataProcess();
  std::atomic<bool> is_quit_{false};
  std::shared_ptr<std::thread> point_process_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;

  //publish callback
  void CheckTimer(uint32_t id, const RawPacket& packet);
  void PublishPointCloud();
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data);
  
  static bool GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id);
  TimestampResult ValidateTimestamp(uint32_t handle, SensorStream stream,
                                    const LivoxLidarEthernetPacket& data);
  void RejectPacket(uint32_t handle, SensorStream stream);

  PointCloudsCallback points_callback_;
  void* pub_client_data_ = nullptr;

  ImuDataCallback imu_callback_;
  void* imu_client_data_ = nullptr;

  PointFrame frame_;

  std::deque<RawPacket> raw_packet_queue_;

  //pub config
  uint64_t publish_interval_ = 100000000; //100 ms

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_;
  std::map<uint32_t, std::vector<PointXyzlt>> points_;
  std::map<uint32_t, LidarExtParameter> lidar_extrinsics_;
  std::mutex timestamp_mutex_;
  TimestampGuard timestamp_guard_;
  std::map<std::pair<uint32_t, SensorStream>, uint64_t> timestamp_warning_times_;
  std::map<std::pair<uint32_t, SensorStream>, std::pair<uint64_t, uint64_t>> stale_publish_counts_;
  std::map<uint32_t, uint64_t> processed_generations_;
  std::map<uint32_t, FrameBoundary> frame_boundaries_;
  uint16_t lidar_listen_id_ = 0;
};

PubHandler &pub_handler();

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_PUB_HANDLER_H_
