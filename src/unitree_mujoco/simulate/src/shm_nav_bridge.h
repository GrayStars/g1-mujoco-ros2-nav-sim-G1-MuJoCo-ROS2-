#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include <string>

#include <mujoco/mujoco.h>

static constexpr const char *kUnitreeMujocoNavShmName = "/unitree_mujoco_nav";
static constexpr uint32_t kUnitreeMujocoNavMagic = 0x564e4a4d;  // MJNV
static constexpr uint32_t kUnitreeMujocoNavVersion = 3;
static constexpr int kUnitreeMujocoNavMaxRanges = 360;
static constexpr int kUnitreeMujocoNavMaxPoints = 24000;

#pragma pack(push, 1)
struct UnitreeMujocoNavShmData
{
  uint32_t magic;
  uint32_t version;
  uint32_t num_ranges;
  uint32_t num_points;
  uint64_t seq;
  double sim_time;
  double pose[7];  // x y z qw qx qy qz
  double qvel[6];  // world linear xyz, angular xyz
  double livox_xyz[3];
  double livox_rpy[3];
  double imu_quat[4];  // qw qx qy qz
  double imu_gyro[3];
  double imu_acc[3];
  float ranges[kUnitreeMujocoNavMaxRanges];
  float points[kUnitreeMujocoNavMaxPoints * 3];  // xyz in livox_frame
};
#pragma pack(pop)

class ShmNavBridge
{
public:
  ShmNavBridge();
  ~ShmNavBridge();

  void Update(const mjModel *model, const mjData *data);

private:
  void OpenSharedMemory();
  void CloseSharedMemory();
  void UpdateRanges(const mjModel *model, const mjData *data);
  void UpdateMid360Cloud(const mjModel *model, const mjData *data, double base_yaw);
  bool LoadMid360Pattern();

  int shm_fd_ = -1;
  UnitreeMujocoNavShmData *shm_ = nullptr;
  int base_body_id_ = -1;
  int livox_site_id_ = -1;
  int lidar_body_exclude_id_ = -1;
  int imu_quat_adr_ = -1;
  int imu_gyro_adr_ = -1;
  int imu_acc_adr_ = -1;
  uint64_t seq_ = 0;
  size_t mid360_start_index_ = 0;

  std::chrono::steady_clock::time_point last_scan_update_;
  std::chrono::steady_clock::time_point last_cloud_update_;

  double scan_hz_ = 10.0;
  double cloud_hz_ = 10.0;
  int num_lidar_rays_ = 180;
  double range_min_ = 0.10;
  double range_max_ = 8.0;
  double cloud_range_max_ = 30.0;
  double livox_xyz_[3] = {0.0, 0.0, 0.95};
  double livox_rpy_[3] = {0.0, -2.3 * 3.14159265358979323846 / 180.0, 0.0};
  std::vector<float> mid360_theta_;
  std::vector<float> mid360_phi_;
};
