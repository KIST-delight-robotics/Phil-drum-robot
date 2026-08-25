#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "nlohmann/json.hpp"

// realsense 헤더
#include <librealsense2/rs.hpp>
// Point Cloud Library 헤더
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include "common/app_context.hpp"
#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
#include "hardware/robot.hpp"
#include "trajectory/trajectory_generator.hpp"

// =============================================================
// RealSense + PCL 기반 드럼 스캐너 (레거시 DrumRobot/DrumDetector 이식)
//
// - MotionPlanner 스레드에서 동기적으로 생성/호출된다 (별도 스레드 없음).
//   스캔 동안 MotionPlanner::run() 루프가 정지하므로, 허리 이동은
//   trajectory_generator.generate_trajectory()를 직접 호출해 control_queue에
//   적재하고, 큐 소진(= send_loop가 전부 전송)으로 이동 완료를 감지한다.
// - 카메라는 스캔 동안에만 열린다 (생성 시 시작, 소멸 시 정지).
// - 예외를 밖으로 던지지 않는다: run_scan()은 성공/실패만 반환.
// - 성공 시 config/drum_coordinate.json을 백업 후 갱신한다.
//   (핫 리로드는 MotionPlanner가 수행)
// =============================================================
class DrumDetector {
public:
    DrumDetector(AppContext &ctxRef, Robot &robotRef,
                 TrajectoryGenerator &trajectoryGeneratorRef, ControlQueue &controlQueueRef);
    ~DrumDetector();    // 카메라 정지

    bool run_scan();    // 전체 스캔. true = drum_coordinate.json 갱신 완료

private:
    AppContext          &ctx;
    Robot               &robot;
    TrajectoryGenerator &trajectory_generator;
    ControlQueue        &control_queue;

    // ===== 상수 (레거시처럼 코드 상수) =====
    static constexpr int    WAIST_JOINT        = 0;                      // motors.json의 허리 관절 id
    static constexpr int    NUM_CIRCLES        = 8;                      // 검출해야 하는 드럼 원 개수 (indexCircles 전제)
    static constexpr double WAIST_MOVE_TIME    = 3.0;                    // 허리 이동 시간 [s]
    static constexpr double WAIST_WAIT_TIMEOUT = WAIST_MOVE_TIME + 10.0; // control_queue 소진 대기 타임아웃 [s]
    static constexpr double SETTLE_TIME        = 1.0;                    // 이동 후 진동 정착 대기 [s]
    static constexpr double LEGACY_TO_SERVER_Z = 0.955;                  // 서버 z = 레거시 z - 0.955 (좌표 파일 대조로 검증)

    // 스캔 허리 각도 목록 [deg] — 레거시처럼 코드에 하드코딩
    // const std::vector<float> scan_angles_deg = {0.0f};
    const std::vector<float> scan_angles_deg = {30.0f, 20.0f, 10.0f, 0.0f, -10.0f, -20.0f, -30.0f};

    bool visualize = false;         // config/drum_scan.json
    bool camera_started = false;

    // ===== RealSense 멤버 (레거시 동일) =====
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    rs2::threshold_filter thresh_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    rs2::disparity_transform depth_to_disparity;  // 생성자에서 true로 초기화
    rs2::disparity_transform disparity_to_depth;  // 생성자에서 false로 초기화

    // ===== 스캔 단계 =====
    void load_scan_config();
    bool init_camera();
    bool move_waist_and_wait(double target_rad, const std::vector<double> &base_q);
    pcl::PointCloud<pcl::PointXYZ>::Ptr capture_cloud();
    bool aborted() const { return !ctx.running.load(); }

    // ===== 레거시 파이프라인 이식 =====
    pcl::PointCloud<pcl::PointXYZ>::Ptr convert_rs2_points_to_pcl(const rs2::points &points);
    rs2::depth_frame apply_filters(const rs2::frameset &frames);
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    std::vector<pcl::PointIndices> extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_to_robot_frame(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float waist_angle_rad);
    Eigen::Matrix4f make_cam2robot_matrix(float waist_angle_rad);
    void detect_circles(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
                        std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds);
    std::vector<int> index_circles(std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs,
                                   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds);  // 원 8개 전제
    std::vector<Eigen::VectorXd> select_candidates_for_circle(const pcl::ModelCoefficients::Ptr &coeffs, char DB);
    std::vector<std::vector<Eigen::VectorXd>> select_candidates(const std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs);
    void visualize_drums(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds,
                         const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates = {});

    // ===== 결과 저장 =====
    bool write_results(const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates, const std::string &ts);
    void dump_cloud_csv(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void dump_candidates_csv(const std::string &path, const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates);
    static std::string timestamp_suffix();   // MMDD_HHMM
};
