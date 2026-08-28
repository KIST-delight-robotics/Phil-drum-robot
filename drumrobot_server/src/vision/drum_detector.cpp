#include "vision/drum_detector.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <filesystem>

DrumDetector::DrumDetector(AppContext &ctxRef, Robot &robotRef,
                           TrajectoryGenerator &trajectoryGeneratorRef, ControlQueue &controlQueueRef)
    : ctx(ctxRef), robot(robotRef),
      trajectory_generator(trajectoryGeneratorRef), control_queue(controlQueueRef),
      align_to_color(RS2_STREAM_COLOR),
      depth_to_disparity(true),
      disparity_to_depth(false) {
}

DrumDetector::~DrumDetector() {
    if (camera_started) {
        try {
            pipe.stop();
        } catch (const std::exception &e) {
            std::cerr << "[DrumDetector] 카메라 정지 실패: " << e.what() << "\n";
        }
    }
}

// =============================================================
// 전체 스캔 흐름
// =============================================================
bool DrumDetector::run_scan() {
    try {
        // 스캔 전 자세 스냅샷 (허리 이동의 기준이자 종료 시 복귀 목표)
        std::vector<double> base_q;
        {
            std::lock_guard<std::mutex> lock(ctx.last_q_mutex);
            base_q = ctx.last_q_target_snapshot;
        }
        if (base_q.size() != static_cast<size_t>(ROBOT::NUM_JOINT)) {
            std::cerr << "[DrumDetector] last_q_target_snapshot 크기 이상 (" << base_q.size() << ") — 스캔 중단\n";
            return false;
        }

        if (!init_camera()) {
            return false;   // 허리를 아직 움직이지 않았으므로 복원 불필요
        }

        // ===== 각도별 캡처 =====
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds;
        bool capture_ok = true;
        for (float deg : scan_angles_deg) {
            if (aborted()) { capture_ok = false; break; }

            std::cout << "[DrumDetector] " << deg << "도 위치로 허리 이동...\n";
            if (!move_waist_and_wait(deg * M_PI / 180.0, base_q)) { capture_ok = false; break; }

            // 명령값이 아닌 실측 허리 각도로 변환 (레거시 getMotorPos와 동일 역할)
            // 허리 모터 미연결(무하드웨어 테스트) 시에는 명령각으로 대체
            float actual_rad;
            auto waist_it = robot.motors.find(WAIST_JOINT);
            if (waist_it != robot.motors.end()) {
                actual_rad = static_cast<float>(waist_it->second->current_joint_angle);
            } else {
                actual_rad = deg * static_cast<float>(M_PI) / 180.0f;
                std::cout << "[DrumDetector] 허리 모터 미연결 — 명령각 " << deg << "도로 변환\n";
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = capture_cloud();
            if (!cloud || cloud->empty()) { capture_ok = false; break; }

            cloud = remove_outliers(cloud);
            cloud = transform_to_robot_frame(cloud, actual_rad);
            accumulated_clouds.push_back(cloud);
        }

        // ===== 무거운 처리 전에 허리 원위치 (성패 무관, 로봇이 먼저 원위치) =====
        if (!aborted()) {
            std::cout << "[DrumDetector] 허리 원위치 복귀...\n";
            move_waist_and_wait(base_q[WAIST_JOINT], base_q);
        }
        if (!capture_ok) {
            std::cerr << "[DrumDetector] 캡처 실패 — 스캔 중단\n";
            return false;
        }

        visualize_drums(accumulated_clouds);

        // 병합 (레거시 use_registration=false 경로: 단순 합산)
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &c : accumulated_clouds) {
            *full_cloud += *c;
        }

        const std::string ts = timestamp_suffix();
        std::filesystem::create_directories("drumrobot_server/data/scan");
        pcl::PointCloud<pcl::PointXYZ>::Ptr indiv_sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &c : accumulated_clouds) {
            *indiv_sor_cloud += *c;
        }
        dump_cloud_csv("drumrobot_server/data/scan/scan_" + ts + "_cloud_no_sor.csv", indiv_sor_cloud);
        indiv_sor_cloud = down_sampling(indiv_sor_cloud);
        dump_cloud_csv("drumrobot_server/data/scan/scan_" + ts + "_cloud_no_sor_no_ds.csv", indiv_sor_cloud);

        if (aborted()) return false;

        full_cloud = remove_outliers(full_cloud);
        full_cloud = down_sampling(full_cloud);
        if (full_cloud->empty()) {
            std::cerr << "[DrumDetector] 필터링 후 점군이 비어 있음 — 스캔 중단\n";
            return false;
        }

        dump_cloud_csv("drumrobot_server/data/scan/scan_" + ts + "_cloud.csv", full_cloud);
        visualize_drums({full_cloud});

        // ===== 클러스터 → 원 검출 =====
        std::vector<pcl::PointIndices> cluster_indices = extract_clusters(full_cloud);

        std::vector<pcl::ModelCoefficients::Ptr> drum_coeffs;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> drum_clouds;

        int iter = 0;
        for (const auto &indices : cluster_indices) {
            if (aborted()) return false;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*full_cloud, indices, *cluster);
            iter++;
            std::cout << "[DrumDetector] " << iter << "번째 클러스터\n";
            detect_circles(cluster, drum_coeffs, drum_clouds);
        }

        // index_circles는 원 8개를 전제로 작성됨 (그 외 개수는 범위 밖 접근) — 개수 가드
        if (drum_coeffs.size() != static_cast<size_t>(NUM_CIRCLES)) {
            std::cerr << "[DrumDetector] 검출된 원이 " << NUM_CIRCLES << "개가 아님 (n="
                      << drum_coeffs.size() << ") — 스캔 중단\n";
            return false;
        }

        index_circles(drum_coeffs, drum_clouds);
        std::vector<std::vector<Eigen::VectorXd>> drum_candidates = select_candidates(drum_coeffs);

        visualize_drums(drum_clouds, drum_candidates);

        // return write_results(drum_candidates, ts);
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] 예외 발생: " << e.what() << " — 스캔 중단\n";
        return false;
    } catch (...) {
        std::cerr << "[DrumDetector] 알 수 없는 예외 — 스캔 중단\n";
        return false;
    }
}

// =============================================================
// 스캔 단계
// =============================================================
bool DrumDetector::init_camera() {
    try {
        int width = 848; int height = 480; int fps = 30;
        // D455 권장 해상도(848x480). 해상도가 높을수록 근거리 뎁스 품질에 유리하다.
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

        rs2::pipeline_profile profile = pipe.start(cfg);
        camera_started = true;

        rs2::device dev = profile.get_device();
        rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
            depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
            std::cout << "[DrumDetector] High Density 프리셋 적용\n";
        }
        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, 200.0f);
            std::cout << "[DrumDetector] 레이저 파워 200 설정\n";
        }

        // 드럼이 0.4~0.6m 사이만 남긴다. (단위: 미터)
        thresh_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.4f);
        thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);

        // Spatial Filter: 평면(드럼 헤드)을 펴주고 엣지를 보존
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
        spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3.0f);

        // Temporal Filter: 프레임 누적을 통한 시간적 노이즈(깜빡임) 제거
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3.0f);

        return true;
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] 카메라 초기화 실패: " << e.what() << "\n";
        return false;
    }
}

bool DrumDetector::move_waist_and_wait(double target_rad, const std::vector<double> &base_q) {
    // (추가) 허리 모터 미연결(무하드웨어 테스트) 시: send_loop가 control_queue를
    // 소비하지 않아 큐가 영원히 비지 않고 대기 타임아웃이 난다.
    // 캡처 루프의 미연결 판정과 동일하게 motors 맵 부재로 판단해 이동을 생략한다.
    if (robot.motors.find(WAIST_JOINT) == robot.motors.end()) {
        std::cout << "[DrumDetector] 허리 모터 미연결 — 허리 이동 생략\n";
        return !aborted();
    }

    MotionPrimitive m;
    m.type    = MotionType::TRANSLATE;
    m.space   = TrajectorySpace::JOINT;
    m.profile = TrajectoryProfile::COSINE;
    m.q_target = base_q;
    m.q_target[WAIST_JOINT] = target_rad;
    m.t_total = WAIST_MOVE_TIME;

    // MotionPlanner 스레드에서 동기 실행 중이므로 직접 궤적 생성이 안전하다.
    // 전체 궤적이 control_queue에 즉시 적재되고 send_loop가 병행 소비한다.
    trajectory_generator.generate_trajectory(m);

    // 스캔 동안 run() 루프가 정지해 control_queue를 채우는 곳이 없으므로
    // 큐 소진 = 이동 완료가 정확히 성립한다. (직전 idle 홀드 잔량도 함께 소진됨)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(WAIST_WAIT_TIMEOUT);
    while (!control_queue.empty()) {
        if (aborted()) return false;
        if (std::chrono::steady_clock::now() > deadline) {
            std::cerr << "[DrumDetector] 허리 이동 대기 타임아웃\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(SETTLE_TIME));   // 진동 정착

    return !aborted();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::capture_cloud() {
    try {
        const int warmup_frames = 30;   // temporal filter 수렴용 워밍업
        for (int i = 0; i < warmup_frames; i++) {
            if (aborted()) return nullptr;
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);
            apply_filters(frames);
        }
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);
        rs2::depth_frame final_frame = apply_filters(frames);

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(final_frame);
        return convert_rs2_points_to_pcl(points);
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] 프레임 캡처 실패: " << e.what() << "\n";
        return nullptr;
    }
}

// =============================================================
// 레거시 파이프라인 이식 (DrumRobot/src/DrumDetector.cpp)
// =============================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::convert_rs2_points_to_pcl(const rs2::points &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const size_t n = points.size();
    cloud->points.reserve(n);
    auto ptr = points.get_vertices();
    for (size_t i = 0; i < n; ++i) {
        const float x = ptr[i].x, y = ptr[i].y, z = ptr[i].z;
        // 무효 픽셀(깊이 0) 및 NaN 제거 — RANSAC 동일점 클러스터 방지
        if (z <= 0.0f || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        cloud->points.emplace_back(x, y, z);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

rs2::depth_frame DrumDetector::apply_filters(const rs2::frameset &frames) {
    rs2::frame filtered = frames.get_depth_frame();

    filtered = thresh_filter.process(filtered);
    filtered = depth_to_disparity.process(filtered);   // Depth -> Disparity 변환 (필수)
    filtered = spat_filter.process(filtered);          // 공간 필터
    filtered = temp_filter.process(filtered);          // 시간 필터
    filtered = disparity_to_depth.process(filtered);   // Disparity -> Depth 복구 (필수)

    return filtered.as<rs2::depth_frame>();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pointcloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(3.0);    // 1 agressive, 2~3해보기
    sor.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*filtered);

    return filtered;
}

std::vector<pcl::PointIndices> DrumDetector::extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud) {
    std::vector<pcl::PointIndices> cluster_indices;

    if (!pointcloud || pointcloud->empty()) {
        return cluster_indices;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.008);
    ec.setMinClusterSize(300);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pointcloud);
    ec.extract(cluster_indices);

    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::transform_to_robot_frame(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float waist_angle_rad) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f T_cam2robot = make_cam2robot_matrix(waist_angle_rad);
    pcl::transformPointCloud(*pointcloud, *transformed, T_cam2robot);

    return transformed;
}

Eigen::Matrix4f DrumDetector::make_cam2robot_matrix(float waist_angle_rad) {
    // 카메라로 얻은 점군을 (레거시) 로봇 프레임으로 변환하기 위한 동차 변환 행렬.
    // rot은 지면으로부터 카메라 설치 높이 및 허리 회전 각도에 따른 동적 좌표계.
    // 주의: 결과는 레거시 좌표계 기준이다. 서버 좌표계로는 출력 시 z에서
    //       LEGACY_TO_SERVER_Z(0.955)를 빼서 변환한다 (외인성 캘리브레이션 보존).
    Eigen::Matrix4f T_cam2rot, T_rot2robot, T_cam2robot;

    T_cam2rot << -0.01219f,  0.999661f, -0.02301f,  0.004527f,
                  0.767679f, 0.024104f,  0.640381f, 0.15327f,
                  0.640719f, -0.00986f,  -0.76771f, -0.08879f,
                  0.0f,      0.0f,       0.0f,      1.0f;

    T_rot2robot << cos(waist_angle_rad), -sin(waist_angle_rad), 0, 0,
                   sin(waist_angle_rad), cos(waist_angle_rad), 0, 0,
                   0, 0, 1, 1.129,
                   0, 0, 0, 1;

    T_cam2robot = T_rot2robot * T_cam2rot;

    return T_cam2robot;
}

void DrumDetector::detect_circles(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
                                  std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs,
                                  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds) {
    std::vector<Eigen::Vector4f> found_centers;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setMaxIterations(5000);
    seg.setRadiusLimits(0.100, 0.205); // 드럼 반경 제약

    for (int pass = 0; pass < 2; ++pass) {
        if (!pointcloud || pointcloud->empty()) {
            std::cout << "[DrumDetector] detect_circles: 점군이 비어 있음\n";
            return;
        }

        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(pointcloud);
        seg.segment(*inliers, *coeffs);

        if (inliers->indices.size() <= 200 || coeffs->values.size() < 7)
            continue;

        std::cout << "[DrumDetector] detect_circles: r = " << coeffs->values[3] << "\n";

        Eigen::Vector4f center(coeffs->values[0], coeffs->values[1], coeffs->values[2], 0.0f);
        bool far_enough = true;

        for (const auto &prev : found_centers) {
            float dist = (center - prev).head<3>().norm();
            if (dist < 0.15f) {
                far_enough = false;
                break;
            }
        }

        if (!far_enough) {
            // 레거시 동작 유지: 근접 원이 나오면 해당 클러스터는 원 1개로 마감
            std::cout << "[DrumDetector] detect_circles: 클러스터에서 하나의 원을 검출\n";
            return;
        }
        found_centers.push_back(center);
        drum_coeffs.push_back(coeffs);

        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pointcloud, *inliers, *inlier_cloud);

        drum_clouds.push_back(inlier_cloud);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(pointcloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*remaining);
        pointcloud.swap(remaining);
    }
    std::cout << "[DrumDetector] detect_circles: 클러스터에서 두 개의 원을 검출\n";
}

std::vector<int> DrumDetector::index_circles(std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs,
                                             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds) {
    const std::vector<int> pos_to_id = {5, 8, 1, 4, 2, 3, 6, 7};
    const float x_group_threshold = 0.1f;
    const size_t n = drum_coeffs.size();

    // 1단계: x 오름차순 인덱스 정렬
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return drum_coeffs[a]->values[0] < drum_coeffs[b]->values[0];
    });

    // 2단계: x가 가까운 그룹 내에서 z 오름차순 정렬
    size_t i = 0;
    while (i < n) {
        size_t j = i + 1;
        while (j < n &&
               std::abs(drum_coeffs[order[j]]->values[0] -
                        drum_coeffs[order[i]]->values[0]) < x_group_threshold) {
            ++j;
        }
        std::sort(order.begin() + i, order.begin() + j, [&](size_t a, size_t b) {
            return drum_coeffs[a]->values[2] < drum_coeffs[b]->values[2];
        });
        i = j;
    }

    // 3단계: pos_to_id 역매핑 — drum_id k가 x/z 정렬에서 몇 번째 위치인지
    std::vector<size_t> drum_id_order(n);
    for (size_t idx = 0; idx < n; ++idx) {
        int target_id = static_cast<int>(idx) + 1;
        for (size_t pos = 0; pos < n; ++pos) {
            if (pos < pos_to_id.size() && pos_to_id[pos] == target_id) {
                drum_id_order[idx] = pos;
                break;
            }
        }
    }

    // 4단계: drum_id 오름차순으로 1회 복사
    std::vector<pcl::ModelCoefficients::Ptr> sorted_coeffs(n);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sorted_clouds(n);
    std::vector<int> drum_ids(n);
    for (size_t idx = 0; idx < n; ++idx) {
        sorted_coeffs[idx] = drum_coeffs[order[drum_id_order[idx]]];
        sorted_clouds[idx] = drum_clouds[order[drum_id_order[idx]]];
        drum_ids[idx] = static_cast<int>(idx) + 1;
        std::cout << "[DrumDetector] index_circles: drum_id=" << drum_ids[idx]
                  << "  x=" << sorted_coeffs[idx]->values[0]
                  << "  z=" << sorted_coeffs[idx]->values[2] << "\n";
    }
    drum_coeffs = sorted_coeffs;
    drum_clouds = sorted_clouds;
    return drum_ids;
}

/*이전 후보점 선정 알고리즘*/
std::vector<Eigen::VectorXd> DrumDetector::select_candidates_for_circle(const pcl::ModelCoefficients::Ptr &coeffs, char DB) {
    std::vector<Eigen::VectorXd> candidate;

    Eigen::Vector3f center(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    float radius = coeffs->values[3];
    Eigen::Vector3f normal(coeffs->values[4], coeffs->values[5], coeffs->values[6]);
    normal.normalize();

    Eigen::Vector3f seed(1.0f, 0.0f, 0.0f);
    if (std::abs(normal.dot(seed)) > 0.9f)
        seed = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    Eigen::Vector3f u = seed.cross(normal).normalized();
    Eigen::Vector3f v = normal.cross(u).normalized();

    if (DB == 'D') {  // 드럼 악기: 중심 + 링(0.4r, 0.8r) x 4방향 = 9개 후보
        int numRings = 2;
        int numAngles = 4;
        candidate.reserve(1 + numRings * numAngles);

        candidate.push_back(center.cast<double>());

        for (int k = 1; k <= numRings; ++k) {
            float rk = radius * 0.4f * static_cast<float>(k);
            for (int j = 0; j < numAngles; ++j) {
                float theta = 2.0f * M_PI * static_cast<float>(j) / static_cast<float>(numAngles);
                Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);
                candidate.push_back(point.cast<double>());
            }
        }
    } else {          // 심벌류: 로봇 방향 에지(0.8r) ±45° = 3개 후보
        float rk = radius * 0.8f;

        Eigen::Vector3f toRobot = -center;
        float projU = toRobot.dot(u);
        float projV = toRobot.dot(v);
        float baseAngle = std::atan2(projV, projU);

        int numCandidates = 3;
        candidate.reserve(numCandidates);

        for (int j = -1; j <= 1; ++j) {
            float theta = baseAngle + static_cast<float>(j) * (M_PI / 4.0f);
            Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);
            candidate.push_back(point.cast<double>());
        }
    }

    // 좌->우(x 오름차순) 정렬. 단, x가 거의 같은(= 같은 좌우 라인) 점들은 한 묶음으로 보고,
    // 그 안에서는 y가 클수록(앞/드럼쪽일수록) 앞 번호를 부여한다.
    // x를 xLineEps 격자로 양자화해 '같은 라인' 판정을 안정적으로 처리한다
    // (부동소수점 == 비교를 피하고, std::sort의 strict weak ordering도 보장).
    const double xLineEps = 0.03; // [m] 좌우 라인 동일 판정 허용오차 (필요시 조정)
    std::sort(candidate.begin(), candidate.end(),
        [xLineEps](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
            double ax = std::round(a(0) / xLineEps);
            double bx = std::round(b(0) / xLineEps);
            if (ax != bx) return ax < bx;          // 왼쪽(-x) -> 오른쪽(+x)
            if (a(1) != b(1)) return a(1) > b(1);  // 같은 라인: y 큰 값이 앞 번호
            return a(2) > b(2);                    // 최종 동률 안정화(결정적)
        });

    std::cout << "[DrumDetector] select_candidates_for_circle: " << DB << " " << candidate.size() << "개 후보 생성\n";
    for (size_t idx = 0; idx < candidate.size(); ++idx) {
        std::cout << "  cand " << idx << ": x=" << candidate[idx](0)
                  << " y=" << candidate[idx](1) << " z=" << candidate[idx](2) << "\n";
    }

    return candidate;
}

std::vector<std::vector<Eigen::VectorXd>> DrumDetector::select_candidates(const std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs) {
    std::vector<std::vector<Eigen::VectorXd>> all_candidates;
    all_candidates.reserve(drum_coeffs.size());

    for (size_t i = 0; i < drum_coeffs.size(); ++i) {
        char DB = (i < 4) ? 'D' : 'B'; // D: 드럼, B: 벨류
        std::vector<Eigen::VectorXd> candidates = select_candidates_for_circle(drum_coeffs[i], DB);
        all_candidates.push_back(candidates);
    }

    return all_candidates;
}
/*
std::vector<Eigen::VectorXd> DrumDetector::select_candidates_for_circle(const pcl::ModelCoefficients::Ptr &coeffs, char DB) {
    std::vector<Eigen::VectorXd> candidate;

    Eigen::Vector3f center(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    float radius = coeffs->values[3];
    Eigen::Vector3f normal(coeffs->values[4], coeffs->values[5], coeffs->values[6]);
    normal.normalize();

    Eigen::Vector3f seed(1.0f, 0.0f, 0.0f);
    if (std::abs(normal.dot(seed)) > 0.9f)
        seed = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    Eigen::Vector3f u = seed.cross(normal).normalized();
    Eigen::Vector3f v = normal.cross(u).normalized();

    if (DB == 'CD') {  // 가까운 드럼 악기(snare, low): 
        
    }
    else (DB == 'FD') {  // 드럼 악기(mid, high): 중심 + 링(0.4r, 0.8r) x 4방향 = 9개 후보
        int numRings = 2;
        int numAngles = 4;
        candidate.reserve(1 + numRings * numAngles);

        candidate.push_back(center.cast<double>());

        for (int k = 1; k <= numRings; ++k) {
            float rk = radius * 0.4f * static_cast<float>(k);
            for (int j = 0; j < numAngles; ++j) {
                float theta = 2.0f * M_PI * static_cast<float>(j) / static_cast<float>(numAngles);
                Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);
                candidate.push_back(point.cast<double>());
            }
        }
    }
    else {          // 심벌류: 로봇 방향 에지(0.8r) ±45° = 3개 후보
        float rk = radius * 0.8f;

        Eigen::Vector3f toRobot = -center;
        float projU = toRobot.dot(u);
        float projV = toRobot.dot(v);
        float baseAngle = std::atan2(projV, projU);

        int numCandidates = 3;
        candidate.reserve(numCandidates);

        for (int j = -1; j <= 1; ++j) {
            float theta = baseAngle + static_cast<float>(j) * (M_PI / 4.0f);
            Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);
            candidate.push_back(point.cast<double>());
        }
    }

    // 좌->우(x 오름차순) 정렬. 단, x가 거의 같은(= 같은 좌우 라인) 점들은 한 묶음으로 보고,
    // 그 안에서는 y가 클수록(앞/드럼쪽일수록) 앞 번호를 부여한다.
    // x를 xLineEps 격자로 양자화해 '같은 라인' 판정을 안정적으로 처리한다
    // (부동소수점 == 비교를 피하고, std::sort의 strict weak ordering도 보장).
    const double xLineEps = 0.03; // [m] 좌우 라인 동일 판정 허용오차 (필요시 조정)
    std::sort(candidate.begin(), candidate.end(),
        [xLineEps](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
            double ax = std::round(a(0) / xLineEps);
            double bx = std::round(b(0) / xLineEps);
            if (ax != bx) return ax < bx;          // 왼쪽(-x) -> 오른쪽(+x)
            if (a(1) != b(1)) return a(1) > b(1);  // 같은 라인: y 큰 값이 앞 번호
            return a(2) > b(2);                    // 최종 동률 안정화(결정적)
        });

    std::cout << "[DrumDetector] select_candidates_for_circle: " << DB << " " << candidate.size() << "개 후보 생성\n";
    for (size_t idx = 0; idx < candidate.size(); ++idx) {
        std::cout << "  cand " << idx << ": x=" << candidate[idx](0)
                  << " y=" << candidate[idx](1) << " z=" << candidate[idx](2) << "\n";
    }

    return candidate;
}

std::vector<std::vector<Eigen::VectorXd>> DrumDetector::select_candidates(const std::vector<pcl::ModelCoefficients::Ptr> &drum_coeffs) {
    std::vector<std::vector<Eigen::VectorXd>> all_candidates;
    all_candidates.reserve(drum_coeffs.size());

    for (size_t i = 0; i < drum_coeffs.size(); ++i) {
        char DB;
        if (i < 2) {DB = 'CD'}
        else if(i < 4) {DB = 'FD'}
        else {DB = 'B'}
        std::vector<Eigen::VectorXd> candidates = select_candidates_for_circle(drum_coeffs[i], DB);
        all_candidates.push_back(candidates);
    }

    return all_candidates;
}
*/
void DrumDetector::visualize_drums(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &drum_clouds,
                                   const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates) {
    if (!visualize) return;
    if (!std::getenv("DISPLAY")) {
        std::cerr << "[DrumDetector] DISPLAY 없음 — 시각화 생략\n";
        return;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Drum Detector"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    // 악기마다 구분되는 색상 팔레트
    const int palette[][3] = {
        {255, 80,  80 },  // red
        {80,  255, 80 },  // green
        {80,  140, 255},  // blue
        {255, 220, 60 },  // yellow
        {255, 120, 255},  // magenta(pink)
        {80,  255, 240},  // cyan(mint)
        {255, 170, 70 },  // orange
        {180, 110, 255},  // purple
    };
    const int palette_size = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < drum_clouds.size(); ++i) {
        if (!drum_clouds[i] || drum_clouds[i]->empty())
            continue;

        const int *c = palette[i % palette_size];
        const std::string id = "drum_" + std::to_string(i);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            color_handler(drum_clouds[i], c[0], c[1], c[2]);
        viewer->addPointCloud<pcl::PointXYZ>(drum_clouds[i], color_handler, id);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    }

    for (size_t i = 0; i < drum_candidates.size(); ++i) {
        for (size_t j = 0; j < drum_candidates[i].size(); ++j) {
            const Eigen::VectorXd &pt = drum_candidates[i][j];
            pcl::PointXYZ p;
            p.x = static_cast<float>(pt(0));
            p.y = static_cast<float>(pt(1));
            p.z = static_cast<float>(pt(2));

            std::string sphere_id = "cand_" + std::to_string(i) + "_" + std::to_string(j);
            viewer->addSphere(p, 0.005, 255.0, 255.0, 255.0, sphere_id);
        }
    }

    // ===== 이벤트 루프: 사용자가 창을 닫을 때까지 대기 =====
    // q 키 입력 시 명시적으로 창 종료 (X 버튼은 wasStopped()로 처리됨)
    viewer->registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "q" && event.keyDown())
            viewer->close();
    });

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    // ===== 루프 종료 후: 창과 VTK 리소스를 확실히 파괴 =====
    // PCL 1.10의 close()는 stopped 플래그와 인터랙터(TerminateApp)만 정리할 뿐
    // OS 레벨 RenderWindow(X11 창)는 파괴하지 않는다. Finalize()를 명시적으로
    // 호출해야 태스크바에 'vtk' 좀비 창이 남지 않는다.
    viewer->close();
    if (viewer->getRenderWindow()) {
        // PCL 1.10 버그 대응(추가): PCLVisualizer 소멸자는 인터랙터에 등록해 둔
        // ExitCallback 등 옵저버를 제거하지 않는다. 인터랙터는 렌더윈도우와의
        // 참조 순환 때문에 뷰어보다 오래 살아남을 수 있고, X 서버가 창 ID를
        // 재사용하면 다음 뷰어 창의 이벤트가 죽은 뷰어의 콜백으로 배달되어
        // use-after-free(segfault)가 난다. 파괴 전에 옵저버를 전부 끊는다.
        if (vtkRenderWindowInteractor *interactor = viewer->getRenderWindow()->GetInteractor()) {
            interactor->RemoveAllObservers();
        }
        viewer->getRenderWindow()->Finalize();
        // (추가) X 창의 실소유자는 렌더윈도우가 아니라 인터랙터의 Xt 위젯이라
        // Finalize()로는 창이 사라지지 않는다 (파괴 요청 자체가 발생 안 함).
        // 렌더윈도우<->인터랙터 참조 순환을 끊어야 viewer.reset() 시 인터랙터가
        // 실제로 파괴되며 XtDestroyWidget으로 창이 닫힌다. 인터랙터 누수가
        // 사라지므로 낡은 콜백에 의한 use-after-free도 원천 차단된다.
        viewer->getRenderWindow()->SetInteractor(nullptr);
    }
    viewer.reset();
}

// =============================================================
// 결과 저장
// =============================================================
bool DrumDetector::write_results(const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates, const std::string &ts) {
    namespace fs = std::filesystem;
    using json = nlohmann::json;

    const std::string config_path = "drumrobot_server/config/drum_coordinate.json";

    // 1) 기존 파일 파싱 (악기 순서·wrist_angle_deg 보존, open hihat 오프셋 계산에 필요)
    json root;
    {
        std::ifstream ifs(config_path);
        if (!ifs.is_open()) {
            std::cerr << "[DrumDetector] " << config_path << " 열기 실패 — 결과 반영 중단\n";
            return false;
        }
        try {
            ifs >> root;
        } catch (const std::exception &e) {
            std::cerr << "[DrumDetector] " << config_path << " 파싱 실패: " << e.what() << " — 결과 반영 중단\n";
            return false;
        }
    }

    // 파일 순서 그대로 악기 항목 추출
    struct InstEntry {
        std::string name;
        int id = -1;
        std::array<double, 3> right_pos{}, left_pos{};
        double right_wrist = 0.0, left_wrist = 0.0;
    };
    std::vector<InstEntry> entries;

    try {
        for (const auto &inst : root.at("instruments")) {
            InstEntry e;
            e.name = inst.at("name").get<std::string>();
            auto it = instrument_name_to_id.find(e.name);
            if (it != instrument_name_to_id.end()) e.id = it->second;

            const auto &right = inst.at("right");
            const auto &left  = inst.at("left");
            for (int i = 0; i < 3; i++) {
                e.right_pos[i] = right.at("position").at(i).get<double>();
                e.left_pos[i]  = left.at("position").at(i).get<double>();
            }
            e.right_wrist = right.at("wrist_angle_deg").get<double>();
            e.left_wrist  = left.at("wrist_angle_deg").get<double>();
            entries.push_back(e);
        }
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] " << config_path << " 형식 이상: " << e.what() << " — 결과 반영 중단\n";
        return false;
    }

    std::map<int, const InstEntry *> by_id;
    for (const auto &e : entries) {
        if (e.id >= 0) by_id[e.id] = &e;
    }

    // 2) 새 좌표 계산 — 검출 결과(레거시 좌표계)를 서버 좌표계로 변환하며 후보 선택
    auto round3 = [](double v) { return std::round(v * 1000.0) / 1000.0; };
    auto to_server = [&](const Eigen::VectorXd &p) {
        return std::array<double, 3>{round3(p(0)), round3(p(1)), round3(p(2) - LEGACY_TO_SERVER_Z)};
    };

    std::map<int, std::array<double, 3>> new_right, new_left;
    for (int id = 1; id <= NUM_CIRCLES; id++) {
        const auto &cand = drum_candidates[id - 1];
        // 드럼(1~4): 후보 9점 x정렬 기준 right=+0.4r(7번), left=-0.4r(1번)
        // 심벌(5~8): 로봇 방향 에지 3점 기준 right=우측(2번), left=좌측(0번)
        //            (하이햇 우손 강제 index 2였던 레거시 selectHitTarget 규칙과 일치)
        const size_t r_idx = (id <= 4) ? 7 : 2;
        const size_t l_idx = (id <= 4) ? 1 : 0;
        if (cand.size() <= r_idx) {
            std::cerr << "[DrumDetector] drum_id " << id << " 후보점 부족 (" << cand.size() << "개) — 결과 반영 중단\n";
            return false;
        }
        new_right[id] = to_server(cand[r_idx]);
        new_left[id]  = to_server(cand[l_idx]);
    }
    // open hihat(9)은 closed hihat(5)과 동일 물리 심벌:
    // 새 closed 좌표 + 기존 파일의 (open - closed) 오프셋을 성분별로 보존
    if (by_id.count(5) && by_id.count(9)) {
        std::array<double, 3> r{}, l{};
        for (int i = 0; i < 3; i++) {
            r[i] = round3(new_right.at(5)[i] + (by_id.at(9)->right_pos[i] - by_id.at(5)->right_pos[i]));
            l[i] = round3(new_left.at(5)[i]  + (by_id.at(9)->left_pos[i]  - by_id.at(5)->left_pos[i]));
        }
        new_right[9] = r;
        new_left[9]  = l;
    }

    // 3) 백업 후 원자적 교체 (.tmp에 쓰고 rename — 중간 크래시에도 원본 보존)
    const std::string backup_path = "drumrobot_server/config/drum_coordinate_backup_" + ts + ".json";
    try {
        fs::copy_file(config_path, backup_path, fs::copy_options::overwrite_existing);
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] 백업 실패: " << e.what() << " — 결과 반영 중단\n";
        return false;
    }

    // 기존 파일 스타일(악기당 한 줄) 유지 — git diff 가독성
    auto fmt_pos = [](const std::array<double, 3> &p) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "[" << p[0] << ", " << p[1] << ", " << p[2] << "]";
        return oss.str();
    };
    auto fmt_wrist = [](double v) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << v;
        return oss.str();
    };

    size_t name_width = 0;
    for (const auto &e : entries) name_width = std::max(name_width, e.name.size());

    const std::string tmp_path = config_path + ".tmp";
    {
        std::ofstream ofs(tmp_path);
        if (!ofs.is_open()) {
            std::cerr << "[DrumDetector] " << tmp_path << " 쓰기 실패 — 결과 반영 중단\n";
            return false;
        }
        ofs << "{\n  \"instruments\": [\n";
        for (size_t i = 0; i < entries.size(); i++) {
            const auto &e = entries[i];
            const bool detected = (e.id >= 1 && new_right.count(e.id) > 0);
            const auto &rp = detected ? new_right.at(e.id) : e.right_pos;
            const auto &lp = detected ? new_left.at(e.id)  : e.left_pos;

            std::string padded_name = "\"" + e.name + "\"," + std::string(name_width - e.name.size(), ' ');

            ofs << "    { \"name\": " << padded_name
                << " \"right\": {\"position\": " << fmt_pos(rp)
                << ", \"wrist_angle_deg\": " << fmt_wrist(e.right_wrist) << "},"
                << " \"left\": {\"position\": " << fmt_pos(lp)
                << ", \"wrist_angle_deg\": " << fmt_wrist(e.left_wrist) << "} }"
                << (i + 1 < entries.size() ? "," : "") << "\n";
        }
        ofs << "  ]\n}\n";
    }
    try {
        fs::rename(tmp_path, config_path);
    } catch (const std::exception &e) {
        std::cerr << "[DrumDetector] " << config_path << " 교체 실패: " << e.what() << "\n";
        return false;
    }

    // 4) 전체 후보점 덤프 (서버 좌표계) — 수동 검토/디버깅용
    dump_candidates_csv("drumrobot_server/data/scan/scan_" + ts + "_candidates.csv", drum_candidates);

    // 5) 이전 좌표 대비 이동량 로그 (정합성 눈검사용)
    for (const auto &e : entries) {
        if (e.id < 1 || new_right.count(e.id) == 0) continue;
        double dr = 0.0, dl = 0.0;
        for (int i = 0; i < 3; i++) {
            dr += std::pow(new_right.at(e.id)[i] - e.right_pos[i], 2);
            dl += std::pow(new_left.at(e.id)[i]  - e.left_pos[i], 2);
        }
        dr = std::sqrt(dr);
        dl = std::sqrt(dl);
        std::cout << "[DrumDetector] " << e.name << ": |dR|=" << std::fixed << std::setprecision(3) << dr
                  << "m |dL|=" << dl << "m"
                  << ((dr > 0.15 || dl > 0.15) ? "  <-- 경고: 이동량 큼, 오검출 여부 확인 필요" : "") << "\n";
    }

    std::cout << "[DrumDetector] " << config_path << " 갱신 완료 (백업: " << backup_path << ")\n";
    return true;
}

void DrumDetector::dump_cloud_csv(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[DrumDetector] 파일 열기 실패: " << path << "\n";
        return;
    }

    file << std::fixed << std::setprecision(6);
    for (const auto &point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            // 서버 좌표계로 변환해 저장
            file << point.x << "," << point.y << "," << (point.z - LEGACY_TO_SERVER_Z) << "\n";
        }
    }
    std::cout << "[DrumDetector] " << path << " 저장 완료\n";
}

void DrumDetector::dump_candidates_csv(const std::string &path, const std::vector<std::vector<Eigen::VectorXd>> &drum_candidates) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[DrumDetector] 파일 열기 실패: " << path << "\n";
        return;
    }

    file << "drum_id,cand_idx,x,y,z\n";
    file << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < drum_candidates.size(); ++i) {
        int drum_id = static_cast<int>(i) + 1;
        for (size_t j = 0; j < drum_candidates[i].size(); ++j) {
            const Eigen::VectorXd &pt = drum_candidates[i][j];
            // 서버 좌표계로 변환해 저장
            file << drum_id << "," << j << "," << pt(0) << "," << pt(1) << "," << (pt(2) - LEGACY_TO_SERVER_Z) << "\n";
        }
    }
    std::cout << "[DrumDetector] " << path << " 저장 완료\n";
}

std::string DrumDetector::timestamp_suffix() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%m%d_%H%M");
    return oss.str();
}
