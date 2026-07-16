#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <vamp/collision/factory.hh>
#include <vamp/collision/filter_centervox.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/robots/piper.hh>
#include <vamp/random/halton.hh>

#include "libobsensor/ObSensor.hpp"

// #define DEBUG_SELF_FILTER

using Robot = vamp::robots::Piper;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

static ob::Pipeline  g_pipeline;
static ob::PointCloudFilter g_pointCloud;
static bool g_camera_initialized = false;

// UDP request from ROS device:
//   magic[8] = "VAMP_PC\0", seq_id:uint32, current_state:6*float32
static constexpr const char   MAGIC[]      = "VAMP_PC\0"; // 8 bytes incl.
static constexpr const size_t MAGIC_LEN    = 8;
static constexpr const int    UDP_PORT     = 9876;

struct ReceivedData
{
    uint32_t seq_id;
    Robot::ConfigurationArray current_state;
};

// ─── Camera transform (camera_link → base_link) ────────────────────────────────
struct Transform {
    float tx, ty, tz;
};

static std::array<float, 3> transformPoint(const std::array<float, 3>& p, const Transform& t)
{
    float R[3][3] = {
        { -1.0f,  0.0f,  0.0f },
        {  0.0f,  0.0f, -1.0f },
        {  0.0f, -1.0f,  0.0f }
    };
    std::array<float, 3> out;
    out[0] = R[0][0]*p[0] + R[0][1]*p[1] + R[0][2]*p[2] + t.tx;
    out[1] = R[1][0]*p[0] + R[1][1]*p[1] + R[1][2]*p[2] + t.ty;
    out[2] = R[2][0]*p[0] + R[2][1]*p[1] + R[2][2]*p[2] + t.tz;
    return out;
}

static const Transform cam_to_base = { 0.06f, 0.495f, 0.16f};

class UDPReceiver
{
public:
    explicit UDPReceiver(int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
            throw std::runtime_error("socket() failed");

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port));

        if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            ::close(fd_);
            throw std::runtime_error("bind() failed");
        }
        std::cout << "[UDP] Listening on port " << port << std::endl;
    }

    ~UDPReceiver() { ::close(fd_); }

private:
    bool parse_packet(const uint8_t *buf, ssize_t n, ReceivedData &out)
    {
        constexpr size_t EXPECTED =
            MAGIC_LEN + sizeof(uint32_t) + Robot::dimension * sizeof(float);

        if (n != static_cast<ssize_t>(EXPECTED))
        {
            std::cerr << "[UDP] Bad packet size: " << n << "\n";
            return false;
        }

        if (std::memcmp(buf, MAGIC, MAGIC_LEN) != 0)
        {
            std::cerr << "[UDP] Magic mismatch\n";
            return false;
        }

        const uint8_t *ptr = buf + MAGIC_LEN;

        std::memcpy(&out.seq_id, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            std::memcpy(&out.current_state[j], ptr, sizeof(float));
            ptr += sizeof(float);
        }

        return true;
    }

public:
    bool receive_latest(ReceivedData &out)
    {
        constexpr size_t EXPECTED =
            MAGIC_LEN + sizeof(uint32_t) + Robot::dimension * sizeof(float);

        uint8_t buf[EXPECTED];

        // Block for one request, then drain queued UDP packets so planning uses the newest state.
        ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (!parse_packet(buf, n, out))
            return false;

        while (true)
        {
            ReceivedData tmp;
            ssize_t m = ::recvfrom(fd_, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr);

            if (m < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;

                std::cerr << "[UDP] recvfrom failed while draining\n";
                break;
            }

            if (parse_packet(buf, m, tmp))
                out = tmp;
        }

        return true;
    }

private:
    int fd_;
};

void init_camera()
{
    if (g_camera_initialized) return;

    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

    auto config = std::make_shared<ob::Config>();
    auto depthProfileList = g_pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    auto depthProfile     = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
    config->enableStream(depthProfile);
    config->setAlignMode(ALIGN_DISABLE);
    g_pipeline.start(config);

    auto cameraParam = g_pipeline.getCameraParam();
    g_pointCloud.setCameraParam(cameraParam);
    g_pointCloud.setCreatePointFormat(OB_FORMAT_POINT);

    for (int i = 0; i < 10; ++i) {
        auto t_frame = g_pipeline.waitForFrames(1000); 
        if (t_frame) break; 
    }

    g_camera_initialized = true;
    std::cerr << "Camera init done\n";
}

bool fetch_pointcloud(std::vector<vamp::collision::Point> &out_pc)
{
    auto frameset = g_pipeline.waitForFrames(100);   
    if (!frameset || !frameset->depthFrame())
        return false;

    auto depthValueScale = frameset->depthFrame()->getValueScale();
    g_pointCloud.setPositionDataScaled(depthValueScale);

    auto frame = g_pointCloud.process(frameset);
    if (!frame) return false;

    int pointsSize = frame->dataSize() / sizeof(OBPoint);
    OBPoint *raw = reinterpret_cast<OBPoint *>(frame->data());

    out_pc.clear();
    out_pc.reserve(pointsSize);

    static constexpr float max_dist = 1.5f;   
    static constexpr float mm_to_m  = 1e-3f;   

    for (int i = 0; i < pointsSize; ++i)
    {
        float x = raw[i].x * mm_to_m;
        float y = raw[i].y * mm_to_m;
        float z = raw[i].z * mm_to_m;

        if (fabsf(x) > max_dist || fabsf(y) > max_dist || fabsf(z) > max_dist) continue;

        std::array<float, 3> p_cam  = { x, y, z };
        std::array<float, 3> p_base = transformPoint(p_cam, cam_to_base);

        out_pc.push_back({ p_base[0], p_base[1], p_base[2] });
    }

    return true;
}

using Cfg = Robot::Configuration;
using CfgArr = Robot::ConfigurationArray;

static const std::array<std::vector<Cfg>, 2> goal = {{
    
    // IK candidates for Cartesian target [-0.4, 0.4, 0.45].
    {
        Cfg(CfgArr{ 2.11117809,  1.85908382, -1.84176752,  1.74399105,  0.47102160,  1.34595315}),
        Cfg(CfgArr{-1.12977201,  0.02766650, -2.05340662, -1.62791853,  0.95642239,  0.35719388}),
        Cfg(CfgArr{-0.44103961,  0.00830091, -2.09503219,  1.62037272,  0.82011956,  1.88855213}),
        Cfg(CfgArr{ 2.11789433,  1.85223746, -1.85166695,  1.59543838,  0.44484674,  1.65446479}),
        Cfg(CfgArr{-0.45071502,  0.03966534, -1.99487908, -1.07380665, -0.91236628, -2.09439897}),
        Cfg(CfgArr{ 2.15937533,  1.84413092, -1.65382009, -1.11944125, -0.54269335, -0.66505553}),
        Cfg(CfgArr{ 2.04071302,  1.97162149, -1.97498786,  1.70574985,  1.05214941, -1.58169643}),
        Cfg(CfgArr{ 2.05080836,  1.95048043, -2.00044511,  1.65956034,  0.66419759,  1.23069691}),
        Cfg(CfgArr{-1.15372592,  0.11841342, -2.32915467, -1.37125746, 1.145927947, -0.39432991}),
        Cfg(CfgArr{ 2.04134227,  1.97004945, -2.02944858, 1.478415783, 0.704682867, 2.031240145})
    },
    
    // IK candidates for Cartesian target [-0.4, -0.4, 0.45].
    {
        Cfg(CfgArr{-2.28034114,  1.75410992, -1.68382479, -1.46139594,  0.06290362,  0.68919569}),
        Cfg(CfgArr{ 0.53314006,  0.02505865, -1.78916767,  0.66814327, -0.98944205, -1.21076248}),
        Cfg(CfgArr{-2.03278568,  2.01560577, -1.94516347,  1.12078706, -0.82348795, -0.93121954}),
        Cfg(CfgArr{-2.33757311,  1.75177519, -1.77447375, -1.74499996,  0.18245156, -2.09439999}),
        Cfg(CfgArr{-2.32718223,  1.75559901, -1.60238687,  0.16589030, -0.12522017, -1.43737259}),
        Cfg(CfgArr{ 0.46330814,  0.00004013, -2.30524866,  1.70652918, -0.87042743, -0.49262154}),
        Cfg(CfgArr{ 1.17498857,  0.37163836, -2.79813244,  1.67389241,  1.12107302,  0.48056189}),
        Cfg(CfgArr{-2.28202822,  1.76125991, -1.79289454, -0.68216029,  0.34222810, -1.07500224}),
        Cfg(CfgArr{ 0.46351480, 0.063215637, -2.47704157, -1.07732786,  1.20526482, -1.32542140}),
        Cfg(CfgArr{ 1.15934996, 0.365037175, -2.53208745, -1.20647595, -1.08352032, -1.72031539})
    }
}};

void save_spheres_to_ply(const Robot::Spheres<1>& out, const std::string& filename)
{
    std::ofstream file(filename);
    file << "ply\nformat ascii 1.0\n";
    file << "element vertex " << Robot::n_spheres << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    file << "end_header\n";
    
    for (auto i = 0U; i < Robot::n_spheres; ++i) {
        file << out.x[{i,0}] << " " << out.y[{i,0}] << " " << out.z[{i,0}] << " 255 0 0\n";
    }
    file.close();
}

std::vector<vamp::collision::Point> filter_robot_from_pc(
    const std::vector<vamp::collision::Point> &pc,
    float point_radius,
    const Robot::ConfigurationArray &current_state)
{
    Robot::Spheres<1> out;
    
    Robot::ConfigurationBlock<1> c_in;
    for (std::size_t i = 0; i < Robot::dimension; ++i) {
        c_in[i][0] = current_state[i]; 
    }
#ifdef DEBUG_SELF_FILTER
    std::cout << "\n--- FK Input Debug ---\n";
    std::cout << "1. current_state : ";
    for(int i=0; i<6; i++) std::cout << std::fixed << std::setprecision(3) << current_state[i] << " ";
    std::cout << "\n";
    
    std::cout << "2. c_in 實際數值 : ";
    for(int i=0; i<6; i++) std::cout << std::fixed << std::setprecision(3) << c_in[{i, 0}] << " ";
    std::cout << "\n----------------------\n";
#endif
    Robot::template sphere_fk<1>(c_in, out);
#ifdef DEBUG_SELF_FILTER
    save_spheres_to_ply(out, "robot_fk.ply");
#endif

    std::vector<vamp::collision::Point> filtered;
    filtered.reserve(pc.size());

    for (const auto &point : pc)
    {
        const float x = point[0], y = point[1], z = point[2], r = point_radius;
        bool valid = true;
        
        for (auto i = 0U; i < Robot::n_spheres; ++i)
        {
            if (vamp::collision::sphere_sphere_sql2(
                    out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}], x, y, z, r) < 0)
            {
                valid = false;
                break;
            }
        }

        if (valid) {
            filtered.emplace_back(point);
        }
    }
    return filtered;
}

void save_pc_to_ply(const std::vector<vamp::collision::Point>& pc, const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open())
    {
        std::cerr << "[PLY] Error: cannot write " << filename << "\n";
        return;
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << pc.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    
    out << "end_header\n";

    for (const auto& point : pc)
    {
        out << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    out.close();
    std::cout << "[PLY] Pointcloud is saved in " << filename << " (" << pc.size() << " points)\n";
}

auto main(int, char **) -> int
{
    UDPReceiver receiver(UDP_PORT);

    std::vector<vamp::collision::Point> filtered_pc_centervox;
    float r_min = 0.008;
    float r_max = 0.07;
    float max_extent = 0.63;
    vamp::collision::Point workspace_aabb_min = {-max_extent, -max_extent, -max_extent};
    vamp::collision::Point workspace_aabb_max = {max_extent, max_extent, max_extent};
    vamp::collision::Point origin = {0.0, 0.0, 0.0};
    float r_point = 0.0025;

    init_camera();

    int target_goal_idx = 0;

    while (true)
    {
        ReceivedData data;
        if (!receiver.receive_latest(data))
            continue;
        Robot::ConfigurationArray start = data.current_state;        

        float error = 1000.0;
        for (const auto& config : goal[target_goal_idx]) {
            float current_max_error = 0.0f;
            auto config_array = config.to_array();
            for (std::size_t i = 0; i < Robot::dimension; ++i) {
                current_max_error = std::max(current_max_error, std::abs(start[i] - config_array[i]));
            }
            error = std::min(error, current_max_error);
        }

        if (error < 0.05f) {
            std::cout << "\nReached goal " << target_goal_idx 
                      << " (Error: " << error << "). Switching goal!\n\n";
            target_goal_idx = 1 - target_goal_idx;
        } 

        std::vector<vamp::collision::Point> pc; 
        if (!fetch_pointcloud(pc))
        {
            std::cerr << "[Camera] fetch failed, skip this cycle\n";
            continue;
        } 
 
#ifdef DEBUG_SELF_FILTER
        save_pc_to_ply(pc, "env_before_filter_robot.ply");
#endif

        std::vector<vamp::collision::Point> pc_no_robot = filter_robot_from_pc(pc, r_point * 3, start);
        
#ifdef DEBUG_SELF_FILTER
        save_pc_to_ply(pc_no_robot, "env_after_filter_robot.ply");
        return 0;
#endif

        EnvironmentInput mvt_environment;

        filtered_pc_centervox = vamp::collision::filter_pointcloud_centervox(pc_no_robot, 0.031, max_extent, origin, workspace_aabb_min, workspace_aabb_max);
        mvt_environment.pointclouds_mvt.emplace_back(filtered_pc_centervox, r_max, workspace_aabb_min, workspace_aabb_max, r_point);
        auto env_v = EnvironmentVector(mvt_environment);

        auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
        
        vamp::planning::RRTCSettings rrtc_settings;
        rrtc_settings.range = 1.0;
        rrtc_settings.max_iterations = 150000;

        auto result =
            RRTC::solve(Robot::Configuration(start), goal[target_goal_idx], env_v, rrtc_settings, rng);

        uint32_t N = 0;
        std::vector<Robot::Configuration> final_path;

        if (result.path.size() > 0)
        {
            vamp::planning::SimplifySettings simplify_settings;
            auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, env_v, simplify_settings, rng);
            final_path = simplify_result.path;
            N = final_path.size();
        } 
        else 
        {
            std::cout << "Planning failed. Path size: 0\n";
        }

        std::string target_ip = "192.168.50.34";
        int target_port = 8080;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
            std::cerr << "Socket creation failed\n";
            return 1;
        }

        struct sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_port = htons(target_port);
        server.sin_addr.s_addr = inet_addr(target_ip.c_str());

        if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
            std::cerr << "Connection failed\n";
            close(sock);
            continue;
        }

        // TCP response to ROS device:
        //   seq_id:uint32, N:uint32, path:N*6*float32
        uint32_t header[2] = {data.seq_id, N};
        send(sock, header, sizeof(header), 0);

        if (N > 0)
        {
            const size_t payload_size = N * 6 * sizeof(float);
            std::vector<char> buffer(payload_size);
            size_t offset = 0;
            for (const auto &config : final_path)
            {
                const auto &arr = config.to_array(); 
                std::memcpy(buffer.data() + offset, arr.data(), 24);
                offset += 24;
            }

            size_t bytes_left = payload_size;
            const char* ptr = buffer.data();
            while (bytes_left > 0)
            {
                ssize_t bytes_sent = send(sock, ptr, bytes_left, 0);
                if (bytes_sent < 0) break;
                ptr += bytes_sent;
                bytes_left -= bytes_sent;
            }
        }

        char dummy;
        recv(sock, &dummy, 1, 0);
        close(sock);
    }

    return 0;
}
