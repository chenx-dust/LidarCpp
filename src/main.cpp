#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include "message/proto/lidar.pb.h"
#include "spdlog/spdlog.h"
#include <cstddef>
#include <csignal>
#include <ctype.h>
#include <zmq.hpp>

uint32_t public_handle = 0;
zmq::context_t ctx;
zmq::socket_t sock(ctx, zmq::socket_type::pub);
std::condition_variable quit_condition;
std::mutex mtx;

rdr::LiDARRawPoints convertPointCloud(const LivoxLidarCartesianHighRawPoint *point_cloud, const uint32_t point_num)
{
    rdr::LiDARRawPoints points;
    for (uint32_t i = 0; i < point_num; ++i)
    {
        auto point = points.add_points();
        point->set_x(point_cloud[i].x);
        point->set_y(point_cloud[i].y);
        point->set_z(point_cloud[i].z);
    }
    return points;
}

void infoCallback(const uint32_t handle, const LivoxLidarInfo *info, void *client_data)
{
    spdlog::info("收到设备信息 dev_type: {}, sn: {}, lidar_ip: {}", info->dev_type, info->sn, info->lidar_ip);
    public_handle = handle;

    SetLivoxLidarPclDataType(handle, kLivoxLidarCartesianCoordinateHighData, nullptr, nullptr);
}

void pointCloudCallback(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
    if (data == nullptr)
        return;
    spdlog::debug("收到点云数据 dot_num: {}, length: {}, data_type: {}", data->dot_num, data->length, data->data_type);
    if (data->data_type != kLivoxLidarCartesianCoordinateHighData)
    {
        spdlog::warn("不支持的点云数据类型 {}", data->data_type);
        return;
    }
    auto points = convertPointCloud((LivoxLidarCartesianHighRawPoint *)data->data, data->dot_num);
    std::string buffer;
    points.SerializeToString(&buffer);
    sock.send(zmq::buffer(buffer));
    spdlog::debug("点云数据已发送");
}

void Stop(int signal)
{
    quit_condition.notify_all();
}

int main()
{
    if (!LivoxLidarSdkInit("config.json"))
    {
        spdlog::error("Livox SDK 初始化失败");
        LivoxLidarSdkUninit();
        throw std::runtime_error("Livox SDK 初始化失败");
    }
    LivoxLidarSdkStart();
    SetLivoxLidarInfoChangeCallback(infoCallback, nullptr);
    SetLivoxLidarPointCloudCallBack(pointCloudCallback, nullptr);
    spdlog::info("Livox SDK 初始化成功");

    sock.bind("tcp://0.0.0.0:8200");
    spdlog::info("ZeroMQ 连接至 tcp://0.0.0.0:8200");

    std::signal(SIGINT, Stop);

    std::unique_lock<std::mutex> lock(mtx);
    quit_condition.wait(lock);

    LivoxLidarSdkUninit();
    printf("退出完毕");
    return 0;
}
