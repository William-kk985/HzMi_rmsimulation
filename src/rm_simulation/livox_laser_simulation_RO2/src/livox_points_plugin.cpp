#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>  // Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/livox_ode_multiray_shape.h"

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
        }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");


        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + "/pointcloud", 10);
        // CustomMsg publisher
        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        // 这行日志用于确认加载的是修好时间基的插件（旧库打的是墙钟偏移）
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"),
                    "timebase: SIM clock, per-point offset_time = 0 (intra-frame motionless)");
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }



    void LivoxPointsPlugin::OnNewLaserScans() {
        if (!rayShape) {
            return; // 检查是否已经初始化了 rayShape
        }

        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        // ★ 2026-09-22 修正（关键）：本帧所有点的采样时刻 = 当前仿真时刻。
        //   Gazebo 的射线传感器是"一次回调把所有射线全部打完"（上面那句 rayShape->Update()），
        //   一帧之内机器人位姿不变 → 帧内没有任何运动，所有点属于同一个仿真时刻。
        //   两条消息必须共用同一个戳：原来各调用一次 now()，差一个仿真步都会让 /scan 与 /odom 对不齐。
        const rclcpp::Time stamp = node_->get_clock()->now();

        // 创建自定义消息 pp_livox，用于发布 Livox CustomMsg 类型消息
        livox_ros_driver2::msg::CustomMsg pp_livox;
        pp_livox.header.stamp = stamp;
        pp_livox.header.frame_id = raySensor->Name();
        int count = 0;

        // 用于 PointCloud2 类型消息发布
        sensor_msgs::msg::PointCloud2 cloud2;
        cloud2.header.stamp = stamp;
        cloud2.header.frame_id = raySensor->Name();

        sensor_msgs::PointCloud2Modifier modifier(cloud2);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // ★★ 2026-09-23（nav 模式排查）：**只发有效回波，不再用 (0,0,0) 占位**。
        //   实测一帧 30000 个射线方向里只有 ~6200 个真有回波（20.8%），其余 78% 被填成 (0,0,0)
        //   发出去 ⇒ 消息 480 KB/帧 @10Hz = 4.8 MB/s 灌进 DDS。后果：
        //     · 任何消费端（linefit / nav2 costmap / RViz）一卡，**RELIABLE + KEEP_LAST(10)**
        //       的写者就会积压 → 阻塞 Gazebo 的 sensor 回调 → **整条感知链冻死且不自恢复**
        //       （实测现象：/scan 与 /segmentation/obstacle 同时停更 180 s，而 linefit/p2l 进程还活着）。
        //     · 真实 Livox 驱动**只发有回波的采样点**，(0,0,0) 本来就不是它发的 ⇒ 这也是保真度修复。
        //   改法：先数一遍有效回波数，按它 resize，再只填有效点（CustomMsg 同步受益）。
        {
            size_t valid = 0;
            for (const auto &pair : points_pair) {
                const double r = rayShape->GetRange(pair.first);
                if (r > RangeMin() && r < RangeMax()) ++valid;
            }
            modifier.resize(valid);
        }

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud2, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud2, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud2, "z");

        // 遍历射线扫描点对
        for (const auto &pair : points_pair) {
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);

            // 无回波 / 超范围：**整点丢弃**（不再填 (0,0,0)）—— 见上方 2026-09-23 说明
            if (range <= RangeMin() || range >= RangeMax()) {
                continue;
            }

            // 计算点云数据
            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;

            // 填充 CustomMsg 点云消息
            livox_ros_driver2::msg::CustomPoint p;
            p.x = point.X();
            p.y = point.Y();
            p.z = point.Z();
            p.reflectivity = intensity;

            // 填充 PointCloud2 点云消息
            *out_x = point.X();
            *out_y = point.Y();
            *out_z = point.Z();

            ++out_x;
            ++out_y;
            ++out_z;

            // ★ 2026-09-22 修正（关键）：逐点时间偏移必须与 header 戳同一条时间轴（**仿真钟**）。
            //   原来这里用 boost::chrono::high_resolution_clock（**墙钟**）累计"生成一帧花了多久"，
            //   而 header 戳是仿真钟 → 两个时间基准混用。下游 FAST-LIO 的用法（laserMapping.cpp:396-410）：
            //       lidar_end_time = header 戳 + 末点 offset      // 再用于 odom.header.stamp (:632)
            //   => /odom 的时间戳 = 仿真戳 + 一帧墙钟耗时：RTF<1 时是 1.3~3 倍（本机 RTF≈0.76），
            //      且逐帧随 CPU 负载抖动；同时 FAST-LIO 拿这个假跨度做去畸变，
            //      把"其实根本没发生"的帧内运动补偿掉 → 转起来时整帧被拧、odom 位姿带负载相关偏差。
            //   cartographer 侧的表现：偶发 Check failed: timed_pose_queue_... → exit -6(SIGABRT)，
            //      以及开 use_odometry 时先验按错时刻套用 → 地图跟着车转（详见 docs/issues_and_findings.md）。
            //   仿真里帧内无运动，正确值就是 0（FAST-LIO 于是得到 lidar_end_time == header 戳，自洽）。
            //   若将来真要建帧内运动模型，应使用 scan_mode/mid360.csv 的 Time 列（单位需先确认），
            //   而不是墙钟。
            p.offset_time = 0;

            // 将点云数据添加到 CustomMsg 消息中
            pp_livox.points.push_back(p);
            count++;
        }

        if (scanPub && scanPub->HasConnections()) {
            scanPub->Publish(laserMsg);
        }

        // 发布 CustomMsg 消息
        pp_livox.point_num = count;
        custom_pub->publish(pp_livox);

        // 发布 PointCloud2 类型消息
        cloud2_pub->publish(cloud2);
    }


    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());
        for (int k = currStartIndex; k < end_index; k += downSample)
        {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size)
            {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex += samplesStep;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}