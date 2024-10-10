#include "path_to_trajectory/path_to_trajectory.hpp"

// route.cppでincludeされているヘッダファイル
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>

constexpr double kmh = 1000.0/3600.0;

struct PathPoint {
    double time;
    double x, y, th;
    double vel, curvature;
    double acc;
};

class Particle {
public:
    double path_time;
    Particle() = default;
    virtual std::vector<PathPoint> generate_path(double dt, double st=0) = 0;
};

struct XY {
    double x, y;
};

// st_x, st_y, st_vel, ed_x, ed_y, ed_vel
class Line : public Particle {
public:
    static constexpr double max_acc = 2.0; // m/s^2
    
    Line(std::vector<std::string> data) : Particle() {
        this->st.x = std::stod(data[1]);
        this->st.y = std::stod(data[2]);
        this->start_vel = std::stod(data[3]) * kmh;
        this->ed.x = std::stod(data[4]);
        this->ed.y = std::stod(data[5]);
        this->end_vel = std::stod(data[6]) * kmh;
        this->dir = std::atan2(this->ed.y - this->st.y, this->ed.x - this->st.x);
    }

    XY st, ed;
    double dir;
    double start_vel, end_vel;

    std::vector<PathPoint> generate_path(double dt, double st=0) override {
        double length = std::hypot(this->ed.x - this->st.x, this->ed.y - this->st.y);
        path_time = length / ((this->start_vel + this->end_vel) / 2.0);
        double acc = (this->end_vel - this->start_vel) / path_time;

        std::vector<PathPoint> path;
        for (double t = st; t < path_time; t += dt) {
            double vel = this->start_vel + acc * t;
            double progress = (this->start_vel + vel) / 2.0 * t / length;
            double x = this->st.x * (1 - progress) + this->ed.x * progress;
            double y = this->st.y * (1 - progress) + this->ed.y * progress;
            double th = this->dir;
            double curvature = 0.0;
            path.push_back({t, x, y, th, vel, curvature, acc});
        }

        return path;
    }
};

// center_x, center_y, radius, start_angle, end_angle, vel
class Arc : public Particle {
public:
    XY center;
    double radius;
    double start_angle, end_angle;
    double vel;

    Arc(std::vector<std::string> data) : Particle() {
        this->center.x = std::stod(data[1]);
        this->center.y = std::stod(data[2]);
        this->radius = std::stod(data[3]);
        this->start_angle = std::stod(data[4]);
        this->end_angle = std::stod(data[5]);
        this->vel = std::stod(data[6]) * kmh;
    };

    std::vector<PathPoint> generate_path(double dt, double st=0) override {
        double length = this->radius * std::abs(this->end_angle - this->start_angle);
        path_time = length / this->vel;
        double acc = 0.0;

        std::vector<PathPoint> path;
        for (double t = st; t < path_time; t += dt) {
            double angle = this->start_angle + (this->end_angle - this->start_angle) * t / path_time;
            double x = this->center.x + this->radius * std::cos(angle);
            double y = this->center.y + this->radius * std::sin(angle);
            double th = angle + M_PI / 2.0;
            double vel = this->vel;
            double curvature = 1.0 / this->radius;
            path.push_back({t, x, y, th, vel, curvature, acc});
        }

        return path;
    }
};

class PathTrajectory {
public:
    std::vector<std::unique_ptr<Particle>> particles;
    double path_time;

    PathTrajectory(std::string filepath) {
        std::ifstream file(filepath);
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::vector<std::string> data;
            std::string token;
            while (std::getline(ss, token, ',')) {
                data.push_back(token);
            }
            if (line[0] == 'L') {
                particles.push_back(std::make_unique<Line>(data));
            } else if (line[0] == 'A') {
                particles.push_back(std::make_unique<Arc>(data));
            }
        }
    }

    std::vector<PathPoint> generate_path(double dt) {
        std::vector<PathPoint> path = particles[0]->generate_path(dt);
        double time_offset = particles[0]->path_time;
        for (int i = 1; i < particles.size(); i++) {
            double st = dt - (time_offset - path.back().time);
            auto next_path = particles[i]->generate_path(dt, st);
            for (int j = 0; j < next_path.size(); j++) {
                next_path[j].time += time_offset;
                path.push_back(next_path[j]);
            }
            time_offset += particles[i]->path_time;
        }
        path_time = time_offset;
        return path;
    }

    void write_path(std::string filepath, double dt = 0.01) {
        std::vector<PathPoint> path = generate_path(dt);
        std::ofstream outfile(filepath);
        outfile << "time,x,y,th,vel,curvature,acc" << std::endl;
        for (int i = 0; i < path.size(); i++) {
            outfile << path[i].time << "," << path[i].x << "," << path[i].y << "," << path[i].th << "," << path[i].vel << "," << path[i].curvature << "," << path[i].acc << std::endl;
        }
    }
};

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node") {
    using std::placeholders::_1;

    pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("output", 1);
    sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>("input", 1, std::bind(&PathToTrajectory::callback, this, _1));
}

void PathToTrajectory::callback(const autoware_auto_planning_msgs::msg::PathWithLaneId::SharedPtr msg) {
    // particles.csvからPathTrajectoryを生成
    auto traj = PathTrajectory("particles.csv");

    // PathTrajectoryの経路を生成 (dt = 0.01を仮定)
    std::vector<PathPoint> generated_path = traj.generate_path(0.01);

    // ROS 2 の Trajectory メッセージ型に変換
    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = msg->header;  // 元のメッセージのヘッダーを使用

    for (const auto& path_point : generated_path) {
        autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
        trajectory_point.pose.position.x = path_point.x;
        trajectory_point.pose.position.y = path_point.y;
        trajectory_point.longitudinal_velocity_mps = path_point.vel;
        
        trajectory.points.push_back(trajectory_point);
    }

    // Trajectory メッセージをパブリッシュ
    pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathToTrajectory>());
    rclcpp::shutdown();
    return 0;
}
