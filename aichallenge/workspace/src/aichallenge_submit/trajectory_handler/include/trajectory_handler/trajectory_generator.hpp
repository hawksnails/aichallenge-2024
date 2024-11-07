#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>


constexpr double kmh = 1000.0/3600.0;

struct PathPoint {
    double time=0;
    double x=0, y=0, th=0;
    double vel=0, curvature=0;
    double acc=0;
    double heading_rate=0;

    PathPoint operator+(const PathPoint& rhs) const {
        PathPoint ret = rhs;
        ret.x = x + std::cos(th) * rhs.x - std::sin(th) * rhs.y;
        ret.y = y + std::sin(th) * rhs.x + std::cos(th) * rhs.y;
        ret.th = th + rhs.th;
        return ret;
    }
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
      try {
        this->st.x = std::stod(data.at(1));
        this->st.y = std::stod(data.at(2));
        this->start_vel = std::stod(data.at(3)) * kmh;
        this->ed.x = std::stod(data.at(4));
        this->ed.y = std::stod(data.at(5));
        this->end_vel = std::stod(data.at(6)) * kmh;
        this->dir = std::atan2(this->ed.y - this->st.y, this->ed.x - this->st.x);
      } catch (const std::exception& e) {
        std::cout << "in constructor of Line:" << e.what() << std::endl;
      }
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
      try {
        this->center.x = std::stod(data.at(1));
        this->center.y = std::stod(data.at(2));
        this->radius = std::stod(data.at(3));
        this->start_angle = std::stod(data.at(4));
        this->end_angle = std::stod(data.at(5));
        this->vel = std::stod(data.at(6)) * kmh;
      } catch (const std::exception& e) {
        std::cout << "in constructor of Arc:" << e.what() << std::endl;
      }
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
            bool is_hidarimawari = this->end_angle > this->start_angle;
            double th = angle + (is_hidarimawari ? M_PI / 2.0 : -M_PI / 2.0);
            th = std::remainder(th, 2 * M_PI);
            double vel = this->vel;
            double curvature = 1.0 / this->radius * ((is_hidarimawari) ? 1 : -1);
            path.push_back({t, x, y, th, vel, curvature, acc});
        }

        return path;
    }
};

class Clothoid : public Particle{
public:
    std::vector<PathPoint> dense_points;
    double du;
    double start_curvature, end_curvature;
    void generate_dense_points(double length, double du=0.0001){
        // length: クロソイドの長さ
        // du: クロソイドの分割の長さ
        this->dense_points.clear();
        double curvature_incresing_rate = (this->end_curvature - this->start_curvature) / length;
        double prev_curvature = this->start_curvature;
        PathPoint prev_point;
        for (double u = 0; u < length; u += du){
            double curvature = this->start_curvature + curvature_incresing_rate * u;
            double th = prev_point.th + prev_curvature * du;
            th = std::remainder(th, 2 * M_PI);
            double& k = curvature_incresing_rate;
            auto dp = PathPoint{};
            dp.x = prev_point.x + std::cos(k * u * u / 2.0) * du - 0.5 * k * u * std::sin(k * u * u / 2.0) * du * du;
            dp.y = prev_point.y + std::sin(k * u * u / 2.0) * du + 0.5 * k * u * std::cos(k * u * u / 2.0) * du * du;
            dp.th = th;
            dp.curvature = curvature;
            dp.heading_rate = prev_curvature;
            this->dense_points.push_back(dp);
            prev_point = dp;
            prev_curvature = curvature;
        }
        this->du = du;
    }
    Clothoid(std::vector<std::string> data) : Particle(){
        this->start_curvature = std::stod(data[1]);
        this->end_curvature = std::stod(data[2]);
        // this->generate_dense_points(1.0);
    }
    Clothoid() = default;
    std::vector<PathPoint> generate_path(double dt, double st=0) override {
        std::vector<PathPoint> path;
        int i = (int)(st * dense_points.at(0).vel / du);
        while (true){
            path.push_back(dense_points.at(i));
            i += (int)(dt * dense_points.at(i).vel / du);
            if (i >= dense_points.size()){
                break;
            }
        }
        return path;
    }
    void interpolate(std::shared_ptr<Line> line, std::shared_ptr<Arc> arc, double length, bool reverse=false){
        // 座標変換前のクロソイドを生成
        //    原点から+xの方向に進み、+yにちょっと曲がる
        start_curvature = 0.0;
        end_curvature = 1.0 / arc->radius;
        generate_dense_points(length);
        // XY curve_center = {dense_points.back().x, arc->radius};

        // st_angle : 座標変換前のクロソイドを考えたときに、クロソイドの始点がどの角度にあるか
        double st_angle;// = dense_points.back().th;
        st_angle = std::acos((arc->radius - dense_points.back().y) / arc->radius);

        // shorten_length : 直線がクロソイド補間によってどれだけ短くなるか
        double shorten_length = 0.0;
        double center_x = dense_points.back().x - std::sqrt(std::pow(arc->radius,2) - std::pow(arc-> radius - dense_points.back().y,2));
        shorten_length = center_x;

        bool is_hidarimawari = arc->end_angle > arc->start_angle;
        if (!is_hidarimawari ^ reverse) {
            for (auto& point : dense_points) {
                point.th *= -1;
                point.y *= -1;
            }
            st_angle = -st_angle;
        }

        // st_frame: クロソイドの始点の座標
        PathPoint st_frame;
        if (!reverse){
            st_frame.x = line->ed.x - shorten_length * std::cos(line->dir);
            st_frame.y = line->ed.y - shorten_length * std::sin(line->dir);
            st_frame.th = line->dir;
        }else{
            st_frame.x = line->st.x + shorten_length * std::cos(line->dir);
            st_frame.y = line->st.y + shorten_length * std::sin(line->dir);
            st_frame.th = line->dir + M_PI;
        }

        // 座標変換
        for (auto& point : dense_points) {
            point = st_frame + point;
        }

        // 情報を追加
        for (int i = 0; i < dense_points.size(); i++) {
            dense_points.at(i).vel = arc->vel;
            dense_points.at(i).acc = 0.0;
            dense_points.at(i).curvature = (end_curvature - start_curvature) * (i * du / length) + start_curvature;
            dense_points.at(i).curvature *= (!is_hidarimawari) ? -1 : 1;
            dense_points.at(i).heading_rate = dense_points.at(i).vel * dense_points.at(i).curvature; 
            if (i == 0){
                dense_points.at(i).time = 0.0;
            }else{
                dense_points.at(i).time = dense_points.at(i-1).time + du / dense_points.at(i-1).vel;
            }
        }
        this->path_time = dense_points.back().time;

        // Arc->Lineの場合
        if (reverse) {
            std::reverse(dense_points.begin(), dense_points.end());
            for (auto& point : dense_points) {
                point.time = path_time - point.time;
                point.th = std::remainder(point.th + M_PI, 2 * M_PI);
            }
        }

        //lineとarcを変更
        if (!reverse){
            line->ed.x = st_frame.x;
            line->ed.y = st_frame.y;
            arc->start_angle += st_angle;
        }else{
            line->st.x = st_frame.x;
            line->st.y = st_frame.y;
            arc->end_angle += st_angle;
        }
    }
};

class TrajectoryGenerator {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    double path_time=0;

    TrajectoryGenerator(std::string filepath) {
        try {
            std::ifstream file(filepath);
            std::string line;
            double clothoid_len = 0;
            while (std::getline(file, line)) {
                // std::cout << "line:" << line << std::endl;
                std::stringstream ss(line);
                std::vector<std::string> data;
                std::string token;
                while (std::getline(ss, token, ',')) {
                    data.push_back(token);
                }
                if (line.at(0) == 'L') {
                    auto l = std::make_shared<Line>(data);
                    if (clothoid_len > 0) {
                        Clothoid clothoid;
                        clothoid.interpolate(std::dynamic_pointer_cast<Line>(l), 
                            std::dynamic_pointer_cast<Arc>(particles.at(particles.size()-1)), clothoid_len, true);
                        particles.push_back(std::make_shared<Clothoid>(clothoid));
                        clothoid_len = 0;
                    }
                    particles.push_back(std::dynamic_pointer_cast<Particle>(l));
                } else if (line.at(0) == 'A') {
                    auto arc = std::make_shared<Arc>(data);
                    if (clothoid_len > 0) {
                        Clothoid clothoid;
                        clothoid.interpolate(std::dynamic_pointer_cast<Line>(particles.at(particles.size()-1)), 
                            std::dynamic_pointer_cast<Arc>(arc), clothoid_len, false);
                        particles.push_back(std::make_shared<Clothoid>(clothoid));
                        clothoid_len = 0;
                    }
                    particles.push_back(std::dynamic_pointer_cast<Particle>(arc));
                } else if (line.at(0) == 'C') {
                    clothoid_len = std::stod(data.at(1));
                }
            }
        } catch (const std::exception& e) {
            std::cout << "in constructor of PathTrajectory" << e.what() << std::endl;
        }
        std::cout << "size of particles = " << particles.size() << std::endl;
    }

    TrajectoryGenerator() = default;

    std::vector<PathPoint> generate_path(double dt) {
        std::vector<PathPoint> path = particles.at(0)->generate_path(dt);
        double time_offset = particles.at(0)->path_time;
        for (int i = 1; i < particles.size(); i++) {
            double st = dt - (time_offset - path.back().time);
            auto next_path = particles.at(i)->generate_path(dt, st);
            for (int j = 0; j < next_path.size(); j++) {
                next_path.at(j).time += time_offset;
                path.push_back(next_path.at(j));
            }
            time_offset += particles.at(i)->path_time;
        }
        path_time = time_offset;
        return path;
    }

    void write_path(std::string filepath, double dt = 0.01) {
        std::vector<PathPoint> path = generate_path(dt);
        std::ofstream outfile(filepath);
        outfile << "time,x,y,th,vel,curvature,acc" << std::endl;
        for (int i = 0; i < path.size(); i++) {
            outfile << path.at(i).time << "," << std::setprecision(15) << path.at(i).x << "," << path.at(i).y << "," << path.at(i).th << "," << path.at(i).vel << "," << path.at(i).curvature << "," << path.at(i).acc << std::endl;
        }
    }
};