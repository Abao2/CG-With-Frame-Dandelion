#include "formatter.hpp"
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

using Eigen::Vector2d;
using Eigen::Matrix2d;

int main(int argc, char **argv){

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log",true);
    auto my_logger = std::make_shared<spdlog::logger>("optimizer", spdlog::sinks_init_list{console_sink, file_sink});
    spdlog::register_logger(my_logger);
    spdlog::set_level(spdlog::level::debug);
    my_logger->set_level(spdlog::level::debug);
    Vector2d p0(double( 2223515459 % 827), double( 2223515459 % 1709));
    Vector2d pk = p0;
    Vector2d grad = p0;
    Matrix2d Hes = Matrix2d::Identity() * 2;
    Hes = Hes.inverse().eval();
    double lambda = 0.5;
    double delta = 0.01;
    double length = 0.0;
    do{
        p0 = pk;
        my_logger->debug("({},{})",p0.x(),p0.y());
        grad.x() = 2.0 * p0.x();
        grad.y() = 2.0 * p0.y();
        pk = p0 - lambda * Hes * grad;
        length = (pk - p0).norm();
    }while(length >= delta);
    my_logger->info(pow(p0.x(),2) + pow(p0.y(),2));
    return 0;
}
