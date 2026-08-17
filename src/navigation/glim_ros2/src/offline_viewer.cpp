#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <boost/program_options.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/viewer/offline_viewer.hpp>
#include <glim_ros/map_frame_normalization.hpp>

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description desc("GLIM offline viewer");
  desc.add_options()                                                               //
    ("help", "produce help message")                                               //
    ("map_path", value<std::string>(), "Input map path (dump directory)")          //
    ("export_path", value<std::string>(), "Export map points to a PLY file and exit")  //
    ("config_path", value<std::string>()->default_value("config"), "Config path")  //
    ("debug", "Enable debug printing")                                             //
    ;

  positional_options_description po;
  po.add("map_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(desc).positional(po).run(), vm);
  notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(glim::get_ringbuffer_sink());
  if (vm.count("debug")) {
    logger->sinks().push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/offline_viewer_log.log", true));
    logger->set_level(spdlog::level::trace);
  }

  spdlog::set_default_logger(logger);

  std::string config_path = vm["config_path"].as<std::string>();
  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);

  std::string init_map_path;
  if (vm.count("map_path")) {
    init_map_path = vm["map_path"].as<std::string>();
    spdlog::info("map_path={}", init_map_path);
  }

  std::string export_map_path;
  if (vm.count("export_path")) {
    if (init_map_path.empty()) {
      std::cerr << "--export_path requires map_path" << std::endl;
      return 1;
    }
    export_map_path = vm["export_path"].as<std::string>();
    spdlog::info("export_path={}", export_map_path);
  }

  Eigen::Isometry3d T_export_world = Eigen::Isometry3d::Identity();
  if (const auto normalization = glim::loadMapFrameNormalization(init_map_path)) {
    T_export_world = *normalization;
    spdlog::info("Applying map frame normalization from map dump");
  } else if (!export_map_path.empty()) {
    spdlog::error("Map dump has no valid map_frame_normalization.txt");
    return 1;
  }

  glim::OfflineViewer viewer(init_map_path, export_map_path, T_export_world);
  viewer.wait();
}
