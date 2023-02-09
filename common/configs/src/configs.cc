#include "common/configs.hpp"
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/parse.h>

ConfigLoader::ConfigLoader(const std::string& file) : node_(YAML::LoadFile(file)) {}

void ConfigLoader::load_once() const {
  for (const auto& cfg : configs_) {
    const auto& name   = cfg.first;
    const auto& setter = cfg.second;
    try {
      setter(node_[name]);
    } catch (YAML::Exception& e) {
      throw ConfigLoadError("'" + name + "' load fail: " + e.what());
    }
  }
}

void ConfigLoader::open(const std::string& file) {
  node_ = YAML::LoadFile(file);
}
