#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include <functional>
#include <string>
#include <unordered_map>

struct ConfigsError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct ConfigAddError : ConfigsError {
  using ConfigsError::ConfigsError;
};

struct ConfigLoadError : ConfigsError {
  using ConfigsError::ConfigsError;
};

class ConfigLoader {
 public:
  ConfigLoader() = default;

  explicit ConfigLoader(const std::string& file);

  template <typename T> void add(std::string name, T* ptr);

  void load_once() const;

  void open(const std::string& file);

 private:
  YAML::Node node_;

  std::unordered_map<std::string, std::function<void(const YAML::Node&)>> configs_;
};

template <typename T> void ConfigLoader::add(std::string name, T* ptr) {
  if (configs_.count(name))
    throw ConfigAddError("'" + name + "' deplicate");
  configs_.emplace(name, [ptr](const YAML::Node& node) { *ptr = node.as<T>(); });
}

#define ConfigAdd(var) add(#var, &(var))

#define ConfigDef(cfg, type, var) \
  type var;                       \
  (cfg).ConfigAdd(var)
