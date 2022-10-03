#pragma once
#include <stdexcept>
#ifndef CONFIGS_HPP
#define CONFIGS_HPP

#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>
#include <functional>

struct ConfigsError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct ConfigAddError : ConfigsError {
    using ConfigsError::ConfigsError;
};

struct ConfigLoadError : ConfigsError {
    using ConfigsError::ConfigsError;
};

class Configs {
public:
    Configs() = default;

    Configs(std::string file);

    template<typename T>
    void Add(std::string name, T *ptr);

    void LoadOnce() const;

    void Open(std::string file);

private:
    YAML::Node node_;
    std::unordered_map<std::string, std::function<void(const YAML::Node&)>> configs_;
};

template<typename T>
void Configs::Add(std::string name, T *ptr) {
    if(configs_.count(name)) throw ConfigAddError("'" + name + "' deplicate");
    configs_.emplace(name, [ptr](const YAML::Node& node){
        *ptr = node.as<T>();
    });
}

#define ConfigAdd(var)      Add(#var, &var)

#define ConfigDef(cfg, type, var) type var; cfg.ConfigAdd(var)

#endif /* CONFIGS_HPP */
