#include "configs.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    if(argc != 2) {
        std::cout << "usage: " << argv[0] << " <config file>" << std::endl;
        return -1;
    }

    Configs cfg;
    ConfigDef(cfg, int,         param1);
    ConfigDef(cfg, float,       param2);
    ConfigDef(cfg, double,      param3);
    ConfigDef(cfg, bool,        param4);
    ConfigDef(cfg, std::string, param5);
    cfg.Open(argv[1]);
    cfg.LoadOnce();

    std::cout << "param1: " << param1 << std::endl;
    std::cout << "param2: " << param2 << std::endl;
    std::cout << "param3: " << param3 << std::endl;
    std::cout << "param4: " << param4 << std::endl;
    std::cout << "param5: " << param5 << std::endl;
    
    return 0;
}
