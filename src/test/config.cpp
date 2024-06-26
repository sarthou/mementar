#include <iostream>
#include <string>

#include "mementar/compat/ros.h"
#include "mementar/core/Parametrization/Configuration.h"

int main()
{
  mementar::Configuration config;
  std::string path_base = mementar::compat::mem_ros::getShareDirectory("mementar");
  bool ok = config.read(path_base + "/files/config_example.yaml");
  if(ok)
  {
    if(config.exist("whitelist"))
    {
      config["whitelist"].pushBack("new property");
      config.display();
      ok = config.write(path_base + "/file_intern/new_configuration.yaml");
      if(!ok)
        std::cout << "fail to write the new configuation file: " << path_base << "/file_intern/new_configuration.yaml" << std::endl;
    }
    else
      std::cout << "whitelist not found" << std::endl;
  }
  else
    std::cout << "fail to read configuation file: " << path_base << "/files/config_example.yaml" << std::endl;
  return 0;
}
