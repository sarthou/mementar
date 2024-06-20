#ifndef MEMENTAR_CONFIGURATION_H
#define MEMENTAR_CONFIGURATION_H

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mementar {

  class ConfigElement
  {
  public:
    std::optional<std::vector<std::string>> data;
    std::optional<std::map<std::string, ConfigElement>> subelem;

    ConfigElement operator[](const std::string& name)
    {
      if(subelem)
        return subelem.value()[name];
      else
        return ConfigElement();
    }

    bool exist(const std::string& name)
    {
      if(subelem)
        return (subelem.value().find(name) != subelem.value().end());
      else
        return false;
    }

    std::vector<std::string> value()
    {
      if(data)
        return data.value();
      else
        return {};
    }

    void push_back(const std::string& value)
    {
      data.value().push_back(value);
    }
  };

  class Configuration
  {
  public:
    std::map<std::string, ConfigElement> config_;

    bool read(const std::string& path);
    bool write(const std::string& path);

    void display();

    ConfigElement& operator[](const std::string& name)
    {
      return config_[name];
    }

    bool exist(const std::string& name)
    {
      return (config_.find(name) != config_.end());
    }

  private:
    void display(std::map<std::string, ConfigElement>& config, size_t nb = 0);
    void displayTab(size_t nb);

    std::string getConfig(std::map<std::string, ConfigElement>& config, size_t nb = 0);
    std::string getTabs(size_t nb);

    void removeComment(std::string& line);
  };

} // namespace mementar

#endif // MEMENTAR_CONFIGURATION_H
