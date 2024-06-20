#include "mementar/core/LtManagement/EpisodicTree/Context.h"

#include <cstddef>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/graphical/Display.h"

namespace mementar {

  void Context::insert(const Triplet* triplet)
  {
    std::map<std::string, size_t>::iterator it;

    it = subjects_.find(triplet->subject_);
    if(it != subjects_.end())
      it->second++;
    else
      subjects_[triplet->subject_] = 1;

    it = predicats_.find(triplet->predicat_);
    if(it != predicats_.end())
      it->second++;
    else
      predicats_[triplet->predicat_] = 1;

    if(triplet->object_.find(':') == std::string::npos)
    {
      it = objects_.find(triplet->object_);
      if(it != objects_.end())
        it->second++;
      else
        objects_[triplet->object_] = 1;
    }
  }

  void Context::remove(const Triplet* triplet)
  {
    std::map<std::string, size_t>::iterator it;

    it = subjects_.find(triplet->subject_);
    if(it != subjects_.end())
      it->second--;

    it = predicats_.find(triplet->predicat_);
    if(it != predicats_.end())
      it->second--;

    it = objects_.find(triplet->object_);
    if(it != objects_.end())
      it->second--;
  }

  bool Context::exist(const std::string& name)
  {
    if(subjects_.find(name) != subjects_.end())
      return true;
    else if(predicats_.find(name) != predicats_.end())
      return true;
    else if(objects_.find(name) != objects_.end())
      return true;
    else
      return true;
  }

  bool Context::subjectExist(const std::string& subject)
  {
    if(subjects_.find(subject) != subjects_.end())
      return true;
    return false;
  }

  bool Context::predicatExist(const std::string& predicat)
  {
    if(predicats_.find(predicat) != predicats_.end())
      return true;
    return false;
  }

  bool Context::objectExist(const std::string& object)
  {
    if(objects_.find(object) != objects_.end())
      return true;
    return false;
  }

  std::string Context::toString()
  {
    std::string res;
    res += "__SUBJECT__\n";
    for(const auto& it : subjects_)
      if(it.second != 0)
        res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

    res += "__PREDICAT__\n";
    for(const auto& it : predicats_)
      if(it.second != 0)
        res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

    res += "__OBJECT__\n";
    for(const auto& it : objects_)
      if(it.second != 0)
        res += "<" + std::to_string(it.second) + ">" + it.first + "\n";

    return res;
  }

  void Context::fromString(const std::string& string)
  {
    std::regex section("__(\\w+)__");
    std::regex context("<(\\d+)>(\\w+)");
    std::smatch match;

    std::map<std::string, size_t>* map_ptr = nullptr;

    std::istringstream iss(string);
    std::string line;
    while(std::getline(iss, line))
    {
      if(std::regex_match(line, match, section))
      {
        if(match[1].str() == "SUBJECT")
          map_ptr = &subjects_;
        else if(match[1].str() == "PREDICAT")
          map_ptr = &predicats_;
        else if(match[1].str() == "OBJECT")
          map_ptr = &objects_;
        else
          map_ptr = nullptr;
      }
      else if(std::regex_match(line, match, context))
      {
        if(map_ptr != nullptr)
        {
          size_t nb = 0;
          std::istringstream iss_match(match[1].str());
          iss_match >> nb;
          if(nb != 0)
            map_ptr->operator[](match[2].str()) = nb;
        }
      }
    }
  }

  void Context::storeContexts(std::vector<Context>& contexts, const std::string& directory)
  {
    Display::info("Save contexts:");

    Display::percent(0);
    std::string res;
    res += "[" + std::to_string(contexts.size()) + "]\n";
    for(size_t i = 0; i < contexts.size(); i++)
    {
      res += "{" + std::to_string(contexts[i].getKey()) + "}{\n";
      res += contexts[i].toString();
      res += "}\n";
      Display::percent((i + 1) * 100 / contexts.size());
    }

    std::ofstream file;
    file.open(directory + "/context.txt");
    file << res;
    file.close();

    Display::percent(100);
    Display::debug("");
  }

  std::string Context::contextsToString(std::vector<Context>& contexts)
  {
    Display::info("Convert contexts:");

    Display::percent(0);
    std::string res;
    res += "[" + std::to_string(contexts.size()) + "]\n";
    for(size_t i = 0; i < contexts.size(); i++)
    {
      res += "{" + std::to_string(contexts[i].getKey()) + "}{\n";
      res += contexts[i].toString();
      res += "}\n";
      Display::percent((i + 1) * 100 / contexts.size());
    }

    Display::percent(100);
    Display::debug("");

    return res;
  }

  void Context::loadContexts(std::vector<Context>& contexts, const std::string& directory)
  {
    // contexts must have a key
    Display::info("Load contexts:");
    std::ifstream t(directory + "/context.txt");
    if(t)
    {
      std::string str((std::istreambuf_iterator<char>(t)),
                      std::istreambuf_iterator<char>());

      size_t pose_start = 0;
      size_t pose_end = 0;

      pose_start = str.find('[') + 1;
      pose_end = str.find(']', pose_start);

      std::string tmp = str.substr(pose_start, pose_end - pose_start);
      size_t nb_contexts = 0;
      std::istringstream iss(tmp);
      iss >> nb_contexts;

      Display::percent(0);

      for(size_t i = 0; i < nb_contexts; i++)
      {
        pose_start = str.find('{', pose_end) + 1;
        pose_end = str.find('}', pose_start);
        tmp = str.substr(pose_start, pose_end - pose_start);
        time_t key = 0;
        iss = std::istringstream(tmp);
        iss >> key;

        pose_start = str.find('{', pose_end) + 1;
        pose_end = str.find('}', pose_start);
        tmp = str.substr(pose_start, pose_end - pose_start);

        for(auto& context : contexts)
        {
          if(key == context.getKey())
          {
            context.fromString(tmp);
            break;
          }
        }

        Display::percent((i + 1) * 100 / nb_contexts);
      }
    }

    Display::percent(100);
    Display::debug("");
  }

  std::vector<Context> Context::stringToContext(const std::string& str)
  {
    std::vector<Context> contexts;

    size_t pose_start = 0;
    size_t pose_end = 0;

    pose_start = str.find('[') + 1;
    pose_end = str.find(']', pose_start);

    std::string tmp = str.substr(pose_start, pose_end - pose_start);
    size_t nb_contexts = 0;
    std::istringstream iss(tmp);
    iss >> nb_contexts;

    for(size_t i = 0; i < nb_contexts; i++)
    {
      pose_start = str.find('{', pose_end) + 1;
      pose_end = str.find('}', pose_start);
      tmp = str.substr(pose_start, pose_end - pose_start);
      time_t key = 0;
      iss = std::istringstream(tmp);
      iss >> key;

      pose_start = str.find('{', pose_end) + 1;
      pose_end = str.find('}', pose_start);
      tmp = str.substr(pose_start, pose_end - pose_start);

      contexts.emplace_back(key);
      contexts[contexts.size() - 1].fromString(tmp);
    }

    return contexts;
  }

} // namespace mementar
