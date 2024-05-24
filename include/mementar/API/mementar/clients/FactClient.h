#ifndef MEMENTAR_FACTCLIENT_H
#define MEMENTAR_FACTCLIENT_H

#include "mementar/API/mementar/clients/ClientBase.h"

namespace mementar
{

class FactClient : public ClientBase
{
public:
  FactClient(const std::string& name) : ClientBase((name == "") ? "fact" : "fact/" + name) {}

    bool exist(const std::string& fact_id);
    bool isActionPart(const std::string& fact_id);
    std::string getActionPart(const std::string& fact_id);
    std::string getData(const std::string& fact_id);
    compat::onto_ros::Time getStamp(const std::string& fact_id);
private:
};

} // namespace mementar

#endif // MEMENTAR_FACTCLIENT_H