#ifndef MEMENTAR_SUBSCRIPTION_H
#define MEMENTAR_SUBSCRIPTION_H

#include <map>
#include <mutex>
#include <vector>

#include "mementar/core/Occasions/IdManager.h"
#include "mementar/core/memGraphs/Branchs/types/TripletPattern.h"
#include "ontologenius/OntologyManipulator.h"

namespace mementar {

  class Subscription
  {
  public:
    Subscription(onto::OntologyManipulator* onto = nullptr) : onto_(onto) {}

    size_t subscribe(const TripletPattern& patern, size_t count);
    bool unsubscribe(int id);

    bool isFinished(size_t id);
    bool empty() { return triplet_paterns_.empty(); }

    std::vector<size_t> evaluate(const Triplet& triplet);

  private:
    std::map<size_t, TripletPattern> triplet_paterns_;
    std::map<size_t, size_t> counts_;
    std::mutex map_mut_;

    IdManager<size_t> id_manager_;

    onto::OntologyManipulator* onto_;

    TripletPattern refinePattern(const TripletPattern& triplet);
    bool compareToTriplet(const TripletPattern& pattern, const Triplet& triplet);
  };

} // namespace mementar

#endif // MEMENTAR_SUBSCRIPTION_H
