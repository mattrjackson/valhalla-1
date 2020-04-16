#include <sif/hazardcost.h>
#include <sif/autocost.h>
#include <sif/truckcost.h>
#include <sif/costfactory.h>
namespace valhalla {
namespace sif {
/**
 * Derived class providing dynamic edge costing for vehicles minimizing an arbitrary hazard.
 */
class HazardCost : public DynamicCost {
public:
    std::function<double(uint32_t, const baldr::DirectedEdge* , const baldr::GraphTile* , const uint32_t, const float, const bool)> ComputeEdgeCostF = 0;    
    std::function<double(uint32_t, const baldr::GraphId id, const uint32_t start, const float end, const bool)> ComputeNodeCostF = 0;
  /**
   * Construct truck costing. Pass in cost type and options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  options pbf with request options.
   */
  HazardCost(const Costing costing, const Options& options);

  virtual ~HazardCost();


  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const
  {
    return cost_ -> AllowMultiPass();
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const
  {
    return cost_ -> access_mode();
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       bool& time_restricted) const
  {
    return cost_ -> Allowed(edge, pred, tile, edgeid, current_time, tz_index, time_restricted);
  }

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const
  {
    return AllowedReverse(edge, pred, opp_edge, tile, opp_edgeid, current_time, tz_index, has_time_restrictions);
  }                              

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const
  {
    return cost_ -> Allowed(node);
  }

  /**
   * Callback for Allowed doing mode  specific restriction checks
   */
  virtual bool ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const
  {
    return ModeSpecificAllowed(restriction);
  }

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const {
    auto cost = cost_ -> EdgeCost(edge, departure, curr_time);
    if(ComputeEdgeCostF == 0 )
      return cost;
  }
    /**
   * Get the cost to traverse the specified directed edge for a reverse search. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge    Pointer to a directed edge.
   * @param   tile    Pointer to the tile which contains the directed edge for speed lookup
   * @param   seconds Seconds of week for predicted speed or free and constrained speed lookup
   * @return  Returns the cost and time (seconds).
   */
  virtual Cost EdgeCostReverse(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const
  {
    auto cost = cost_ ->EdgeCostReverse(edge, tile, seconds);
    if(ComputeEdgeCostF == 0 )
      return cost;
  }
  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const
  {
    auto cost = cost_ -> EdgeCost(edge, tile, seconds);
    if(ComputeEdgeCostF == 0 )
      return cost;
  }                        

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const
  {
    auto cost = cost_ ->TransitionCost(edge, node, pred);
    if(ComputeNodeCostF == 0 )
      return cost;
  }                              

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge) const
  {
    auto cost = cost_ -> TransitionCostReverse(idx, node, pred, edge);
    if(ComputeNodeCostF == 0 )
      return cost;
  }

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const
  {
    return 0; // without making change to A* heuristic algorithm, only way to ensure we get optimal path is to revert to dijkstra.  
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const
  {
    return cost_ -> travel_type();
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by truck.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    return cost_ -> GetEdgeFilter();
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   * @return Function/functor to be used in filtering out nodes
   */
  virtual const NodeFilter GetNodeFilter() const {
    // throw back a lambda that checks the access for this type of costing
    return cost_ -> GetNodeFilter();
  }


private:
  cost_ptr_t cost_; // Vehicle type: tractor trailer
 
};

HazardCost::HazardCost(const Costing costing, const Options& options) 
: DynamicCost(options, TravelMode::kDrive)
{
   sif::CostFactory<sif::DynamicCost> factory;
   factory.RegisterStandardCostingModels();
   Costing sub_costing;
   valhalla::Costing_Enum_Parse(0 , &sub_costing);
   cost_ = factory.Create(sub_costing, options);
}

void ParseHazardCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           CostingOptions* pbf_costing_options) {
  auto json_costing_options = rapidjson::get_child_optional(doc, costing_options_key.c_str());

  if (json_costing_options) {
    // TODO: farm more common stuff out to parent class
    ParseCostOptions(*json_costing_options, pbf_costing_options);

    // If specified, parse json and set pbf values
    pbf_costing_options->set_sub_costing_option(
        rapidjson::get_optional<std::string>(*json_costing_options, "/sub_costing_option")
            .get_value_or("auto"));
  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_sub_costing_option("auto");
  }
}


cost_ptr_t CreateHazardCost(const Costing costing, const Options& options) {
  return std::make_shared<HazardCost>(costing, options);
}

} // namespace sif
} // namespace valhalla