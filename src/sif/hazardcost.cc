#include <sif/hazardcost.h>
#include <sif/autocost.h>
#include <sif/truckcost.h>
#include <sif/costfactory.h>
namespace valhalla {
namespace sif {
namespace
{
    float kheuristic_multiplier = 1.0f;
}

HazardCost::HazardCost(const Costing costing, const Options& options) 
: DynamicCost(options, TravelMode::kDrive)
{
   const CostingOptions& costing_options = options.costing_options(static_cast<int>(costing));
   sif::CostFactory<sif::DynamicCost> factory;
   factory.RegisterStandardCostingModels();
   Costing sub_costing;
   valhalla::Costing_Enum_Parse(costing_options.sub_costing_option() , &sub_costing);
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
    pbf_costing_options->set_heuristic_multiplier(
        rapidjson::get_optional<float>(*json_costing_options, "/heuristic_multiplier")
            .get_value_or(kheuristic_multiplier));
  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_sub_costing_option("auto");
     pbf_costing_options->set_heuristic_multiplier(kheuristic_multiplier);
  }
}


cost_ptr_t CreateHazardCost(const Costing costing, const Options& options) {
  return std::make_shared<HazardCost>(costing, options);
}

} // namespace sif
} // namespace valhalla