#ifndef MAPPING_VALUE_CONVERSION_TABLES_H_
#define MAPPING_VALUE_CONVERSION_TABLES_H_

#include <map>
#include <vector>

#include "common/port.h"
#include "glog/logging.h"

namespace mapping
{

// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.
class ValueConversionTables
{
    public:
      const std::vector<float> *GetConversionTable(float unknown_result,
                                                   float lower_bound,
                                                   float upper_bound);

    private:
      std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                                float /* upper_bound */>,
               std::unique_ptr<const std::vector<float>>>
          bounds_to_lookup_table_;
};

} // namespace mapping

#endif // MAPPING_VALUE_CONVERSION_TABLES_H_
