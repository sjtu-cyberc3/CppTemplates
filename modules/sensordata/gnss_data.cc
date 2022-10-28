#include "gnss_data.h"

std::ostream& operator<<(std::ostream& os, const GNSSData& data) {
  os << std::fixed << data.time << " " << data.longitude << " " << data.latitude << " " << data.altitude << " " << data.status << " " << data.service
     << " " << data.local_E << " " << data.local_N << " " << data.local_U << std::endl;
  return os;
}

std::istream& operator>>(std::istream& in, GNSSData& this_data) {
  in >> this_data.time >> this_data.longitude >> this_data.latitude >> this_data.altitude >> this_data.status >> this_data.service
    >> this_data.local_E >> this_data.local_N >> this_data.local_U;
  return in;
}