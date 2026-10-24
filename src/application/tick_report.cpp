#include "px4_backtrack/application/tick_report.hpp"

#include <iomanip>
#include <sstream>

#include "px4_backtrack/infrastructure/debug_log.hpp"

namespace px4_backtrack {
namespace application {

using infrastructure::DebugLog;

std::string statusJson(double t, const TickOutput& out) {
  std::ostringstream json;
  json << std::fixed << std::setprecision(3) << "{\"t\":" << t
       << ",\"flight\":" << out.flight_id
       << ",\"state\":" << DebugLog::jsonString(out.state)
       << ",\"control\":" << DebugLog::jsonString(domain::controlName(out.control))
       << ",\"mission\":" << out.mission_id
       << ",\"reason\":" << DebugLog::jsonString(domain::missionReasonName(out.mission_reason))
       << ",\"mode_request\":" << DebugLog::jsonString(out.mode_status.mode)
       << ",\"mode_result\":"
       << DebugLog::jsonString(modeRequestResultName(out.mode_status.result))
       << ",\"alerts\":[";
  const char* separator = "";
  for (int i = 0; i < static_cast<int>(domain::Alert::Count); ++i) {
    const auto alert = static_cast<domain::Alert>(i);
    if (out.alerts.has(alert)) {
      json << separator << DebugLog::jsonString(domain::alertName(alert));
      separator = ",";
    }
  }
  json << "]}";
  return json.str();
}

}  // namespace application
}  // namespace px4_backtrack
