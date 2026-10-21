// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnss_comm/gnss_utility.hpp>
#include <cstdint>
#include <optional>

namespace vins_adapter {

// gnss_comm serializes absolute pulse times using GPS week numbering for every
// time scale. T_SYS_* identifies the clock scale, not a different week epoch.
inline std::optional<gnss_comm::gtime_t> ToGpsPulseTime(
    gnss_comm::gtime_t pulse, std::uint32_t time_system, bool utc_based) {
    using namespace gnss_comm;
    if (utc_based || time_system == T_SYS_UTC) return utc2gpst(pulse);
    switch (time_system) {
    case T_SYS_GPS:
    case T_SYS_GAL:
    case T_SYS_QZS:
        return pulse;
    case T_SYS_BDS:
        return time_add(pulse, 14.0);
    case T_SYS_GLO:
        return utc2gpst(time_add(pulse, -3.0 * 3600.0));
    default:
        return std::nullopt;
    }
}

} // namespace vins_adapter
