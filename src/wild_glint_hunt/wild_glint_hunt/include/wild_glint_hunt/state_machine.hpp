#pragma once

#include <string>

namespace wild_glint_hunt
{

enum class HuntState
{
  INIT = 0,
  FOLLOW_ROUTE = 1,
  ALIGN = 2,
  STRIKE = 3,
  VERIFY = 4,
  EXIT = 5,
  FINISH = 6,
  ERROR = 7,
  SEARCH_ALL = 8,
  EXECUTE_PLAN = 9,
  SEARCH = 10
};

inline std::string to_string(HuntState state)
{
  switch (state) {
    case HuntState::INIT: return "S0_INIT";
    case HuntState::FOLLOW_ROUTE: return "S1_FOLLOW_ROUTE";
    case HuntState::ALIGN: return "S2_ALIGN";
    case HuntState::STRIKE: return "S3_STRIKE";
    case HuntState::VERIFY: return "S4_VERIFY";
    case HuntState::EXIT: return "S5_EXIT";
    case HuntState::FINISH: return "S6_FINISH";
    case HuntState::SEARCH_ALL: return "S_SEARCH_ALL_LEGACY";
    case HuntState::EXECUTE_PLAN: return "S_EXECUTE_PLAN_LEGACY";
    case HuntState::SEARCH: return "S_SEARCH_LEGACY";
    default: return "SERR_ERROR";
  }
}

}  // namespace wild_glint_hunt
