#include <cstdint>
#include "mca/hardware/ethercat_pdo_mapping_profile.hpp"

int main() {
  using Profile = mca::hardware::EthercatPdoMappingProfile<8U>;
  using Direction = mca::hardware::EthercatPdoDirection;
  Profile profile{};
  if (!profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output})) return 1;
  if (!profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x607aU, 0U, 32U, Direction::Output})) return 2;
  if (!profile.add({5U, 0x29cU, 0x03b11002U, 3U, 0x1a00U, 0x6041U, 0U, 16U, Direction::Input})) return 3;
  if (!profile.valid() || profile.size() != 3U) return 4;

  Profile duplicate{};
  if (!duplicate.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output})) return 5;
  if (duplicate.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output})) return 6;

  Profile wrong_order{};
  if (!wrong_order.add({5U, 0x29cU, 0x03b11002U, 3U, 0x1a00U, 0x6041U, 0U, 16U, Direction::Input})) return 7;
  if (wrong_order.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output})) return 8;

  Profile wrong_direction{};
  if (!wrong_direction.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output})) return 9;
  if (wrong_direction.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6041U, 0U, 16U, Direction::Input})) return 10;

  Profile bad_width{};
  if (bad_width.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 0U, Direction::Output})) return 11;
  return 0;
}
