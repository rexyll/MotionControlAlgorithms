#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/ethercat_adapter_state.hpp"

#if defined(MCA_ENABLE_IGH)
#include <ecrt.h>
#endif

namespace mca::hardware {

#if defined(MCA_ENABLE_IGH)

template <std::size_t MaxSlaves, std::size_t MaxDomains, std::size_t MaxEntries>
class EthercatIghAdapter final {
 public:
  using State = EthercatAdapterState<MaxSlaves, MaxDomains, MaxEntries>;
  using Topology = typename State::Topology;
  using Layout = typename State::Layout;

  struct Config {
    unsigned int master_index{0U};
    std::uint32_t cycle_time_ns{0U};
    std::uint32_t send_interval_us{0U};
  };

  static constexpr std::size_t kMaxSdoBytes = 256U;
  using SdoProfile = EthercatSdoProfile<MaxEntries, kMaxSdoBytes>;
  using SdoRequest = EthercatSdoRequest<kMaxSdoBytes>;
  using SdoResult = EthercatSdoResult<kMaxSdoBytes>;

  struct PdoBinding {
    std::uint16_t station_position{0U};
    std::uint32_t vendor_id{0U};
    std::uint32_t product_code{0U};
    std::uint16_t index{0U};
    std::uint8_t subindex{0U};
    std::uint8_t bit_length{0U};
    std::uint16_t domain_id{0U};
    EthercatPdoDirection direction{EthercatPdoDirection::Input};
  };

  explicit EthercatIghAdapter(const Config& config = {}) noexcept
      : config_(config) {}
  ~EthercatIghAdapter() noexcept { shutdown(); }

  EthercatIghAdapter(const EthercatIghAdapter&) = delete;
  EthercatIghAdapter& operator=(const EthercatIghAdapter&) = delete;

  [[nodiscard]] const State& state() const noexcept { return state_; }
  [[nodiscard]] const Topology& topology() const noexcept { return state_.topology(); }
  [[nodiscard]] const Layout& layout() const noexcept { return state_.layout(); }
  [[nodiscard]] std::uint64_t cycle_sequence() const noexcept { return cycle_sequence_; }
  [[nodiscard]] std::uint16_t observed_wkc() const noexcept { return observed_wkc_; }
  // IgH exposes WC completeness but not the configured expected counter value.
  // A zero expected value therefore means 'not exposed', never 'zero expected'.
  [[nodiscard]] std::uint16_t expected_wkc() const noexcept { return 0U; }
  [[nodiscard]] std::uint16_t domain_working_counter(std::uint16_t domain_id) const noexcept;
  [[nodiscard]] bool domain_working_counter_complete(std::uint16_t domain_id) const noexcept;

  mca::core::Status request_master() noexcept;
  mca::core::Status discover() noexcept;
  mca::core::Status configure(const std::array<PdoBinding, MaxEntries>& bindings,
                              std::size_t binding_count) noexcept;
  // Must be configured after PDO layout and before master activation.
  mca::core::Status configure_sdo_profile(const SdoProfile& profile) noexcept;
  mca::core::Status enqueue_sdo(const SdoRequest& request) noexcept;
  // now_timestamp_ns must use the same monotonic clock domain as request deadlines.
  mca::core::Status poll_sdo(std::uint64_t request_id, std::uint64_t now_timestamp_ns,
                             SdoResult& output) noexcept;
  mca::core::Status activate() noexcept;
  mca::core::Status exchange() noexcept;
  mca::core::Status read_pdo(std::size_t binding_index, std::uint64_t& value) const noexcept;
  mca::core::Status write_pdo(std::size_t binding_index, std::uint64_t value) noexcept;
  mca::core::Status shutdown() noexcept;

 private:
  static EthercatAlState map_al_state(std::uint8_t state) noexcept;
  [[nodiscard]] std::size_t domain_index(std::uint16_t domain_id) const noexcept;

  struct SdoSlotRuntime {
    ec_sdo_request_t* request{nullptr};
    std::uint64_t request_id{0U};
    bool occupied{false};
    bool timed_out{false};
    EthercatSdoOperation operation{EthercatSdoOperation::Read};
    std::size_t max_response_size{0U};
  };

  Config config_{};
  State state_{};
  ec_master_t* master_{nullptr};
  std::array<ec_domain_t*, MaxDomains> domains_{};
  std::array<ec_slave_config_t*, MaxSlaves> slave_configs_{};
  std::array<std::uint8_t*, MaxDomains> domain_data_{};
  std::array<std::size_t, MaxDomains> domain_size_{};
  std::array<unsigned int, MaxEntries> offsets_{};
  std::array<unsigned int, MaxEntries> bit_positions_{};
  std::array<std::uint16_t, MaxDomains> domain_wkc_{};
  std::array<bool, MaxDomains> domain_wc_complete_{};
  SdoProfile sdo_profile_{};
  std::array<SdoSlotRuntime, MaxEntries> sdo_slots_{};
  std::array<std::uint64_t, MaxEntries> slot_deadlines_{};
  std::size_t sdo_slot_count_{0U};
  std::size_t domain_count_{0U};
  std::size_t binding_count_{0U};
  std::uint16_t observed_wkc_{0U};
  std::uint64_t cycle_sequence_{0U};
  bool activated_{false};
};

#endif  // MCA_ENABLE_IGH

}  // namespace mca::hardware
