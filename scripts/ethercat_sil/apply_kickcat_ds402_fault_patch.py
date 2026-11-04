#!/usr/bin/env python3
"""Apply the MCA deterministic DS402 fault fixture to one pinned KickCAT checkout."""
from __future__ import annotations

import pathlib
import sys

PINNED_REVISION = "cb2e1a192ac11ffaa58f08945053b8129abf94cb"


def replace_once(source: str, needle: str, replacement: str, label: str) -> str:
    count = source.count(needle)
    if count != 1:
        raise RuntimeError(f"{label}: expected one exact insertion anchor, found {count}")
    return source.replace(needle, replacement, 1)


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {sys.argv[0]} KICKCAT_SOURCE_DIR")

    root = pathlib.Path(sys.argv[1]).resolve()
    header_path = root / "lib/simulation/include/kickcat/simulation/devices/Ds402Motor.h"
    app_path = root / "lib/simulation/src/DeviceApp.cc"
    if not header_path.is_file() or not app_path.is_file():
        raise SystemExit(f"unexpected KickCAT source layout under {root}")

    import subprocess

    revision = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != PINNED_REVISION:
        raise SystemExit(
            f"KickCAT revision mismatch: expected {PINNED_REVISION}, got {revision}"
        )

    header = header_path.read_text(encoding="utf-8")
    app = app_path.read_text(encoding="utf-8")
    header_marked = "fault_after_cycles" in header
    app_marked = "fault_after_cycles" in app and "fault_reaction_cycles" in app
    if header_marked and app_marked:
        old_counter = """            last_control_word_ = control_word;
            if (not fault_injected_ and status_word_value_ == OPERATION_ENABLED
                and (control_word & 0x000FU) == 0x000FU)
            {
                ++cycle_count_;
            }

            if (not fault_injected_ and params_.fault_after_cycles > 0U
"""
        new_counter = """            last_control_word_ = control_word;
            if (not fault_injected_ and status_word_value_ == OPERATION_ENABLED
                and (control_word & 0x000FU) == 0x000FU)
            {
                ++cycle_count_;
            }

            if (not fault_injected_ and params_.fault_after_cycles > 0U
"""
        if old_counter in header:
            header = header.replace(old_counter, new_counter, 1)
            header_path.write_text(header, encoding="utf-8")
            print(
                "PASS: upgraded DS402 fault counter to count only Operation Enabled cycles "
                f"at {revision}"
            )
            return 0
        if new_counter in header:
            print(f"KickCAT DS402 fault fixture already applied (enabled-cycle semantics) at {revision}")
            return 0
        raise SystemExit("unknown existing KickCAT fault fixture version; restore the pinned clean checkout")
    if header_marked or app_marked:
        raise SystemExit("partial KickCAT fault patch detected; restore the pinned clean checkout")

    header = replace_once(
        header,
        "            double max_velocity  = 0.0;     // ticks/s clamp, 0 = unlimited\n",
        "            double max_velocity  = 0.0;     // ticks/s clamp, 0 = unlimited\n"
        "            // Deterministic one-shot DS402 fault injection. Zero disables it.\n"
        "            std::uint32_t fault_after_cycles = 0U;\n"
        "            std::uint32_t fault_reaction_cycles = 3U;\n",
        "Ds402Motor::Params",
    )
    header = replace_once(
        header,
        "        static constexpr uint16_t OPERATION_ENABLED  = SWITCHED_ON | ds402::status::masks::OPERATION_ENABLE;\n",
        "        static constexpr uint16_t OPERATION_ENABLED  = SWITCHED_ON | ds402::status::masks::OPERATION_ENABLE;\n"
        "        static constexpr uint16_t FAULT_REACTION_ACTIVE = 0x000FU;\n"
        "        static constexpr uint16_t FAULT = ds402::status::value::FAULT_STATE;\n",
        "Ds402Motor state constants",
    )

    fault_logic = """            const bool fault_reset_edge = (control_word & 0x0080U) != 0U
                                       and (last_control_word_ & 0x0080U) == 0U;
            last_control_word_ = control_word;
            ++cycle_count_;

            if (not fault_injected_ and params_.fault_after_cycles > 0U
                and cycle_count_ >= params_.fault_after_cycles)
            {
                fault_injected_ = true;
                fault_reaction_active_ = true;
                fault_reaction_remaining_ = std::max<std::uint32_t>(
                    1U, params_.fault_reaction_cycles);
            }

            // Fault Reaction Active lasts for the configured cycles, then Fault
            // remains latched until a fresh controlword bit-7 rising edge.
            if (fault_reaction_active_)
            {
                status_word_value_ = FAULT_REACTION_ACTIVE;
                if (fault_reaction_remaining_ > 0U)
                {
                    --fault_reaction_remaining_;
                }
                if (fault_reaction_remaining_ == 0U)
                {
                    fault_reaction_active_ = false;
                    fault_latched_ = true;
                }
                return;
            }
            if (fault_latched_)
            {
                if (fault_reset_edge)
                {
                    fault_latched_ = false;
                    status_word_value_ = SWITCH_ON_DISABLED;
                }
                else
                {
                    status_word_value_ = FAULT;
                }
                return;
            }

"""
    header = replace_once(
        header,
        "        void stepStateMachine(uint16_t control_word)\n        {\n",
        "        void stepStateMachine(uint16_t control_word)\n        {\n" + fault_logic,
        "DS402 state-machine body",
    )
    header = replace_once(
        header,
        "        uint16_t      status_word_value_{SWITCH_ON_DISABLED};\n",
        "        uint16_t      status_word_value_{SWITCH_ON_DISABLED};\n"
        "        std::uint32_t cycle_count_{0U};\n"
        "        std::uint32_t fault_reaction_remaining_{0U};\n"
        "        std::uint16_t last_control_word_{0U};\n"
        "        bool          fault_injected_{false};\n"
        "        bool          fault_reaction_active_{false};\n"
        "        bool          fault_latched_{false};\n",
        "DS402 fault state members",
    )

    app = replace_once(
        app,
        "            mp.max_velocity  = config.value(\"motor_max_velocity\", mp.max_velocity);\n",
        "            mp.max_velocity  = config.value(\"motor_max_velocity\", mp.max_velocity);\n"
        "            mp.fault_after_cycles = config.value(\"fault_after_cycles\", mp.fault_after_cycles);\n"
        "            mp.fault_reaction_cycles = config.value(\"fault_reaction_cycles\", mp.fault_reaction_cycles);\n",
        "DeviceApp JSON parameters",
    )

    header_path.write_text(header, encoding="utf-8")
    app_path.write_text(app, encoding="utf-8")
    print(f"PASS: applied MCA DS402 fault fixture to KickCAT {revision}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
