#include <ecrt.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct MasterGuard {
    ec_master_t* master{nullptr};

    ~MasterGuard() {
        if (master != nullptr) {
            ecrt_release_master(master);
        }
    }
};

bool require_preop(ec_master_t* master,
                   std::ofstream& raw,
                   std::uint16_t ring,
                   const char* stage) {
    ec_slave_info_t info{};
    const int status = ecrt_master_get_slave(master, ring, &info);
    if (status != 0) {
        raw << "STATE_CHECK position=" << ring << " stage=" << stage
            << " query_status=" << status << "\n";
        std::cerr << "could not query slave state before/after SDO at ring "
                  << ring << " stage=" << stage << "\n";
        return false;
    }

    const auto al_state = static_cast<unsigned int>(info.al_state & 0x0FU);
    raw << "STATE_CHECK position=" << ring << " stage=" << stage
        << " al_state=0x" << std::hex << std::uppercase << al_state
        << std::dec << "\n";
    if (al_state != EC_AL_STATE_PREOP) {
        std::cerr << "slave left PREOP during CoE readback: ring=" << ring
                  << " stage=" << stage << " al_state=0x"
                  << std::hex << std::uppercase << al_state << std::dec
                  << "\n";
        return false;
    }
    return true;
}

struct ReadResult {
    std::uint32_t value{0};
    std::size_t size{0};
};

bool read_sdo(ec_master_t* master,
              std::ofstream& raw,
              std::uint16_t ring,
              std::uint16_t index,
              std::uint8_t subindex,
              std::size_t width,
              ReadResult& result) {
    result.value = 0;
    result.size = 0;
    if (!require_preop(master, raw, ring, "before_sdo")) {
        return false;
    }
    std::array<std::uint8_t, 4> bytes{};
    std::uint32_t abort_code = 0;
    std::size_t result_size = 0;
    const auto started = std::chrono::steady_clock::now();

    const int status = ecrt_master_sdo_upload(
        master, ring, index, subindex, bytes.data(), width, &result_size,
        &abort_code);

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started).count();

    raw << "READ position=" << ring << " 0x" << std::hex << std::uppercase
        << index << ":" << static_cast<unsigned int>(subindex) << std::dec
        << " width=" << width << " exit=" << status
        << " elapsed_ms=" << elapsed << "\n";

    if (status != 0 || result_size != width) {
        raw << "result_size=" << result_size << " abort_code=0x"
            << std::hex << std::uppercase << abort_code << std::dec << "\n";
        std::cerr << "SDO read failed: ring=" << ring << " 0x"
                  << std::hex << std::uppercase << index << ":"
                  << static_cast<unsigned int>(subindex) << std::dec
                  << " status=" << status << " result_size=" << result_size
                  << " abort_code=0x" << std::hex << abort_code << std::dec
                  << "\n";
        return false;
    }

    if (!require_preop(master, raw, ring, "after_sdo")) {
        return false;
    }
    for (std::size_t i = 0; i < width; ++i) {
        result.value |= static_cast<std::uint32_t>(bytes[i]) << (8U * i);
    }
    result.size = result_size;

    raw << "0x" << std::hex << std::uppercase << result.value << std::dec
        << " " << result.value << "\n";
    return true;
}

bool append_row(std::ofstream& rows, char kind, const std::string& fields) {
    rows << kind << '\t' << fields << '\n';
    return rows.good();
}

bool verify_assignment(ec_master_t* master,
                       std::ofstream& rows,
                       std::ofstream& raw,
                       std::uint16_t ring,
                       std::uint16_t assignment,
                       const char* label) {
    ReadResult read{};
    if (!read_sdo(master, raw, ring, assignment, 0, 1, read)) {
        return false;
    }
    const std::uint32_t pdo_count = read.value;
    if (pdo_count == 0 || pdo_count > 16) {
        std::cerr << label << " assignment count out of range: "
                  << pdo_count << "\n";
        return false;
    }
    if (!append_row(rows, 'C', std::to_string(ring) + "\t" +
                    std::to_string(assignment) + "\t" +
                    std::to_string(pdo_count))) {
        return false;
    }

    std::cout << label << " assignment 0x" << std::hex << std::uppercase
              << assignment << std::dec << ": " << pdo_count << " PDO(s)\n";

    for (std::uint32_t pdo_position = 1; pdo_position <= pdo_count;
         ++pdo_position) {
        if (!read_sdo(master, raw, ring, assignment,
                      static_cast<std::uint8_t>(pdo_position), 2, read)) {
            return false;
        }
        const auto pdo_index = static_cast<std::uint16_t>(read.value);
        if (pdo_index == 0) {
            std::cerr << label << " assignment contains zero PDO index at "
                      << pdo_position << "\n";
            return false;
        }
        if (!append_row(rows, 'A', std::to_string(ring) + "\t" +
                        std::to_string(assignment) + "\t" +
                        std::to_string(pdo_position) + "\t" +
                        std::to_string(pdo_index))) {
            return false;
        }

        if (!read_sdo(master, raw, ring, pdo_index, 0, 1, read)) {
            return false;
        }
        const std::uint32_t entry_count = read.value;
        if (entry_count == 0 || entry_count > 64) {
            std::cerr << "PDO 0x" << std::hex << pdo_index << std::dec
                      << " mapping entry count out of range: "
                      << entry_count << "\n";
            return false;
        }
        if (!append_row(rows, 'P', std::to_string(ring) + "\t" +
                        std::to_string(assignment) + "\t" +
                        std::to_string(pdo_position) + "\t" +
                        std::to_string(pdo_index) + "\t" +
                        std::to_string(entry_count))) {
            return false;
        }

        std::cout << "  PDO 0x" << std::hex << std::uppercase << pdo_index
                  << std::dec << ": " << entry_count
                  << " mapped entries\n";

        for (std::uint32_t entry_position = 1; entry_position <= entry_count;
             ++entry_position) {
            if (!read_sdo(master, raw, ring, pdo_index,
                          static_cast<std::uint8_t>(entry_position), 4, read)) {
                return false;
            }
            const std::uint32_t mapped = read.value;
            const std::uint32_t bit_length = mapped & 0xFFU;
            if (bit_length == 0) {
                std::cerr << "PDO 0x" << std::hex << pdo_index << std::dec
                          << " entry " << entry_position
                          << " has zero width\n";
                return false;
            }
            if (!append_row(rows, 'M', std::to_string(ring) + "\t" +
                            std::to_string(assignment) + "\t" +
                            std::to_string(pdo_position) + "\t" +
                            std::to_string(pdo_index) + "\t" +
                            std::to_string(entry_position) + "\t" +
                            std::to_string(mapped))) {
                return false;
            }
            std::cout << "    " << std::hex << std::uppercase
                      << std::setw(4) << std::setfill('0')
                      << ((mapped >> 16U) & 0xFFFFU) << ":"
                      << std::setw(2) << ((mapped >> 8U) & 0xFFU)
                      << std::dec << " width=" << bit_length << "\n";
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: coe_sdo_readback EXPECTED_SLAVES ROWS.tsv RAW.log\n";
        return 2;
    }

    const std::string expected_text(argv[1]);
    std::size_t parsed = 0;
    const unsigned long expected = std::stoul(expected_text, &parsed, 10);
    if (parsed != expected_text.size() || expected == 0 || expected > 64) {
        std::cerr << "EXPECTED_SLAVES must be in [1, 64]\n";
        return 2;
    }

    std::ofstream rows(argv[2], std::ios::trunc);
    std::ofstream raw(argv[3], std::ios::trunc);
    if (!rows || !raw) {
        std::cerr << "could not create CoE readback output files\n";
        return 2;
    }

    MasterGuard guard;
    guard.master = ecrt_request_master(0);
    if (guard.master == nullptr) {
        std::cerr << "could not reserve IgH master 0\n";
        return 1;
    }

    // Reserve the master to finish any in-flight scan and freeze the slave
    // list, but deliberately do not call ecrt_master_activate(). This helper
    // only reads CoE objects: it does not configure PDO domains or run cyclic
    // process data. Before activation, IgH owns frame I/O and the blocking
    // ecrt_master_sdo_upload() API is valid (master_any); activating here would
    // transfer I/O ownership to the application and start applying cyclic
    // configuration/DC state transitions that this readback helper does not
    // implement.
    ec_master_info_t master_info{};
    if (ecrt_master(guard.master, &master_info) != 0 ||
        master_info.slave_count != static_cast<unsigned int>(expected)) {
        std::cerr << "IgH topology changed before CoE readback: expected "
                  << expected << " slave(s), found "
                  << master_info.slave_count << "\n";
        return 1;
    }

    std::array<ec_slave_info_t, 64> slave_info{};
    for (unsigned long ring_value = 0; ring_value < expected; ++ring_value) {
        const auto ring = static_cast<std::uint16_t>(ring_value);
        if (ecrt_master_get_slave(guard.master, ring, &slave_info[ring_value]) != 0) {
            std::cerr << "could not query IgH slave identity at ring position "
                      << ring << "\n";
            return 1;
        }
        const auto& identity = slave_info[ring_value];
        if (identity.vendor_id != 0x0000029CU ||
            identity.product_code != 0x03B11002U ||
            identity.revision_number != 0x00050005U ||
            (identity.al_state & 0x0FU) != EC_AL_STATE_PREOP) {
            std::cerr << "unexpected IgH slave identity at ring position "
                      << ring << ": vendor=0x" << std::hex
                      << identity.vendor_id << " product=0x"
                      << identity.product_code << " revision=0x"
                      << identity.revision_number << std::dec << "\n";
            return 1;
        }
        raw << "SLAVE position=" << ring << " vendor=0x"
            << std::hex << std::uppercase << identity.vendor_id
            << " product=0x" << identity.product_code
            << " revision=0x" << identity.revision_number
            << " al_state=0x" << static_cast<unsigned int>(identity.al_state)
            << std::dec << "\n";
    }

    raw << "MASTER_ACTIVATED=0 io_owner=IgH_kernel\n";
    raw.flush();
    std::cout << "IgH master reserved; kernel-owned I/O retained "
                 "(CoE readback without cyclic activation)\n";

    for (unsigned long ring_value = 0; ring_value < expected; ++ring_value) {
        const auto ring = static_cast<std::uint16_t>(ring_value);
        const auto& identity = slave_info[ring_value];
        if (!append_row(rows, 'S', std::to_string(ring) + "\t" +
                        std::to_string(identity.vendor_id) + "\t" +
                        std::to_string(identity.product_code))) {
            std::cerr << "could not write CoE identity row\n";
            return 1;
        }

        std::cout << "Slave ring position " << ring
                  << " CoE PDO mapping readback\n";
        if (!verify_assignment(guard.master, rows, raw, ring, 0x1C12,
                               "RxPDO") ||
            !verify_assignment(guard.master, rows, raw, ring, 0x1C13,
                               "TxPDO")) {
            return 1;
        }
    }

    rows.flush();
    raw.flush();
    if (!rows.good() || !raw.good()) {
        std::cerr << "failed to flush CoE readback artifacts\n";
        return 1;
    }

    std::cout << "PASS: IgH kernel-owned CoE SDO readback completed for "
              << expected << " slave(s)\n";
    return 0;
}
