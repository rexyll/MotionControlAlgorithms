#pragma once
#define EC_MAX_SYNC_MANAGERS 16
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ec_master ec_master_t;
typedef struct ec_domain ec_domain_t;
typedef struct ec_slave_config ec_slave_config_t;
typedef struct ec_sdo_request ec_sdo_request_t;

typedef enum {
  EC_DIR_INVALID = 0,
  EC_DIR_OUTPUT = 1,
  EC_DIR_INPUT = 2
} ec_direction_t;

typedef enum {
  EC_WD_DEFAULT = 0,
  EC_WD_ENABLE = 1,
  EC_WD_DISABLE = 2
} ec_watchdog_mode_t;

typedef struct {
  uint16_t index;
  uint8_t subindex;
  uint8_t bit_length;
} ec_pdo_entry_info_t;

typedef struct {
  uint16_t index;
  unsigned int n_entries;
  ec_pdo_entry_info_t* entries;
} ec_pdo_info_t;

typedef struct {
  uint8_t index;
  ec_direction_t dir;
  uint8_t n_pdos;
  ec_pdo_info_t* pdos;
  ec_watchdog_mode_t watchdog_mode;
} ec_sync_info_t;

typedef enum {
  EC_REQUEST_UNUSED = 0,
  EC_REQUEST_BUSY = 1,
  EC_REQUEST_SUCCESS = 2,
  EC_REQUEST_ERROR = 3
} ec_request_state_t;

typedef struct { unsigned int slave_count; } ec_master_info_t;
typedef struct {
  uint16_t position;
  uint32_t vendor_id;
  uint32_t product_code;
  uint32_t revision_number;
  uint32_t serial_number;
  uint8_t al_state;
} ec_slave_info_t;
typedef struct {
  unsigned int working_counter;
  unsigned int wc_state;
} ec_domain_state_t;
typedef struct {
  unsigned int slaves_responding;
  unsigned int al_states : 4;
  unsigned int link_up : 1;
} ec_master_state_t;
typedef struct {
  unsigned int online : 1;
  unsigned int operational : 1;
  unsigned int al_state : 4;
} ec_slave_config_state_t;

enum {
  EC_AL_STATE_INIT = 1,
  EC_AL_STATE_PREOP = 2,
  EC_AL_STATE_SAFEOP = 4,
  EC_AL_STATE_OP = 8,
  EC_WC_COMPLETE = 2
};

ec_master_t* ecrt_request_master(unsigned int);
void ecrt_release_master(ec_master_t*);
int ecrt_master(ec_master_t*, ec_master_info_t*);
int ecrt_master_get_slave(ec_master_t*, uint16_t, ec_slave_info_t*);
int ecrt_master_state(const ec_master_t*, ec_master_state_t*);
int ecrt_slave_config_state(const ec_slave_config_t*, ec_slave_config_state_t*);
ec_domain_t* ecrt_master_create_domain(ec_master_t*);
ec_slave_config_t* ecrt_master_slave_config(ec_master_t*, uint16_t, uint16_t, uint32_t, uint32_t);
int ecrt_slave_config_pdos(ec_slave_config_t*, unsigned int, const ec_sync_info_t*);
int ecrt_master_get_sync_manager(ec_master_t*, uint16_t, uint8_t, ec_sync_info_t*);
int ecrt_master_get_pdo(ec_master_t*, uint16_t, uint8_t, uint16_t, ec_pdo_info_t*);
int ecrt_master_get_pdo_entry(ec_master_t*, uint16_t, uint8_t, uint16_t, uint16_t, ec_pdo_entry_info_t*);
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t*, uint16_t, uint8_t, ec_domain_t*, unsigned int*);
ec_sdo_request_t* ecrt_slave_config_create_sdo_request(ec_slave_config_t*, uint16_t, uint8_t, size_t);
int ecrt_sdo_request_timeout(ec_sdo_request_t*, uint32_t);
uint8_t* ecrt_sdo_request_data(ec_sdo_request_t*);
size_t ecrt_sdo_request_data_size(const ec_sdo_request_t*);
int ecrt_sdo_request_read(ec_sdo_request_t*);
int ecrt_sdo_request_write(ec_sdo_request_t*);
ec_request_state_t ecrt_sdo_request_state(const ec_sdo_request_t*);
int ecrt_master_set_send_interval(ec_master_t*, size_t);
int ecrt_master_activate(ec_master_t*);
int ecrt_master_deactivate(ec_master_t*);
uint8_t* ecrt_domain_data(ec_domain_t*);
size_t ecrt_domain_size(const ec_domain_t*);
int ecrt_master_receive(ec_master_t*);
int ecrt_domain_process(ec_domain_t*);
int ecrt_domain_state(const ec_domain_t*, ec_domain_state_t*);
int ecrt_domain_queue(ec_domain_t*);
int ecrt_master_send(ec_master_t*);

#ifdef __cplusplus
}
#endif
