/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  CAN bootloader support
 */
#include <AP_HAL/AP_HAL.h>
#include <hal.h>
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <canard.h>
#include "support.h"
#include <dronecan_msgs.h>
#include "can.h"
#include "bl_protocol.h"
#include <drivers/stm32/canard_stm32.h>
#include "app_comms.h"
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <stdio.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_CheckFirmware/AP_CheckFirmware.h>

static CanardInstance canard;
static uint32_t canard_memory_pool[4096/4];
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
static uint8_t initial_node_id = HAL_CAN_DEFAULT_NODE_ID;

// can config for 1MBit
static uint32_t baudrate = 1000000U;

#if HAL_USE_CAN
static CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    0 // filled in below
};
// pipelining is not faster when using ChibiOS CAN driver
#define FW_UPDATE_PIPELINE_LEN 1
#else
ChibiOS::CANIface can_iface[HAL_NUM_CAN_IFACES];
#endif

#ifndef CAN_APP_VERSION_MAJOR
#define CAN_APP_VERSION_MAJOR                                           2
#endif
#ifndef CAN_APP_VERSION_MINOR
#define CAN_APP_VERSION_MINOR                                           0
#endif
#ifndef CAN_APP_NODE_NAME
#define CAN_APP_NODE_NAME "org.ardupilot." CHIBIOS_BOARD_NAME
#endif

#ifdef EXT_FLASH_SIZE_MB
static_assert(EXT_FLASH_SIZE_MB == 0, "DroneCAN bootloader cannot support external flash");
#endif

static uint8_t node_id_allocation_transfer_id;
static uavcan_protocol_NodeStatus node_status;
static uint32_t send_next_node_id_allocation_request_at_ms;
static uint8_t node_id_allocation_unique_id_offset;

static void processTx(void);

// keep up to 4 transfers in progress
#ifndef FW_UPDATE_PIPELINE_LEN
#define FW_UPDATE_PIPELINE_LEN 4
#endif

#if CH_CFG_USE_MUTEXES == TRUE
static HAL_Semaphore can_mutex;
#endif

static struct {
    uint32_t rtt_ms;
    uint32_t ofs;
    uint8_t node_id;
    uint8_t path[sizeof(uavcan_protocol_file_Path::path.data)+1];
    uint8_t sector;
    uint32_t sector_ofs;
    uint8_t transfer_id;
    uint8_t idx;
    struct {
        uint8_t tx_id;
        uint32_t sent_ms;
        uint32_t offset;
        bool have_reply;
        uavcan_protocol_file_ReadResponse pkt;
    } reads[FW_UPDATE_PIPELINE_LEN];
    uint16_t erased_to;
} fw_update;

/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data);
    memset(out_uid, 0, len);
    memcpy(out_uid, (const void *)UDID_START, MIN(len,12));
}

/*
  simple 16 bit random number generator
 */
static uint16_t get_randomu16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}


/**
 * Returns a pseudo random integer in a given range
 */
static uint32_t get_random_range(uint16_t range)
{
    return get_randomu16() % range;
}

/*
  handle a GET_NODE_INFO request
 */
static void handle_get_node_info(CanardInstance* ins,
                                 CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uavcan_protocol_GetNodeInfoResponse pkt {};

    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    pkt.status = node_status;
    pkt.software_version.major = CAN_APP_VERSION_MAJOR;
    pkt.software_version.minor = CAN_APP_VERSION_MINOR;

    readUniqueID(pkt.hardware_version.unique_id);

    // use hw major/minor for APJ_BOARD_ID so we know what fw is
    // compatible with this hardware
    pkt.hardware_version.major = APJ_BOARD_ID >> 8;
    pkt.hardware_version.minor = APJ_BOARD_ID & 0xFF;

    char name[strlen(CAN_APP_NODE_NAME)+1];
    strcpy(name, CAN_APP_NODE_NAME);
    pkt.name.len = strlen(CAN_APP_NODE_NAME);
    memcpy(pkt.name.data, name, pkt.name.len);

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer, true);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  send a read for a fw update
 */
static bool send_fw_read(uint8_t idx)
{
    auto &r = fw_update.reads[idx];
    r.tx_id = fw_update.transfer_id;
    r.have_reply = false;

    uavcan_protocol_file_ReadRequest pkt {};
    pkt.path.path.len = strlen((const char *)fw_update.path);
    pkt.offset = r.offset;
    memcpy(pkt.path.path.data, fw_update.path, pkt.path.path.len);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer, true);

    if (canardRequestOrRespond(&canard,
                               fw_update.node_id,
                               UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                               UAVCAN_PROTOCOL_FILE_READ_ID,
                               &fw_update.transfer_id,
                               CANARD_TRANSFER_PRIORITY_HIGH,
                               CanardRequest,
                               &buffer[0],
                               total_size) > 0) {
        // mark it as having been sent
        r.sent_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

/*
  send a read for a fw update
 */
static void send_fw_reads(void)
{
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<FW_UPDATE_PIPELINE_LEN; i++) {
        const uint8_t idx = (fw_update.idx+i) % FW_UPDATE_PIPELINE_LEN;
        const auto &r = fw_update.reads[idx];
        if (r.have_reply) {
            continue;
        }
        if (r.sent_ms != 0 && now - r.sent_ms < 10+2*MAX(250,fw_update.rtt_ms)) {
            // waiting on a response
            continue;
        }
        if (!send_fw_read(idx)) {
            break;
        }
    }
}

/*
  erase up to at least the given sector number
 */
static void erase_to(uint16_t sector)
{
    if (sector < fw_update.erased_to) {
        return;
    }
    flash_func_erase_sector(sector);
    fw_update.erased_to = sector+1;

    /*
      pre-erase any non-erased pages up to end of flash. This puts all
      the load of erasing at the start of flashing which is much
      faster than flashing as we go on boards with small flash
      sectors.  We stop at the first already erased page so we don't
      end up wasting time erasing already erased pages when the
      firmware is much smaller than the total flash size
     */
    while (flash_func_sector_size(fw_update.erased_to) != 0 &&
           !flash_func_is_erased(fw_update.erased_to)) {
        flash_func_erase_sector(fw_update.erased_to);
        fw_update.erased_to++;
    }
}

/*
  handle response to file read for fw update
 */
static void handle_file_read_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (transfer->source_node_id != fw_update.node_id) {
        return;
    }
    /*
      match the response to a sent request
     */
    uint8_t idx = 0;
    bool found = false;
    for (idx=0; idx<FW_UPDATE_PIPELINE_LEN; idx++) {
        const auto &r = fw_update.reads[idx];
        if (r.tx_id == transfer->transfer_id) {
            found = true;
            break;
        }
    }
    if (!found) {
        // not a current transfer, we may be getting long delays
        fw_update.rtt_ms = MIN(3000, fw_update.rtt_ms+250);
        return;
    }
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &fw_update.reads[idx].pkt)) {
        return;
    }
    fw_update.reads[idx].have_reply = true;
    uint32_t rtt = MIN(3000,MAX(AP_HAL::millis() - fw_update.reads[idx].sent_ms, 25));
    fw_update.rtt_ms = uint32_t(0.9 * fw_update.rtt_ms + 0.1 * rtt);

    while (fw_update.reads[fw_update.idx].have_reply) {
        auto &r = fw_update.reads[fw_update.idx];
        if (r.offset != fw_update.ofs) {
            // bad sequence
            r.have_reply = false;
            r.sent_ms = 0;
            break;
        }
        const auto &pkt = r.pkt;
        const uint16_t len = pkt.data.len;
        const uint16_t len_words = (len+3U)/4U;
        const uint8_t *buf = (uint8_t *)pkt.data.data;
        uint32_t buf32[len_words] {};
        memcpy((uint8_t*)buf32, buf, len);

        if (fw_update.ofs == 0) {
            flash_set_keep_unlocked(true);
        }

        const uint32_t sector_size = flash_func_sector_size(fw_update.sector);
        if (sector_size == 0) {
            // firmware is too big
            fw_update.node_id = 0;
            flash_write_flush();
            flash_set_keep_unlocked(false);
            node_status.vendor_specific_status_code = uint8_t(check_fw_result_t::FAIL_REASON_BAD_LENGTH_APP);
            break;
        }
        if (fw_update.sector_ofs == 0) {
            erase_to(fw_update.sector);
        }
        if (fw_update.sector_ofs+len > sector_size) {
            erase_to(fw_update.sector+1);
        }
        if (!flash_write_buffer(fw_update.ofs, buf32, len_words)) {
            continue;
        }

        fw_update.ofs += len;
        fw_update.sector_ofs += len;
        if (fw_update.sector_ofs >= flash_func_sector_size(fw_update.sector)) {
            fw_update.sector++;
            fw_update.sector_ofs -= sector_size;
        }

        if (len < sizeof(uavcan_protocol_file_ReadResponse::data.data)) {
            fw_update.node_id = 0;
            flash_write_flush();
            flash_set_keep_unlocked(false);
            const auto ok = check_good_firmware();
            node_status.vendor_specific_status_code = uint8_t(ok);
            if (ok == check_fw_result_t::CHECK_FW_OK) {
                jump_to_app();
            }
            return;
        }

        r.have_reply = false;
        r.sent_ms = 0;
        r.offset += FW_UPDATE_PIPELINE_LEN*sizeof(uavcan_protocol_file_ReadResponse::data.data);
        send_fw_read(fw_update.idx);
        processTx();

        fw_update.idx = (fw_update.idx + 1) % FW_UPDATE_PIPELINE_LEN;
    }

    // show offset number we are flashing in kbyte as crude progress indicator
    node_status.vendor_specific_status_code = 1 + (fw_update.ofs / 1024U);
}

/*
  handle a begin firmware update request. We start pulling in the file data
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (fw_update.node_id == 0) {
        uavcan_protocol_file_BeginFirmwareUpdateRequest pkt;
        if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &pkt)) {
            return;
        }
        if (pkt.image_file_remote_path.path.len > sizeof(fw_update.path)-1) {
            return;
        }
        memset(&fw_update, 0, sizeof(fw_update));
        for (uint8_t i=0; i<FW_UPDATE_PIPELINE_LEN; i++) {
            fw_update.reads[i].offset = i*sizeof(uavcan_protocol_file_ReadResponse::data.data);
        }
        memcpy(fw_update.path, pkt.image_file_remote_path.path.data, pkt.image_file_remote_path.path.len);
        fw_update.path[pkt.image_file_remote_path.path.len] = 0;
        fw_update.node_id = pkt.source_node_id;
        if (fw_update.node_id == 0) {
            fw_update.node_id = transfer->source_node_id;
        }
    }

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer, true);
    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        node_id_allocation_unique_id_offset = 0;
        return;
    }

    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
        return;
    }
    
    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset = msg.unique_id.len;
        send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else if (msg.node_id != CANARD_BROADCAST_NODE_ID) { // new ID valid? (if not we will time out and start over)
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(ins, transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        handle_begin_firmware_update(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_READ_ID:
        handle_file_read_response(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        NVIC_SystemReset();
        break;
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_READ_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
        return true;
    default:
        break;
    }

    return false;
}

#if HAL_USE_CAN
static void processTx(void)
{
    static uint8_t fail_count;
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        CANTxFrame txmsg {};
        txmsg.DLC = txf->data_len;
        memcpy(txmsg.data8, txf->data, 8);
        txmsg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
        txmsg.IDE = 1;
        txmsg.RTR = 0;
        if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE) == MSG_OK) {
            canardPopTxQueue(&canard);
            fail_count = 0;
        } else {
            // just exit and try again later. If we fail 8 times in a row
            // then start discarding to prevent the pool filling up
            if (fail_count < 8) {
                fail_count++;
            } else {
                canardPopTxQueue(&canard);
            }
            return;
        }
    }
}

static void processRx(void)
{
    CANRxFrame rxmsg {};
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
        CanardCANFrame rx_frame {};

#ifdef HAL_GPIO_PIN_LED_BOOTLOADER
        palToggleLine(HAL_GPIO_PIN_LED_BOOTLOADER);
#endif
        const uint64_t timestamp = AP_HAL::micros64();
        memcpy(rx_frame.data, rxmsg.data8, 8);
        rx_frame.data_len = rxmsg.DLC;
        if(rxmsg.IDE) {
            rx_frame.id = CANARD_CAN_FRAME_EFF | rxmsg.EID;
        } else {
            rx_frame.id = rxmsg.SID;
        }
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}
#else
// Use HAL CAN interface
static void processTx(void)
{
    static uint8_t fail_count;
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        AP_HAL::CANFrame txmsg {};
        txmsg.dlc = txf->data_len;
        memcpy(txmsg.data, txf->data, 8);
        txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
        // push message with 1s timeout
        bool send_ok = false;
        for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
            send_ok |= (can_iface[i].send(txmsg, AP_HAL::micros64() + 1000000, 0) > 0);
        }
        if (send_ok) {
            canardPopTxQueue(&canard);
            fail_count = 0;
        } else {
            // just exit and try again later. If we fail 8 times in a row
            // then start discarding to prevent the pool filling up
            if (fail_count < 8) {
                fail_count++;
            } else {
                canardPopTxQueue(&canard);
            }
            return;
        }
    }
}

static void processRx(void)
{
    AP_HAL::CANFrame rxmsg;
    while (true) {
        bool got_pkt = false;
        for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
            bool read_select = true;
            bool write_select = false;
            can_iface[i].select(read_select, write_select, nullptr, 0);
            if (!read_select) {
                continue;
            }
#ifdef HAL_GPIO_PIN_LED_BOOTLOADER
            palToggleLine(HAL_GPIO_PIN_LED_BOOTLOADER);
#endif
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            can_iface[i].receive(rxmsg, timestamp, flags);
            memcpy(rx_frame.data, rxmsg.data, 8);
            rx_frame.data_len = rxmsg.dlc;
            rx_frame.id = rxmsg.id;
            canardHandleRxFrame(&canard, &rx_frame, timestamp);
            got_pkt = true;
        }
        if (!got_pkt) {
            break;
        }
    }
}
#endif //#if HAL_USE_CAN

/*
  wrapper around broadcast
 */
static void canard_broadcast(uint64_t data_type_signature,
                             uint16_t data_type_id,
                             uint8_t &transfer_id,
                             uint8_t priority,
                             const void* payload,
                             uint16_t payload_len)
{
#if CH_CFG_USE_MUTEXES == TRUE
    WITH_SEMAPHORE(can_mutex);
#endif
    canardBroadcast(&canard,
                    data_type_signature,
                    data_type_id,
                    &transfer_id,
                    priority,
                    payload,
                    payload_len);
}


/*
  handle waiting for a node ID
 */
static void can_handle_DNA(void)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        return;
    }

    if (AP_HAL::millis() < send_next_node_id_allocation_request_at_ms) {
        return;
    }

    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
    
    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(CANARD_BROADCAST_NODE_ID << 1U);

    if (node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data) - node_id_allocation_unique_id_offset);
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canard_broadcast(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                     UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                     node_id_allocation_transfer_id,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &allocation_request[0],
                     (uint16_t) (uid_size + 1));

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation_unique_id_offset = 0;
}

static void send_node_status(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer, true);

    static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

    canard_broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                     UAVCAN_PROTOCOL_NODESTATUS_ID,
                     transfer_id,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     buffer,
                     len);
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        node_status.mode = fw_update.node_id?UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE:UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
        send_node_status();
    }
}

void can_set_node_id(uint8_t node_id)
{
    initial_node_id = node_id;
}

// check for a firmware update marker left by app
bool can_check_update(void)
{
    bool ret = false;
#if HAL_RAM_RESERVE_START >= 256
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic == APP_BOOTLOADER_COMMS_MAGIC && comms->my_node_id != 0) {
        can_set_node_id(comms->my_node_id);
        fw_update.node_id = comms->server_node_id;
        for (uint8_t i=0; i<FW_UPDATE_PIPELINE_LEN; i++) {
            fw_update.reads[i].offset = i*sizeof(uavcan_protocol_file_ReadResponse::data.data);
        }
        memcpy(fw_update.path, comms->path, sizeof(uavcan_protocol_file_Path::path.data)+1);
        ret = true;
        // clear comms region
        memset(comms, 0, sizeof(struct app_bootloader_comms));
    }
#endif
#if defined(CAN1_BASE) && defined(RCC_APB1ENR_CAN1EN)
    // check for px4 fw update. px4 uses the filter registers in CAN1
    // to communicate with the bootloader. This only works on CAN1
    if (!ret && stm32_was_software_reset()) {
        uint32_t *fir = (uint32_t *)(CAN1_BASE + 0x240);
        struct PACKED app_shared {
            union {
                uint64_t ull;
                uint32_t ul[2];
                uint8_t  valid;
            } crc;
            uint32_t signature;
            uint32_t bus_speed;
            uint32_t node_id;
        } *app = (struct app_shared *)&fir[4];
        /* we need to enable the CAN peripheral in order to look at
           the FIR registers.
        */
        RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
        static const uint32_t app_signature = 0xb0a04150;
        if (app->signature == app_signature &&
            app->node_id > 0 && app->node_id < 128) {
            // crc is in reversed word order in FIR registers
            uint32_t sig[3];
            memcpy((uint8_t *)&sig[0], (const uint8_t *)&app->signature, sizeof(sig));
            const uint64_t crc = crc_crc64(sig, 3);
            const uint32_t *crc32 = (const uint32_t *)&crc;
            if (crc32[0] == app->crc.ul[1] &&
                crc32[1] == app->crc.ul[0]) {
                // reset signature so we don't get in a boot loop
                app->signature = 0;
                // setup node ID
                can_set_node_id(app->node_id);
                // and baudrate
                baudrate = app->bus_speed;
                ret = true;
            }
        }
    }
#endif
    return ret;
}

void can_start()
{
    node_status.vendor_specific_status_code = uint8_t(check_good_firmware());
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;

#if HAL_USE_CAN
    // calculate optimal CAN timings given PCLK1 and baudrate
    CanardSTM32CANTimings timings {};
    canardSTM32ComputeCANTimings(STM32_PCLK1, baudrate, &timings);
    cancfg.btr = CAN_BTR_SJW(0) |
        CAN_BTR_TS2(timings.bit_segment_2-1) |
        CAN_BTR_TS1(timings.bit_segment_1-1) |
        CAN_BTR_BRP(timings.bit_rate_prescaler-1);
    canStart(&CAND1, &cancfg);
#else
    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        can_iface[i].init(baudrate, AP_HAL::CANIface::NormalMode);
    }
#endif
    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    if (initial_node_id != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&canard, initial_node_id);
    }

    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (stm32_was_watchdog_reset()) {
        node_status.vendor_specific_status_code = uint8_t(check_fw_result_t::FAIL_REASON_WATCHDOG);
    }

    {
        /*
          support termination solder bridge or switch and optional LED
         */
#if defined(HAL_GPIO_PIN_GPIO_CAN1_TERM) && defined(HAL_GPIO_PIN_GPIO_CAN1_TERM_SWITCH)
        const bool can1_term = palReadLine(HAL_GPIO_PIN_GPIO_CAN1_TERM_SWITCH);
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN1_TERM, can1_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN1_TERM_LED, can1_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif

#if defined(HAL_GPIO_PIN_GPIO_CAN2_TERM) && defined(HAL_GPIO_PIN_GPIO_CAN2_TERM_SWITCH)
        const bool can2_term = palReadLine(HAL_GPIO_PIN_GPIO_CAN2_TERM_SWITCH);
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN2_TERM, can2_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN2_TERM_LED, can2_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif

#if defined(HAL_GPIO_PIN_GPIO_CAN3_TERM) && defined(HAL_GPIO_PIN_GPIO_CAN3_TERM_SWITCH)
        const bool can3_term = palReadLine(HAL_GPIO_PIN_GPIO_CAN3_TERM_SWITCH);
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN3_TERM, can3_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN3_TERM_LED, can3_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif
    }
}


void can_update()
{
    // do one loop of CAN support. If we are doing a firmware update
    // then loop until it is finished
    do {
        processTx();
        processRx();
        can_handle_DNA();
        static uint32_t last_1Hz_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_1Hz_ms >= 1000) {
            last_1Hz_ms = now;
            process1HzTasks(AP_HAL::micros64());
        }
        if (fw_update.node_id != 0) {
            send_fw_reads();
        }
#if CH_CFG_ST_FREQUENCY >= 1000000
        // give a bit of time for background processing
        chThdSleepMicroseconds(200);
#endif
    } while (fw_update.node_id != 0);
}

// printf to CAN LogMessage for debugging
void can_vprintf(const char *fmt, va_list ap)
{
    // only on H7 for now, where we have plenty of flash
#if defined(STM32H7)
    uavcan_protocol_debug_LogMessage pkt {};
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer, true);
    static uint8_t logmsg_transfer_id;

    canard_broadcast(UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                     UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                     logmsg_transfer_id,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     buffer,
                     len);
#endif // defined(STM32H7)
}

// printf to CAN LogMessage for debugging
void can_printf(const char *fmt, ...)
{
    // only on H7 for now, where we have plenty of flash
#if defined(STM32H7)
    va_list ap;
    va_start(ap, fmt);
    can_vprintf(fmt, ap);
    va_end(ap);
#endif // defined(STM32H7)
}

void can_printf_severity(uint8_t severity, const char *fmt, ...)
{
    // only on H7 for now, where we have plenty of flash
#if defined(STM32H7)
    va_list ap;
    va_start(ap, fmt);
    can_vprintf(fmt, ap);
    va_end(ap);
#endif
}


#endif // HAL_USE_CAN
