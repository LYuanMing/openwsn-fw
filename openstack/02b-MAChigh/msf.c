#include "opendefs.h"
#include "msf.h"
#include "neighbors.h"
#include "sixtop.h"
#include "scheduler.h"
#include "schedule.h"
#include "openrandom.h"
#include "idmanager.h"
#include "icmpv6rpl.h"
#include "IEEE802154E.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "energy_aware.h"

//=========================== definition =====================================

//=========================== variables =======================================

msf_vars_t msf_vars;
msf_vars_debug_t msf_vars_debug;

//=========================== prototypes ======================================

// sixtop callback
uint16_t msf_getMetadata(void);

metadata_t msf_translateMetadata(void);

void msf_handleRCError(uint8_t code, open_addr_t *address);

void msf_timer_housekeeping_cb(opentimers_id_t id);

void msf_timer_housekeeping_task(void);

void msf_timer_waitretry_cb(opentimers_id_t id);

void msf_timer_clear_task(void);

// msf private
void msf_trigger6pAdd(void);

void msf_trigger6pDelete(void);

void msf_housekeeping(void);

//=========================== public ==========================================

void msf_init(void) {

    open_addr_t temp_neighbor;

    memset(&msf_vars, 0, sizeof(msf_vars_t));
    memset(&msf_vars_debug, 0, sizeof(msf_vars_debug_t));
    sixtop_setSFcallback(
            (sixtop_sf_getsfid_cbt) msf_getsfid,
            (sixtop_sf_getmetadata_cbt) msf_getMetadata,
            (sixtop_sf_translatemetadata_cbt) msf_translateMetadata,
            (sixtop_sf_handle_callback_cbt) msf_handleRCError
    );

    memset(&temp_neighbor, 0, sizeof(temp_neighbor));
    temp_neighbor.type = ADDR_ANYCAST;
    schedule_addActiveSlot(
            msf_hashFunction_getSlotoffset(idmanager_getMyID(ADDR_64B)),     // slot offset
            CELLTYPE_RX,                                                     // type of slot
            FALSE,                                                           // shared?
            TRUE,                                                            // auto cell?
            msf_hashFunction_getChanneloffset(idmanager_getMyID(ADDR_64B)),  // channel offset
            &temp_neighbor                                                   // neighbor
    );

    msf_vars.housekeepingTimerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_MSF);
    msf_vars.housekeepingPeriod = HOUSEKEEPING_PERIOD;
    opentimers_scheduleIn(
            msf_vars.housekeepingTimerId,
            openrandom_getRandomizePeriod(msf_vars.housekeepingPeriod, msf_vars.housekeepingPeriod),
            TIME_MS,
            TIMER_PERIODIC,
            msf_timer_housekeeping_cb
    );
    msf_vars.waitretryTimerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_MSF);
}

// called by schedule
void msf_updateCellsElapsed(open_addr_t *neighbor, cellType_t type) {

#if ADAPTIVE_MSF
    if (icmpv6rpl_isPreferredParent(neighbor) == FALSE) {
        return;
    }

    // update numcellselapsed
    switch (type) {
        case CELLTYPE_TX:
            msf_vars.numCellsElapsed_tx++;
            break;
        case CELLTYPE_RX:
            msf_vars.numCellsElapsed_rx++;
            break;
        default:
            // not appliable
            return;
    }

    // adapt to upward traffic
    if (msf_vars.numCellsElapsed_tx == MAX_NUMCELLS) {

        LOG_VERBOSE(COMPONENT_MSF, ERR_TX_CELL_USAGE, msf_vars.numCellsUsed_tx, 0);
        msf_vars.needAddTx = FALSE;
        msf_vars.needDeleteTx = FALSE;

        msf_vars.previousNumCellsUsed_tx = msf_vars.numCellsUsed_tx;

        // for debugging purposes
        msf_vars_debug.numCellsUsed_tx = msf_vars.numCellsUsed_tx;

        if (msf_vars.numCellsUsed_tx > LIM_NUMCELLSUSED_HIGH) {
            msf_vars.needAddTx = TRUE;
            scheduler_push_task(msf_trigger6pAdd, TASKPRIO_MSF);
        }
        if (msf_vars.numCellsUsed_tx < LIM_NUMCELLSUSED_LOW) {
            msf_vars.needDeleteTx = TRUE;
            scheduler_push_task(msf_trigger6pDelete, TASKPRIO_MSF);
        }
        msf_vars.numCellsElapsed_tx = 0;
        msf_vars.numCellsUsed_tx = 0;
    }

    // adapt to downward traffic when there are negotiated Tx cells in schedule
    if (schedule_getNumberOfNegotiatedCells(neighbor, CELLTYPE_TX) == 0) {
        return;
    }

    // adapt to downward traffic
    if (msf_vars.numCellsElapsed_rx == MAX_NUMCELLS) {

        LOG_VERBOSE(COMPONENT_MSF, ERR_RX_CELL_USAGE, msf_vars.numCellsUsed_rx, 0);
        msf_vars.needAddRx = FALSE;
        msf_vars.needDeleteRx = FALSE;

        msf_vars.previousNumCellsUsed_rx = msf_vars.numCellsUsed_rx;

        // for debugging purposes
        msf_vars_debug.numCellsUsed_rx = msf_vars.numCellsUsed_rx;

        if (msf_vars.numCellsUsed_rx > LIM_NUMCELLSUSED_HIGH) {
            msf_vars.needAddRx = TRUE;
            scheduler_push_task(msf_trigger6pAdd, TASKPRIO_MSF);
        }
        if (msf_vars.numCellsUsed_rx < LIM_NUMCELLSUSED_LOW) {
            msf_vars.needDeleteRx = TRUE;
            scheduler_push_task(msf_trigger6pDelete, TASKPRIO_MSF);
        }
        msf_vars.numCellsElapsed_rx = 0;
        msf_vars.numCellsUsed_rx = 0;
    }
#endif
}

void msf_updateCellsUsed(open_addr_t *neighbor, cellType_t type) {

    if (icmpv6rpl_isPreferredParent(neighbor) == FALSE) {
        return;
    }

    switch (type) {
        case CELLTYPE_TX:
            msf_vars.numCellsUsed_tx++;
            break;
        case CELLTYPE_RX:
            msf_vars.numCellsUsed_rx++;
            break;
        default:
            // not appliable
            return;
    }
}

//=========================== callback =========================================

uint8_t msf_getsfid(void) {
    return IANA_6TISCH_SFID_MSF;
}

uint16_t msf_getMetadata(void) {
    return SCHEDULE_MINIMAL_6TISCH_DEFAULT_SLOTFRAME_HANDLE;
}

metadata_t msf_translateMetadata(void) {
    return METADATA_TYPE_FRAMEID;
}

void msf_handleRCError(uint8_t code, open_addr_t *address) {
    uint16_t waitDuration;

    // perf3 2026-10-05 -- BEHAVIOUR FIX (see FREEZE_20261003.md appendix AA.5).
    //
    // IANA_6TOP_RC_BUSY was handled by permanently marking the neighbour
    // f6PNORES = TRUE.  That flag is only ever cleared by
    // neighbors_resetEntry() (neighbors.c:814), which is a full reset that also
    // drops the address, so a single transient BUSY disabled the neighbour for
    // good.
    //
    // Worse, the flag makes icmpv6rpl_getPreferredParentEui64() return FALSE
    // (icmpv6rpl.c:308-311), and that same call gates BOTH
    //   - forwarding_getNextHop()  -> next hop becomes ADDR_NONE -> E_FAIL, and
    //   - the 6P cell-addition task (msf.c:289) -> so the cell can never be
    //     re-established either.
    // i.e. one BUSY is self-sustaining: it stops the traffic AND the mechanism
    // that would repair the traffic.  Measured consequence: goodput collapsed to
    // zero and stayed there for the rest of the run.
    //
    // BUSY means "receiver temporarily unable to accept" (6P semantics), so the
    // correct response is the same bounded backoff already used for RESET and
    // LOCKED just above: stop adding cells for a random WAITDURATION, then let
    // the normal periodic task retry.  That is what the neighbouring branch
    // does, and it leaves the neighbour usable for routing in the meantime.
    if (code == IANA_6TOP_RC_BUSY) {
        // was: neighbors_setNeighborNoResource(address) -- permanent, and
        // self-sustaining because it also blocks the repair path.
        msf_vars.waitretry = TRUE;
        waitDuration = WAITDURATION_MIN + openrandom_get16b() % WAITDURATION_RANDOM_RANGE;
        opentimers_scheduleIn(
                msf_vars.waitretryTimerId,
                waitDuration,
                TIME_MS,
                TIMER_ONESHOT,
                msf_timer_waitretry_cb
        );
        neighbors_updateSequenceNumber(address);
        return;
    }

    if (code == IANA_6TOP_RC_RESET || code == IANA_6TOP_RC_LOCKED) {
        // waitretry
        msf_vars.waitretry = TRUE;
        waitDuration = WAITDURATION_MIN + openrandom_get16b() % WAITDURATION_RANDOM_RANGE;
        opentimers_scheduleIn(
                msf_vars.waitretryTimerId,
                waitDuration,
                TIME_MS,
                TIMER_ONESHOT,
                msf_timer_waitretry_cb
        );
    }

    if (code == IANA_6TOP_RC_ERROR || code == IANA_6TOP_RC_VER_ERR || code == IANA_6TOP_RC_SFID_ERR) {
        // quarantine
    }

    if (code == IANA_6TOP_RC_SEQNUM_ERR || code == IANA_6TOP_RC_CELLLIST_ERR) {
        // clear
        scheduler_push_task(msf_timer_clear_task, TASKPRIO_MSF);
    }

    neighbors_updateSequenceNumber(address);
}

void msf_timer_waitretry_cb(opentimers_id_t id) {
    msf_vars.waitretry = FALSE;
}

void msf_timer_housekeeping_cb(opentimers_id_t id) {
    PORT_TIMER_WIDTH newDuration;

    // update the timer period
    newDuration = openrandom_getRandomizePeriod(msf_vars.housekeepingPeriod, msf_vars.housekeepingPeriod),
            opentimers_updateDuration(msf_vars.housekeepingTimerId, newDuration);

    // calling the task directly as the timer_cb function is executed in
    // task mode by opentimer already
    msf_timer_housekeeping_task();
}

//=========================== tasks ============================================

void msf_timer_housekeeping_task(void) {

    msf_housekeeping();
}

void msf_timer_clear_task(void) {
    open_addr_t neighbor;
    bool foundNeighbor;

    // get preferred parent
    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&neighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    sixtop_request(
            IANA_6TOP_CMD_CLEAR,       // code
            &neighbor,                 // neighbor
            NUMCELLS_MSF,              // number cells
            CELLOPTIONS_MSF,           // cellOptions (not used)
            NULL,                      // celllist to add (not used)
            NULL,                      // celllist to delete (not used)
            IANA_6TISCH_SFID_MSF,      // sfid
            0,                         // list command offset (not used)
            0                          // list command maximum celllist (not used)
    );
}

//=========================== private =========================================

#if PERF3_ENERGY_CELL_GATE_MV > 0
// perf3 2026-10-13 (user design: "energy-gated cell allocation").
//
// WHY.  The campaign's one-cell model, expected_cmax(f, n_cell=1), assumes the
// uplink is ONE cell, but the DUT actually held TWO transmit cells to its
// parent: the 6P-negotiated dedicated cell (slot 82 here) AND the autonomous
// shared cell that sixtop_send_internal() installs at the parent's hash offset
// (slot 53 here).  Measured 2026-10-13 with the openserial-header ASN probe:
// 61% of the DUT's DATA frames leave on the auto shared cell and 36% on the
// dedicated one, so every measured throughput sat at a CONSTANT 1.52x the
// one-cell model -- the 1.52x is not a calibration error, it is an extra cell.
//
// WHAT.  MSF's own traffic thresholds (LIM_NUMCELLSUSED_HIGH etc.) are left
// exactly as they are.  An ENERGY rule is added on top: while the supply
// voltage is BELOW PERF3_ENERGY_CELL_GATE_MV (the campaign sets 96% of
// MAX_VOLTAGE_MV = 4032 mV) the node keeps exactly ONE uplink cell, and that
// cell is the 6P-NEGOTIATED one.  At/above the gate both cells may coexist
// (stock behaviour).
//
// DIRECTION MATTERS (paid for on the bench, 2026-10-13 23:29).  The first
// version did the opposite -- kept the auto shared cell and shed the
// negotiated one -- and the uplink stopped dead: uinject.c:622 gates every DATA
// transmission on schedule_hasNegotiatedCellToNeighbor(parent, CELLTYPE_TX),
// which is FALSE for a shared cell (schedule_getNumberOfNegotiatedCells()
// requires shared == FALSE, schedule.c:912-916).  The DUT then dropped every
// packet at d_nocell (measured: n_tx pinned at 82, d_nocell +14 every 14 s,
// n_rx = 0 for the whole window; the freeze guard aborted the cell at 422 s).
// sixtop.c:1764 had already recorded that exact failure for a blocked 6P ADD.
// Shedding the auto cell instead is also what the standards say a node should
// do: RFC 9033 3.1/3.2 have a node delete its AutoTxCell once a dedicated Tx
// cell exists.
//
// The shed is only armed once a negotiated TX cell EXISTS, because during join
// the 6P ADD request itself travels over the auto cell.
static bool perf3_energy_gate_open(void) {
    return (energy_vars.voltage_mV >= PERF3_ENERGY_CELL_GATE_MV);
}

// Remove the AUTONOMOUS SHARED TX cell to <neighbor> (the one
// sixtop_send_internal() installs at the parent's hash offset), leaving the
// negotiated dedicated cell as the single uplink cell.  Returns how many were
// removed.  The minimal TXRX cell at slot 0 is not matched: its neighbor is
// ADDR_ANYCAST, not the preferred parent.
static uint8_t perf3_shed_auto_shared_tx_cell(open_addr_t *neighbor) {
    uint8_t i;
    uint8_t pass;
    uint8_t removed_count;
    bool removed;
    slotinfo_element_t info;

    removed_count = 0;
    for (pass = 0; pass < 4; pass++) {
        removed = FALSE;
        for (i = 0; i < schedule_getFrameLength(); i++) {
            schedule_getSlotInfo(i, &info);
            if (info.link_type == CELLTYPE_TX &&
                    info.shared == TRUE &&
                    info.isAutoCell == TRUE &&
                    packetfunctions_sameAddress(neighbor, &(info.address)) == TRUE) {
                schedule_removeActiveSlot(info.slotOffset, info.link_type,
                                          info.shared, neighbor);
                removed_count++;
                removed = TRUE;
                // the buffer shifts under us (a backup entry can be promoted
                // into the freed slot), so restart the scan
                break;
            }
        }
        if (removed == FALSE) {
            break;
        }
    }
    return removed_count;
}
#endif

void msf_trigger6pAdd(void) {
    open_addr_t neighbor;
    bool foundNeighbor;
    cellInfo_ht celllist_add[CELLLIST_MAX_LEN];

    uint8_t cellOptions;

    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    if (msf_vars.waitretry) {
        return;
    }

    // get preferred parent
    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&neighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    // check what type of cell need to add

    if (msf_vars.needAddTx) {
        cellOptions = CELLOPTIONS_TX;
    } else {
        if (msf_vars.needAddRx) {
            cellOptions = CELLOPTIONS_RX;
        } else {
            // no need to add cell
            return;
        }
    }

#if PERF3_MAX_TX_CELLS > 0
    // perf3: one-slot uplink.  Do not grow the TX cell set beyond the cap; the
    // surplus is removed by msf_trigger6pDelete() instead.
    if (cellOptions == CELLOPTIONS_TX
            && schedule_getNumberOfNegotiatedCells(&neighbor, CELLTYPE_TX)
               >= PERF3_MAX_TX_CELLS) {
        if (msf_vars.needDeleteTx == FALSE
                && schedule_getNumberOfNegotiatedCells(&neighbor, CELLTYPE_TX)
                   > PERF3_MAX_TX_CELLS) {
            msf_vars.needDeleteTx = TRUE;
            scheduler_push_task(msf_trigger6pDelete, TASKPRIO_MSF);
        }
        return;
    }
#endif

    if (msf_candidateAddCellList(celllist_add, NUMCELLS_MSF) == FALSE) {
        // failed to get cell list to add
        return;
    }

    sixtop_request(
            IANA_6TOP_CMD_ADD,           // code
            &neighbor,                   // neighbor
            NUMCELLS_MSF,                // number cells
            cellOptions,                 // cellOptions
            celllist_add,                // celllist to add
            NULL,                        // celllist to delete (not used)
            IANA_6TISCH_SFID_MSF,        // sfid
            0,                           // list command offset (not used)
            0                            // list command maximum celllist (not used)
    );
}

void msf_trigger6pDelete(void) {
    open_addr_t neighbor;
    bool foundNeighbor;
    cellInfo_ht celllist_delete[CELLLIST_MAX_LEN];

    uint8_t cellOptions;

    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    if (msf_vars.waitretry) {
        return;
    }

    // get preferred parent
    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&neighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    if (msf_vars.needDeleteTx) {
        if (schedule_getNumberOfNegotiatedCells(&neighbor, CELLTYPE_TX) <= 1) {
            // at least one negotiated Tx cell presents
            msf_vars.needDeleteTx = FALSE;
        }
    }

#if PERF3_MAX_TX_CELLS > 0
    // perf3: one-slot uplink.  If cells were negotiated before the cap was
    // enforced (or were inherited), shed them down to the cap as soon as the
    // node is synchronised, independent of the MSF usage thresholds.
    if (msf_vars.needDeleteTx == FALSE
            && schedule_getNumberOfNegotiatedCells(&neighbor, CELLTYPE_TX)
               > PERF3_MAX_TX_CELLS) {
        msf_vars.needDeleteTx = TRUE;
    }
#endif

    // check what type of cell need to delete
    if (msf_vars.needDeleteTx) {
        cellOptions = CELLOPTIONS_TX;
    } else {
        if (msf_vars.needDeleteRx) {
            cellOptions = CELLOPTIONS_RX;
        } else {
            // no need to delete cell
            return;
        }
    }

    if (msf_candidateRemoveCellList(celllist_delete, &neighbor, NUMCELLS_MSF, cellOptions) == FALSE) {
        // failed to get cell list to delete
        return;
    }

    sixtop_request(
            IANA_6TOP_CMD_DELETE,   // code
            &neighbor,              // neighbor
            NUMCELLS_MSF,           // number cells
            cellOptions,            // cellOptions
            NULL,                   // celllist to add (not used)
            celllist_delete,        // celllist to delete
            IANA_6TISCH_SFID_MSF,   // sfid
            0,                      // list command offset (not used)
            0                       // list command maximum celllist (not used)
    );
}

bool msf_candidateAddCellList(
        cellInfo_ht *cellList,
        uint8_t requiredCells
) {
    uint8_t i;
    frameLength_t slotoffset;
    uint8_t numCandCells;

    memset(cellList, 0, CELLLIST_MAX_LEN * sizeof(cellInfo_ht));
    numCandCells = 0;
    for (i = 0; i < CELLLIST_MAX_LEN; i++) {
        slotoffset = openrandom_get16b() % schedule_getFrameLength();
        if (schedule_isSlotOffsetAvailable(slotoffset) == TRUE) {
            cellList[numCandCells].slotoffset = slotoffset;
            cellList[numCandCells].channeloffset = openrandom_get16b() & 0x0F;
            cellList[numCandCells].isUsed = TRUE;
            numCandCells++;
        }
    }

    if (numCandCells < requiredCells || requiredCells == 0) {
        return FALSE;
    } else {
        return TRUE;
    }
}

bool msf_candidateRemoveCellList(
        cellInfo_ht *cellList,
        open_addr_t *neighbor,
        uint8_t requiredCells,
        uint8_t cellOptions
) {
    uint8_t i;
    uint8_t numCandCells;
    slotinfo_element_t info;

    memset(cellList, 0, CELLLIST_MAX_LEN * sizeof(cellInfo_ht));
    numCandCells = 0;
    for (i = 0; i < schedule_getFrameLength(); i++) {
        schedule_getSlotInfo(i, &info);
        if (
                packetfunctions_sameAddress(neighbor, &(info.address)) &&
                info.link_type == cellOptions &&
                info.isAutoCell == FALSE
                ) {
            cellList[numCandCells].slotoffset = i;
            cellList[numCandCells].channeloffset = info.channelOffset;
            cellList[numCandCells].isUsed = TRUE;
            numCandCells++;
            if (numCandCells == CELLLIST_MAX_LEN) {
                break;
            }
        }
    }

    if (numCandCells < requiredCells) {
        return FALSE;
    } else {
        return TRUE;
    }
}

void msf_housekeeping(void) {

    open_addr_t parentNeighbor;
    open_addr_t nonParentNeighbor;
    bool foundNeighbor;
    cellInfo_ht celllist_add[CELLLIST_MAX_LEN];
    cellInfo_ht celllist_delete[CELLLIST_MAX_LEN];

    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    if (schedule_hasNegotiatedTxCellToNonParent(&parentNeighbor, &nonParentNeighbor) == TRUE) {

        // send a clear request to the non-parent neighbor

        sixtop_request(
                IANA_6TOP_CMD_CLEAR,     // code
                &nonParentNeighbor,      // neighbor
                NUMCELLS_MSF,            // number cells
                CELLOPTIONS_MSF,         // cellOptions
                NULL,                    // celllist to add (not used)
                NULL,                    // celllist to delete (not used)
                IANA_6TISCH_SFID_MSF,    // sfid
                0,                       // list command offset (not used)
                0                        // list command maximum celllist (not used)
        );
        return;
    }

#if PERF3_ENERGY_CELL_GATE_MV > 0
    // perf3 2026-10-13 v2: "shut" side of the energy rule (see the block above
    // msf_trigger6pAdd).  Runs on every housekeeping pass -- 30 s -- so the auto
    // shared cell is shed as soon as the capacitor is below the gate.  The shed
    // is armed ONLY when a negotiated TX cell exists, because during join the
    // 6P ADD request itself travels over the auto cell, and because uinject.c:622
    // would refuse to transmit at all if the negotiated cell were the one taken
    // away (measured failure mode, see the comment block above).
    if (perf3_energy_gate_open() == FALSE &&
            schedule_getNumberOfNegotiatedCells(&parentNeighbor, CELLTYPE_TX) > 0) {
        perf3_shed_auto_shared_tx_cell(&parentNeighbor);
    }
#endif

    if (schedule_getNumberOfNegotiatedCells(&parentNeighbor, CELLTYPE_TX) == 0) {
        msf_vars.needAddTx = TRUE;
        msf_trigger6pAdd();
        return;
    }

    if (msf_vars.waitretry) {
        return;
    }

    if (schedule_isNumTxWrapped(&parentNeighbor) == FALSE) {
        return;
    }

    memset(celllist_delete, 0, CELLLIST_MAX_LEN * sizeof(cellInfo_ht));
    if (schedule_getCellsToBeRelocated(&parentNeighbor, celllist_delete)) {
        if (msf_candidateAddCellList(celllist_add, NUMCELLS_MSF) == FALSE) {
            // failed to get cell list to add
            return;
        }
        sixtop_request(
                IANA_6TOP_CMD_RELOCATE,  // code
                &parentNeighbor,         // neighbor
                NUMCELLS_MSF,            // number cells
                CELLOPTIONS_MSF,         // cellOptions
                celllist_add,            // celllist to add
                celllist_delete,         // celllist to delete
                IANA_6TISCH_SFID_MSF,    // sfid
                0,                       // list command offset (not used)
                0                        // list command maximum celllist (not used)
        );
    }
}

uint16_t msf_hashFunction_getSlotoffset(open_addr_t *address) {

    uint16_t moteId;

    moteId = (((uint16_t)(address->addr_64b[6])) << 8) + (uint16_t)(address->addr_64b[7]);

    return SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + \
            (moteId % (SLOTFRAME_LENGTH - SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS));
}

uint8_t msf_hashFunction_getChanneloffset(open_addr_t *address) {

    uint16_t moteId;

    moteId = (((uint16_t)(address->addr_64b[6])) << 8) + (uint16_t)(address->addr_64b[7]);

    return moteId % NUM_CHANNELS;
}

void msf_setHashCollisionFlag(bool isCollision) {
    msf_vars.f_hashCollision = isCollision;
}

bool msf_getHashCollisionFlag(void) {
    return msf_vars.f_hashCollision;
}

uint8_t msf_getPreviousNumCellsUsed(cellType_t cellType) {
    switch (cellType) {
        case CELLTYPE_TX:
            return msf_vars.previousNumCellsUsed_tx;
            break;
        case CELLTYPE_RX:
            return msf_vars.previousNumCellsUsed_rx;
            break;
        default:
            // not appliable
            return 0;
    }
}

bool debugPrint_msf() {
    openserial_printStatus(STATUS_MSF, (uint8_t * ) & msf_vars_debug, sizeof(msf_vars_debug_t));
    return TRUE;
}
