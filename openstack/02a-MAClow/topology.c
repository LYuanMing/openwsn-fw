#include "config.h"
#include "opendefs.h"
#include "topology.h"
#include "idmanager.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

/**
\brief Force a topology.

This function is used to force a certain topology, by hard-coding the list of
acceptable neighbors for a given mote. This function is invoked each time a
packet is received. If it returns FALSE, the packet is silently dropped, as if
it were never received. If it returns TRUE, the packet is accepted.

Typically, filtering packets is done by analyzing the IEEE802.15.4 header. An
example body for this function which forces a topology is:

   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case TOPOLOGY_MOTE1:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE2) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      case TOPOLOGY_MOTE2:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE1 ||
             ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE3) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      default:
         returnVal=TRUE;
   }
   return returnVal;

By default, however, the function should return TRUE to *not* force any
topology.

\param[in] ieee802514_header The parsed IEEE802.15.4 MAC header.

\return TRUE if the packet can be received.
\return FALSE if the packet should be silently dropped.
*/
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header) {
#if OPENWSN_FORCETOPOLOGY_C
   bool returnVal;

   // guard against non-64-bit source address (the union access below is only
   // valid for ADDR_64B frames; foreign 16-bit sources would read garbage)
   if (ieee802514_header->src.type != ADDR_64B) {
      return TRUE;
   }

   returnVal = FALSE;

   // Per-board role table (EUI64 last byte -> behaviour).
   // Measured 2026-09-11 with tools/node_id.sh. Original nrf52840DK root
   // probe is sn 1050208732 (DK on-board nrf52840 is **powered off** — root
   // is an external nrf52840 attached to the DK's SWD pins and powered
   // separately at 3.3V). Bare nRF52840 DUT board reads via J-Link V9.8 PLUS
   // sn 602719539.
   //
   // Leaf board set evolved during the day as boards failed/replaced:
   //   sn 1050227853  EUI64[7]=0xE4   (always on, OK)
   //   sn 1050271158  EUI64[7]=0xCA   (replaced: died mid-day, marked inactive)
   //   sn 1050255521  EUI64[7]=0xCD   (replacement for sn 1050271158)
   //   sn 1050275279  EUI64[7]=0x18   (OK)
   //   sn 1050284179  EUI64[7]=0xBA   (OK)
   //   sn 1050292671  EUI64[7]=0x38   (OK)
   //
   // Round 1 (2026-09-11 16:09): only 0xe4 = leaf_on, others = leaf_off
   //   self 0x32  = root (external, flashed via DK sn 1050208732 OB)
   //   self 0x8c  = DUT  (bare nRF52840, VDD <- PPK2)
   //   self 0xe4  = leaf_on  (DK sn 1050227853) - accepts ONLY DUT
   //   self 0xcd  = leaf_off (DK sn 1050255521) - reject all
   //   self 0x18  = leaf_off (DK sn 1050275279) - reject all
   //   self 0xba  = leaf_off (DK sn 1050284179) - reject all
   //   self 0x38  = leaf_off (DK sn 1050292671) - reject all
   //
   // Round 2 (2026-09-11 17:00): all 5 leaves accept DUT, verify each connects
   //   self 0xe4 / 0xcd / 0x18 / 0xba / 0x38  = all 5 leaves accept ONLY DUT
   //
   // Round 7 (2026-09-11 20:20): per-build configurable leaf-on set, edited
   //   directly in this file by tools/apply_config.py --leaf-on N (N=2..5).
   //   For i <= N: case 0x<leaf-bytes[i]> branches accept DUT.
   //   For i >  N: case 0x<leaf-bytes[i]> branches fall through to returnVal=FALSE
   //   (reject all). Currently active leaf set (in apply_config order):
   //   self 0xe4  = leaf 1 (DK sn 1050227853)
   //   self 0xcd  = leaf 2 (DK sn 1050255521)
   //   self 0x18  = leaf 3 (DK sn 1050275279)
   //   self 0xba  = leaf 4 (DK sn 1050284179)
   //   self 0xd9  = leaf 5 (DK sn 683260444; replaced 0x38/1050292671 2026-09-14)
   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      // root and DUT must stay reachable: accept everything
      case 0x32:
      case 0x8c:
         returnVal = TRUE;
         break;
      // leaf 1 (LEAF1_ON)
      case 0xe4:
         if (ieee802514_header->src.addr_64b[7] == 0x8c) { returnVal = TRUE; }
         break;
      // leaf 2 (LEAF2_ON)
      case 0xcd:
         if (ieee802514_header->src.addr_64b[7] == 0x8c) { returnVal = TRUE; }
         break;
      // leaf 3 (LEAF3_ON)
      case 0x18:
         if (ieee802514_header->src.addr_64b[7] == 0x8c) { returnVal = TRUE; }
         break;
      // leaf 4 (LEAF4_ON)
      case 0xba:
         if (ieee802514_header->src.addr_64b[7] == 0x8c) { returnVal = TRUE; }
         break;
      // leaf 5 (LEAF5_ON)
      case 0x38:
         returnVal = FALSE;
         break;
      // leaf 5 replacement (LEAF5_ON; replaced 0x38/1050292671 2026-09-14)
      case 0xd9:
         if (ieee802514_header->src.addr_64b[7] == 0x8c) { returnVal = TRUE; }
         break;
      default:
         // any future / unknown board: behave like root/DUT (don't break the network)
         returnVal = TRUE;
         break;
   }
   return returnVal;
#else
   return TRUE;
#endif /* OPENWSN_FORCETOPOLOGY_C */
}

//=========================== private =========================================
