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

   // Per-board role table for the perf-test bench (EUI64 last byte -> behaviour).
   // Bench = 5 x nRF52840 DK, all flashed from this same tree.
   //   0xe4  root   DK sn 1050227853   vcom0 COM30
   //   0xcd  DUT    DK sn 1050255521   vcom0 COM41
   //   0x18  child1 DK sn 1050275279   vcom0 COM34
   //   0xba  child2 DK sn 1050284179   vcom0 COM36
   //   0xd9  child3 DK sn 683260444    vcom0 COM42
   // root/DUT accept everything; each child accepts ONLY the DUT, which forces
   // the DUT to be the only parent (N is a deterministic set, not a random one).
   // CHILD1_ON..CHILD3_ON are rewritten in place by tools/apply_config_perf.py.
   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      // root (0xe4) and DUT (0xcd) accept everything
      case 0xe4:
      case 0xcd:
         returnVal = TRUE;
         break;
      // child 1 (CHILD1_ON) -- accepts ONLY the DUT
      case 0x18:
         if (ieee802514_header->src.addr_64b[7] == 0xcd) { returnVal = TRUE; }
         break;
      // child 2 (CHILD2_ON) -- accepts ONLY the DUT
      case 0xba:
         if (ieee802514_header->src.addr_64b[7] == 0xcd) { returnVal = TRUE; }
         break;
      // child 3 (CHILD3_ON) -- accepts ONLY the DUT
      case 0xd9:
         if (ieee802514_header->src.addr_64b[7] == 0xcd) { returnVal = TRUE; }
         break;
      default:
         // unknown board: behave like root/DUT so the network is not broken
         returnVal = TRUE;
         break;
   }
   return returnVal;
#else
   return TRUE;
#endif /* OPENWSN_FORCETOPOLOGY_C */
}

//=========================== private =========================================
