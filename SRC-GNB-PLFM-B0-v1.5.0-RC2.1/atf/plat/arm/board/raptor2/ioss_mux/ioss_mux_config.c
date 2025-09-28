#include <lib/mmio.h>
#include "ioss_mux_config.h"

#if B0_BUILD
struct ioss_muxing_info_t ioss_mux_table[MAX_IOSS_OUTPUT_EVENTS] = {
	{0, 123},		/* xgmac0_ptp_pps_o_0 */
	{1, 175},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_00 */
	{2, 176},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_01 */
	{3, 177},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_02 */
	{4, 178},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_03 */
	{5, 166},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_00 */
	{6, 167},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_01 */
	{7, 168},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_02 */
	{8, 169},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_03 */
	{9, 157},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_00 */
	{10, 158},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_01 */
	{11, 159},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_02 */
	{12, 160},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_03 */
	{13, 148},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_00 */
	{14, 149},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_01 */
	{15, 150},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_02 */
	{16, 151},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_03 */
	{17, 191},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_00 */
	{18, 192},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_01 */
	{19, 193},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_02 */
	{20, 194},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_03 */
	{21, 170},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_00 */
	{22, 171},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_01 */
	{23, 172},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_02 */
	{24, 173},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_03 */
	{25, 161},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_00 */
	{26, 162},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_01 */
	{27, 163},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_02 */
	{28, 164},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_03 */
	{29, 152},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_00 */
	{30, 153},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_01 */
	{31, 154},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_02 */
	{32, 155},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_03 */
	{33, 147},		/* XLGMAC sbd_intr_0 */
	{34, 156},		/* XGMAC3 sbd_intr_0 */
	{35, 165},		/* XGMAC2 sbd_intr_0 */
	{36, 174},		/* XGMAC1 sbd_intr_0 */
	{37, 207},		/* XGMAC0 sbd_intr_0 */
	{38, 131},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_00 */
	{39, 132},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_01 */
	{40, 133},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_02 */
	{41, 134},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_03 */
	{42, 135},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_04 */
	{43, 136},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_05 */
	{44, 137},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_06 */
	{45, 138},		/* ptn_e56_u_xlmac_sbd_perch_tx_intr_o_07 */
	{46, 139},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_00 */
	{47, 140},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_01 */
	{48, 141},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_02 */
	{49, 142},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_03 */
	{50, 143},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_04 */
	{51, 144},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_05 */
	{52, 145},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_06 */
	{53, 146},		/* ptn_e56_u_xlmac_sbd_perch_rx_intr_o_07 */
	{54, 179},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_04 */
	{55, 180},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_05 */
	{56, 181},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_06 */
	{57, 182},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_07 */
	{58, 195},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_04 */
	{59, 196},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_05 */
	{60, 197},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_06 */
	{61, 198},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_07 */
	{62, 13},		/* pcie0_int_evt_deassert_intd_grt */
	{63, 14},		/* pcie0_int_evt_deassert_intc_grt */
	{64, 15},		/* pcie0_int_evt_deassert_intb_grt */
	{65, 16},		/* pcie0_int_evt_deassert_inta_grt */
	{66, 17},		/* pcie0_int_evt_assert_intd_grt */
	{67, 18},		/* pcie0_int_evt_assert_intc_grt */
	{68, 19},		/* pcie0_int_evt_assert_intb_grt */
	{69, 20},		/* pcie0_int_evt_assert_inta_grt */
	{70, 29},		/* pcie_evt_intd_deasserted */
	{71, 30},		/* pcie_evt_intc_deasserted */
	{72, 31},		/* pcie_evt_intb_deasserted */
	{73, 32},		/* pcie_evt_inta_deasserted */
	{74, 33},		/* pcie_evt_intd_asserted */
	{75, 34},		/* pcie_evt_intc_asserted */
	{76, 35},		/* pcie_evt_intb_asserted */
	{77, 36},		/* pcie_evt_inta_asserted */
	{78, 362},		/* pcie_msi_ctrl_int */
	{79, 50},		/* pcie0_int_evt_cfg_aer_rc_err_int */
	{80, 43},		/* pcie0_int_evt_cfg_pme_int */
	{81, 274},		/* pcie_dma_int[34] / usb_interrupt[0] */
	{82, 275},		/* pcie_dma_int[35] / usb_interrupt[1] */
	{83, 276},		/* pcie_dma_int[36] / usb_interrupt[2] */
	{84, 277},		/* pcie_dma_int[37] / usb_interrupt[3] */
	{85, 78},		/* pcie0_int_evt_radm_intd_asserted 	*/
	{86, 77},		/* pcie0_int_evt_radm_intc_asserted 	*/
	{87, 76},		/* pcie0_int_evt_radm_intb_asserted 	*/
	{88, 75},		/* pcie0_int_evt_radm_inta_asserted 	*/
	{89, 74},		/* pcie0_int_evt_radm_intd_deasserted 	*/
	{90, 73},		/* pcie0_int_evt_radm_intc_deasserted 	*/
	{91, 72},		/* pcie0_int_evt_radm_intb_deasserted 	*/
	{92, 71},		/* pcie0_int_evt_radm_inta_deasserted 	*/
	{93, 94},		/* ccix_int_evt_radm_inta_asserted 		*/
	{94, 93},		/* ccix_int_evt_radm_intb_asserted 		*/
	{95, 92},		/* ccix_int_evt_radm_intc_asserted 		*/
	{96, 91},		/* ccix_int_evt_radm_intd_asserted 		*/
	{97, 90},		/* ccix_int_evt_radm_inta_deasserted 	*/
	{98, 89},		/* ccix_int_evt_radm_intb_deasserted 	*/
	{99, 88},		/* ccix_int_evt_radm_intc_deasserted 	*/
	{100, 87},		/* ccix_int_evt_radm_intd_deasserted 	*/
	{101, 108},		/* ccix_int_evt_cfg_aer_rc_err_int 		*/
	{102, 101},		/* ccix_int_evt_cfg_pme_int 			*/
	{103, IOSS_EVENT_UNUSED},
	{104, IOSS_EVENT_UNUSED},
	{105, IOSS_EVENT_UNUSED},
	{106, IOSS_EVENT_UNUSED},
	{107, IOSS_EVENT_UNUSED},
	{108, IOSS_EVENT_UNUSED},
	{109, IOSS_EVENT_UNUSED},
	{110, IOSS_EVENT_UNUSED},
	{111, IOSS_EVENT_UNUSED},
	{112, IOSS_EVENT_UNUSED},
	{113, IOSS_EVENT_UNUSED},
	{114, IOSS_EVENT_UNUSED},
	{115, IOSS_EVENT_UNUSED},
	{116, IOSS_EVENT_UNUSED},
	{117, IOSS_EVENT_UNUSED},
	{118, IOSS_EVENT_UNUSED},
	{119, IOSS_EVENT_UNUSED},
	{120, IOSS_EVENT_UNUSED},
	{121, IOSS_EVENT_UNUSED},
	{122, IOSS_EVENT_UNUSED},
	{123, IOSS_EVENT_UNUSED},
	{124, IOSS_EVENT_UNUSED},
	{125, IOSS_EVENT_UNUSED},
	{126, IOSS_EVENT_UNUSED},
	{127, IOSS_EVENT_UNUSED},
};
#else
struct ioss_muxing_info_t ioss_mux_table[MAX_IOSS_OUTPUT_EVENTS] = {
	{0, 225},		/* usb_interrupt[0] */
	{1, 226},		/* usb_interrupt[1] */
	{2, 109},		/* USB PHY DRVVBUS0 */
	{3, 123},		/* ptp_pps_o */
	{4, 2},			/* smlh_link_up */
	{5, IOSS_EVENT_UNUSED},	/* TBD msi_ctrl_int */
	{6, 27},		/* hp_int */
	{7, 36},		/* pcie0_int_evt_radm_inta_asserted */
	{8, 35},		/* pcie0_int_evt_radm_intb_asserted */
	{9, 34},		/* pcie0_int_evt_radm_intc_asserted */
	{10, 33},		/* pcie0_int_evt_radm_intd_asserted */
	{11, 234},		/* CCIX edma_int[0] */
	{12, 235},		/* CCIX edma_int[1] */
	{13, 233},		/* CCIX radm_inta_asserted */
	{14, 232},		/* CCIX radm_intb_asserted */
	{15, 231},		/* CCIX radm_intc_asserted */
	{16, 230},		/* CCIX radm_intd_asserted */
	{17, 240},		/* PCIE edma_int[0] */
	{18, 241},		/* PCIE edma_int[1] */
	{19, 242},		/* PCIE edma_int[2] */
	{20, 243},		/* PCIE edma_int[3] */
	{21, 244},		/* PCIE edma_int[4] */
	{22, 248},		/* PCIE edma_int[8] */
	{23, 249},		/* PCIE edma_int[9] */
	{24, 250},		/* PCIE edma_int[10] */
	{25, 251},		/* PCIE edma_int[11] */
	{26, 252},		/* PCIE edma_int[12] */
	{27, 175},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_00 */
	{28, 176},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_01 */
	{29, 177},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_02 */
	{30, 178},		/* ptn_xgmac_xgmac0_sbd_perch_tx_intr_o_03 */
	{31, 166},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_00 */
	{32, 167},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_01 */
	{33, 168},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_02 */
	{34, 169},		/* ptn_xgmac_xgmac1_sbd_perch_tx_intr_o_03 */
	{35, 157},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_00 */
	{36, 158},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_01 */
	{37, 159},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_02 */
	{38, 160},		/* ptn_xgmac_xgmac2_sbd_perch_tx_intr_o_03 */
	{39, 148},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_00 */
	{40, 149},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_01 */
	{41, 150},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_02 */
	{42, 151},		/* ptn_xgmac_xgmac3_sbd_perch_tx_intr_o_03 */
	{43, 191},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_00 */
	{44, 192},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_01 */
	{45, 193},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_02 */
	{46, 194},		/* ptn_xgmac_xgmac0_sbd_perch_rx_intr_o_03 */
	{47, 170},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_00 */
	{48, 171},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_01 */
	{49, 172},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_02 */
	{50, 173},		/* ptn_xgmac_xgmac1_sbd_perch_rx_intr_o_03 */
	{51, 161},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_00 */
	{52, 162},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_01 */
	{53, 163},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_02 */
	{54, 164},		/* ptn_xgmac_xgmac2_sbd_perch_rx_intr_o_03 */
	{55, 152},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_00 */
	{56, 153},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_01 */
	{57, 154},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_02 */
	{58, 155},		/* ptn_xgmac_xgmac3_sbd_perch_rx_intr_o_03 */
	{59, 147},		/* XLGMAC sbd_intr_0 */
	{60, 156},		/* XGMAC3 sbd_intr_0 */
	{61, 165},		/* XGMAC2 sbd_intr_0 */
	{62, 174},		/* XGMAC1 sbd_intr_0 */
	{63, 207},		/* XGMAC0 sbd_intr_0 */
};
#endif

/*
 * IOSS Module Initialization.
 *    - Program IOSS Mux based on muxing table.
 */
void ioss_mux_init(void)
{
	uint8_t i;
	uintptr_t addr;

	for(i = 0; i < MAX_IOSS_OUTPUT_EVENTS; i++) {
		if(ioss_mux_table[i].input < MAX_IOSS_INPUT_EVENTS) {
			addr = IOSS_MUX_PRG_BASE_ADDR + (4 * ioss_mux_table[i].output);
			mmio_write_32(addr, ioss_mux_table[i].input);
		}
	}
}
