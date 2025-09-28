/*
 *
 * EdgeQ Inc.
 *
 * Raptor2 WLAN Device Driver
 * MAC80211 Callback Implementation
 *
 */

struct ieee80211_ops raptor2_ops = {
	.start              		= raptor2_start,
	.stop               		= raptor2_stop,
	.add_interface      		= raptor2_add_interface,
	.change_interface   		= raptor2_change_interface,
	.remove_interface   		= raptor2_remove_interface,
	.config             		= raptor2_config,
	.start_ap			= raptor2_start_ap,
	.stop_ap			= raptor2_stop_ap,
	.configure_filter		= raptor2_configure_filter,
	.set_key			= raptor2_set_key,
	.set_default_unicast_key	= raptor2_set_default_unicast_key,
	.sta_state			= raptor2_sta_state,
	.sta_notify			= raptor2_sta_notify,
	.sta_rc_update			= raptor2_sta_rc_update,
	.conf_tx            		= raptor2_conf_tx,
	.set_rts_threshold 		= raptor2_set_rts_threshold,
	.set_frag_threshold 		= raptor2_set_frag_threshold,
	.set_antenna 			= raptor2_set_antenna,
	.get_antenna 			= raptor2_get_antenna,
	.hw_scan			= raptor2_hw_scan,
	.remain_on_channel		= raptor2_remain_on_channel,
	.cancel_remain_on_channel	= raptor2_cancel_remain_on_channel,
	.add_chanctx			= raptor2_add_chanctx,
	.remove_chanctx			= raptor2_remove_chanctx,
	.change_chanctx			= raptor2_change_chanctx,
	.assign_vif_chanctx		= raptor2_assign_vif_chanctx,
	.unassign_vif_chanctx		= raptor2_unassign_vif_chanctx,
	.switch_vif_chanctx		= raptor2_switch_vif_chanctx,
	.channel_switch_beacon		= raptor2_channel_switch_beacon,
	.wake_tx_queue      		= raptor2_wake_tx_queue,
	.tx                 		= raptor2_tx,
	.flush              		= raptor2_flush,
	.ampdu_action       		= raptor2_ampdu_action,
	.allow_buffered_frames		= raptor2_allow_buffered_frames,
	.release_buffered_frames 	= raptor2_release_buffered_frames,
	.bss_info_changed   		= raptor2_bss_info_changed,
	.get_tsf            		= raptor2_get_tsf,
	.set_tsf            		= raptor2_set_tsf,
	.reset_tsf          		= raptor2_reset_tsf,
	.get_survey         		= raptor2_get_survey,
	.rfkill_poll        		= raptor2_rfkill_poll_state,
	.tx_frames_pending  		= raptor2_tx_frames_pending,
	.tx_last_beacon     		= raptor2_tx_last_beacon,
	.get_stats          		= raptor2_get_stats,

#if defined(CONFIG_MAC80211_DEBUGFS) && defined(CONFIG_RAPTOR2_STATION_STATISTICS)
	.sta_add_debugfs    		= raptor2_sta_add_debugfs,
#endif
	.sw_scan_start      		= raptor2_sw_scan_start,
	.sw_scan_complete   		= raptor2_sw_scan_complete,
	.get_txpower        		= raptor2_get_txpower,
};
