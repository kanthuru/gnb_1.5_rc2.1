import sys
import time
import shlex
import subprocess
import configparser
import logging
import signal

DAEMON_RESTART_MAX_CNT = 10

#
# Configuration abstraction
#
class timesync_config:
    def __init__(self, cfg_file_name):
        self.cfg_file_name = cfg_file_name
        self.config = None
        """
        time & sync solution (e.g. EdgeQ, AccuTime, etc)
        """
        self.sync_type = None
        self.sync_profile = None
        self.min_warmup_time = None
        self.max_holdover_time = None
        self.eth_intf = None
        self.ptp_dev = None
        self.servo_conf_file = None
        self.servo_model = None
        self.first_step_threshold = None
        self.step_threshold = None
        self.lock_threshold = None
        self.locked_stable_spec = None
        self.verbose = None
        self.syslog = None
        self.gps_conf_file = None
        self.gps_device = None
        self.gps_log = None
        self.gps_socket_file = None
        self.ptp_profile = None
        self.ptp_domain = None
        self.at_ptp_profile = None

    def update_servo_config(self):
        servo_config = configparser.ConfigParser()
        servo_config.read(self.servo_conf_file)

        # Update servo config params
        if self.eth_intf is not None:
            servo_config['global']['eth_intf'] = \
                self.eth_intf
        if self.ptp_dev is not None:
            servo_config['global']['device'] = \
                self.ptp_dev
        if self.sync_type is not None:
            servo_config['global']['mode'] = \
                self.sync_type
        if self.servo_model is not None:
            servo_config['global']['model'] = \
                self.servo_model
        if self.min_warmup_time is not None:
            servo_config['global']['min_warmup_time'] = \
                self.min_warmup_time
        if self.max_holdover_time is not None:
            servo_config['global']['max_holdover_time'] = \
                self.max_holdover_time
        if self.first_step_threshold is not None:
            servo_config['global']['first_step_threshold'] = \
                self.first_step_threshold
        if self.step_threshold is not None:
            servo_config['global']['step_threshold'] = \
                self.step_threshold
        if self.lock_threshold is not None:
                servo_config['global']['lock_threshold'] = \
                self.lock_threshold
        if self.locked_stable_spec is not None:
                servo_config['global']['locked_stable_spec'] = \
                self.locked_stable_spec
        if self.verbose is not None:
            servo_config['global']['verbose'] = self.verbose
        if self.syslog is not None:
            servo_config['global']['syslog'] = self.syslog
        if self.gps_socket_file is not None:
            servo_config['gps']['gpsd_socket'] = \
                self.gps_socket_file
        if self.ptp_domain is not None:
            servo_config['ptp']['domain'] = \
                self.ptp_domain

        with open(self.servo_conf_file, 'w') \
        as old_servo_config:
            servo_config.write(old_servo_config)

    def update_gps_config(self):
        gps_config = configparser.ConfigParser()
        gps_config.read(self.gps_conf_file)

        # Update gps config params
        if self.gps_device is not None:
            gps_config['global']['device'] = \
                self.gps_device
        if self.eth_intf is not None:
            gps_config['global']['eth_intf'] = \
                self.eth_intf
        if self.gps_socket_file is not None:
            gps_config['global']['socket_file'] = \
                self.gps_socket_file

        with open(self.gps_conf_file, 'w') \
        as old_gps_config:
            gps_config.write(old_gps_config)

    def process_config(self):
        self.config = configparser.ConfigParser()
        self.config.read(self.cfg_file_name)

        # Global section
        if self.config.has_section('global'):
            self.sync_type = self.config['global']['source']
            self.eth_intf = self.config['global']['eth_intf']
            self.ptp_dev = self.config['global']['ptp_dev']
            self.min_warmup_time = \
                self.config['global']['min_warmup_time']
            self.max_holdover_time = \
                self.config['global']['max_holdover_time']

        # Servo section
        if self.config.has_section('servo'):
            self.servo_conf_file = \
                self.config['servo']['servo_conf_file']
            self.servo_model = self.config['servo']['model']
            self.first_step_threshold = \
                self.config['servo']['first_step_threshold']
            self.step_threshold = self.config['servo']['step_threshold']
            self.lock_threshold = self.config['servo']['lock_threshold']
            self.locked_stable_spec = \
                self.config['servo']['locked_stable_spec']
            self.verbose = self.config['servo']['verbose']
            self.syslog = self.config['servo']['syslog']

        # GPS section
        if self.config.has_section('gps'):
            self.gps_conf_file = \
                self.config['gps']['gps_conf_file']
            self.gps_device = self.config['gps']['device']
            self.gps_log = self.config['gps']['log']
            self.gps_socket_file = \
                self.config['gps']['socket_file']

        # PTP section
        if self.config.has_section('ptp'):
            self.sync_profile = self.config['ptp']['profile']
            self.ptp_profile = self.config['ptp']['profile']
            self.ptp_domain = self.config['ptp']['domain']

        if self.config.has_section('accutime'):
            self.at_ptp_profile = self.config['accutime']['profile']

        # Update raptor2_servo and gpsd config files
        self.update_servo_config()
        self.update_gps_config()

#
# Sync Daemon Implementation
#
class timesync_daemon:
    def __init__(self, type, profile):
        self.sync_type = type
        self.sync_profile = profile
        self.sync_daemon = None
        self.syslog_daemon = None
        self.daemon_log = None
        self.restart_cnt = 0

    def start_raptor2_servo(self, servo_conf_file: str):
        """EdgeQ Time & Sync Servo"""
        syslog_daemon_cmd = "syslogd"
        syslog_daemon_cmd_split = shlex.split( syslog_daemon_cmd )
        self.syslog_daemon = subprocess.Popen( syslog_daemon_cmd_split )

        self.daemon_log = open("/tmp/r2_servo.log", "w")

        r2_servo_cmd = "raptor2_servo -f " + servo_conf_file
        r2_servo_cmd_split = shlex.split( r2_servo_cmd )
        self.sync_daemon = subprocess.Popen( r2_servo_cmd_split, \
                                             stdout=self.daemon_log, \
                                             stderr=self.daemon_log )

    def start_ptp4l(self):
        """Linux ptp4l (open source)"""
        self.daemon_log = open("/tmp/ptp.log", "w")

        ptp_cmd = "ptp4l -f /etc/ptp4l/ptp4l_" + self.sync_profile + ".conf  "
        ptp_cmd_split = shlex.split( ptp_cmd )
        self.sync_daemon = subprocess.Popen( ptp_cmd_split, \
                                             stdout=self.daemon_log, \
                                             stderr=self.daemon_log )

    def start_gpsd(self, gps_conf_file: str):
        """EdgeQ GPS daemon"""
        self.daemon_log = open("/tmp/gps.log", "w")

        gpsd_cmd = "gpsd -f " + gps_conf_file
        gpsd_cmd_split = shlex.split( gpsd_cmd )
        self.sync_daemon = subprocess.Popen( gpsd_cmd_split, \
                                             stdout=self.daemon_log, \
                                             stderr=self.daemon_log )

    def start_accutime_driver(self) -> int:
        """AccuTime Driver"""
        at_drv_cmd = "sync_timing_core_driver"
        at_drv_cmd_split = shlex.split( at_drv_cmd )
        self.sync_daemon = subprocess.Popen( at_drv_cmd_split, \
                                             stdout=subprocess.DEVNULL,
                                             stderr=subprocess.STDOUT )
        return 0

    def start_accutime_ptp2stack(self, at_ptp_profile: str) -> int:
        """AccuTime PTP stack"""
        at_ptp_cmd = "sync_timing_ptp2stack " + at_ptp_profile
        at_ptp_cmd_split = shlex.split( at_ptp_cmd )
        self.sync_daemon = subprocess.Popen( at_ptp_cmd_split, \
                                             stdout=subprocess.DEVNULL,
                                             stderr=subprocess.STDOUT )

        return 0

    def start_timesync_daemon(self, servo_conf_file: str, \
                              gps_conf_file: str, \
                              at_ptp_profile: str) -> int:
        """
        Currently supported modes: GPS, PTP, GPS+PTP

        Returns:
        int: 0 on success, -1 if failed
        """
        if self.sync_type == "ptp":
            self.start_ptp4l()
        elif self.sync_type == "gps":
            self.start_gpsd(gps_conf_file)
        elif self.sync_type == "r2_servo":
            self.start_raptor2_servo(servo_conf_file)
        elif self.sync_type == "accutime_driver":
            self.start_accutime_driver()
            time.sleep(5)
        elif self.sync_type == "accutime_ptp2stack":
            self.start_accutime_ptp2stack(at_ptp_profile)
        else:
            logging.info(self.sync_type + " not supported")

        return 0 if self.get_daemon_status() == None else -1

    def get_daemon_status(self):
        status = self.sync_daemon.poll()
        return status

    def kill_daemon(self):
        try:
            if self.syslog_daemon is not None:
                # Kill syslogd
                self.syslog_daemon.kill()
                logging.info("Killed syslogd")

            self.sync_daemon.kill()
            self.sync_daemon = None
            self.daemon_log.close()
            self.daemon_log = None
            logging.info("Killed " + self.sync_type)
        except AttributeError:
            # Already killed
            pass

# Time Sync Service implementation
class timesync_service:
    def __init__(self, cfg_file):
        self.sync_cfg = None
        self.cfg_file = cfg_file
        self.daemons = []

    def init_timesync_service(self):
        self.sync_cfg = timesync_config( self.cfg_file )

        self.sync_cfg.process_config()

        sync_source = self.sync_cfg.sync_type

        if sync_source == "ptp":
            sync_daemon = timesync_daemon( "ptp", self.sync_cfg.sync_profile )
            self.daemons.append( sync_daemon )
        elif sync_source == "gps":
            sync_daemon = timesync_daemon( "gps", None )
            self.daemons.append( sync_daemon )
        elif sync_source == "gps+ptp":
            sync_daemon = timesync_daemon( "gps", None )
            self.daemons.append( sync_daemon )
            sync_daemon = timesync_daemon( "ptp", self.sync_cfg.sync_profile )
            self.daemons.append( sync_daemon )
        elif sync_source == "accutime":
            # Start AccuTime drive first
            sync_daemon = timesync_daemon( "accutime_driver", None )
            self.daemons.append( sync_daemon )

            # AccuTime PTP stack
            sync_daemon = timesync_daemon( "accutime_ptp2stack", \
                                            self.sync_cfg.sync_profile )
            self.daemons.append( sync_daemon )

            # Remove log files if clear_logfile_startup is set to 1
            parser = configparser.ConfigParser()
            parser.read("/etc/sync_timing_driver.conf")

            if parser.has_section('core'):
                if (parser['core']['clear_logfile_startup'] == '1'):
                    try:
                        with open("/var/log/synctimingdriver.log", 'w') as file:
                            file.write('')
                    except Exception as e:
                       logging.error(f"Failed to clear synctimingdriver.log: {e}")

            # Get dctcxo output enable (OE) control register value
            # 0x60 corresponds to the address of SiT5357 DCTCXO
            # 0x1 is the offset for OE control
            i2cget_dctxo_oe_detect = "i2cget -y 0 0x60 0x1 w"
            i2cget_split = shlex.split( i2cget_dctxo_oe_detect )
            oe_control_reg = subprocess.run(i2cget_split, capture_output=True, text=True)

            # Here, oe_control_reg is of form "0x0004\n" so we expect last but one char to be 4
            if len(oe_control_reg.stdout) > 1 and oe_control_reg.stdout[-2] != '4':
                logging.info('dctcxo oe not set, setting now')

                # Set the dctcxo output enable value to 1 
                i2cset_dctxo_oe_set = "i2cset -y 0 0x60 0x1 0x0004 w"
                i2cset_split = shlex.split( i2cset_dctxo_oe_set )
                subprocess.run(i2cset_split)
        else:
            logging.error("Unknown source " + sync_source)

        if sync_source != "accutime":
            """
            Force DPLL out of holdover if using EdgeQ solution
            """
            logging.info("Force DPLL out of holdover and clear status")

            # PLL A
            dpll_force_out_of_ho = "dpll_cmd /dev/spidev1.0 500000 wr 0x25 0x2 0x0"
            dpll_cmd_split = shlex.split( dpll_force_out_of_ho )
            subprocess.run(dpll_cmd_split)

            # Clear status
            dpll_clear_status = "dpll_cmd /dev/spidev1.0 500000 wr 0x2a"
            dpll_cmd_split = shlex.split( dpll_clear_status )
            subprocess.run(dpll_cmd_split)

            # Start raptor2_servo for all the modes
            logging.info("Append EdgeQ servo to timesync daemons")
            sync_daemon = timesync_daemon( "r2_servo", None )
            self.daemons.append( sync_daemon )

    def start_timesync_service(self):
        for sync_daemon in self.daemons:
            ret = sync_daemon.start_timesync_daemon( \
                    self.sync_cfg.servo_conf_file, \
                        self.sync_cfg.gps_conf_file, \
                            self.sync_cfg.at_ptp_profile )
            if ret != 0:
                logging.error("Failed to start " + sync_daemon.sync_type)
            else:
                logging.info(sync_daemon.sync_type + " started!")

    def monitor_timesync_service(self):
        """
        Monitor time & sync processes. If any process dies,
        restart it. If failed to restart 10 consecutive times,
        exit this function
        """
        while True:
            for sync_daemon in self.daemons:
                status = sync_daemon.get_daemon_status()

                """
                If one daemon fails, kill all daemons and restart again
                """
                if status != None:
                    logging.error(sync_daemon.sync_type + " failed!!!")
                    self.kill_timesync_service()
                    time.sleep( 1 )
                    self.start_timesync_service()
                    break
            try:
                time.sleep( 5 )
            except KeyboardInterrupt as kbIntr:
                logging.info("Exiting " + self.__class__.__name__)
                return

    def kill_timesync_service(self):
        for sync_daemon in reversed(self.daemons):
            logging.info("Stopping " + sync_daemon.sync_type)
            sync_daemon.kill_daemon()
            time.sleep( 2 )

def sig_handler(signum, frame):
    service.kill_timesync_service()
    sys.exit(0)

signal.signal(signal.SIGTERM, sig_handler)

def main():
    global service
    logging.basicConfig(filename='/var/log/timesync_service.log', \
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', \
                        level=logging.INFO, filemode='w')
    logging.info("Starting timesync_service.py!")
    service = timesync_service( "/etc/time_syncmgr/time_syncmgr.conf" )

    service.init_timesync_service()
    service.start_timesync_service()
    service.monitor_timesync_service()

if __name__ == "__main__":
    main()

