Modifications in GNB Platform Source Code

a) Disable AVS  
   (The change needed is in `SRC/u-boot/board/edgeq/raptor2.c`.  
   Comment out the `cfg_avs_pmic(avs_idx);` function call OR update the function to reflect the PMIC I2C address.)

b) Use eMMC instead of SD card  

   ```
   &emmc {
       status = "ok";
       /delete-property/ mmc_coherent;
       memory-region = <&emmc_memory>;
   };

   &sdcard {
       status = "disabled";
   };
   ```

   (Update the DTS files: `arch/arm64/boot/dts/edgeq/raptor2-B0-hawk.dts` and `arch/arm64/boot/dts/edgeq/raptor2-B0-hawk-f.dts`.)

c) GPS Modifications (using Quectel GPS module instead of    u-blox)  
   The Quectel UART (serial) operates at 460800, whereas the u-blox on the Hawk board operates at 38400.  

   ```
   &bss_serial1 {
       status = "ok";
       clock-frequency = <750000000>;
       gnss {
           status = "okay";
           current-speed = <38400>;
       };
   };
   ```

   Include a similar block in your board-specific DTS file and change the `current-speed` field to:  
   `current-speed = <460800>;`

d) Disable eth1  
   Kalinga has eth0, so disable eth1.

e) Update timezone, resize, and remove visual mode  
   Edit `/etc/profile` and add the following at the end:

   ```
   resize
   export PS1="\w$ "
   sudo timedatectl set-timezone Asia/Kolkata
   echo "alias vi='vim -u NONE -N'" >> ~/.bashrc
   source ~/.bashrc
   ```

f) Update Linux version  
   Edit `./localversion-rt` and replace `-rt1` with `-rt11-GNB-PLFM-B0-v1.5.0-RC2.1`.

g) Kalinga delta files  
   1. `l1_rfic_seq.yaml` (update PA and LMA enable sequence suitable for Kalinga GPIO)  
   2. `eth0.network` (remove DHCP, set a fixed IP address, and add a global route)  
   3. `ems.service` (add EMS and EMS-web-app services)  
   4. PTP/GPS timing settings (`sync_timing_driver.conf`, `time_syncmgr.conf`)  
   5. `rpacpd` (redirect PCAP packets to the host for Wireshark)  
   6. `raptor2_wrapper.service` (add service for `raptor2_wrapper.sh`)  
   7. `raptor2_wrapper.sh` (run EMS using pinned code)  
   8. `libadrv9029.so` (modified specifically for the Kalinga board)

