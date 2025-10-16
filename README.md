# About: Multicore Demo
Ensembe dual-core demo for STANDBY Mode added:
    - RTSS-HE logs are on LP-UART (P7_6 and P7_7)
    - RTSS-HP logs are on UART4 (P9_1 and P9_2)

At first power-on, both cores are booted by the Secure Enclave. Since
the HP core detects no wake events, it powers down immediately. The HE
core also detects no wake events and continues to perform the first-time
system setup. It configures the LPTIMER as its wakeup source and puts
the MCU in global STANDBY Mode. The MCU stays in STANDBY Mode until the
next LPTIMER expiration (configured for 1000 ms).

Each LPTIMER expiration wakes up the HE core only. The HE core spends
some time awake then returns the MCU to STANDBY Mode. After ten LPTIMER
wake events the HE core uses the Message Handling Unit (MHU) to wake up
the HP core. While waiting for a response from the HP core, the HE will
enter deep sleep (subsystem stays on).

When the HP core receives a message via the MHU, it will run for some
time in a while(1) loop before returning a response to the HE core. If
the HP does not respond in time the HE core will timeout and reset it.

# Building the binaries
Open the directory using VSCode and switch to the CMSIS View (CTRL+SHIFT+ALT+S).
Click the gear icon labeled "Manage Solution Settings". Here you will choose the
Active Target. If you are planning to run this demo on the Devkit-e7, then you
will want to build binaries for the E7-HE and E7-HP. Otherwise, for the Devkit-e8,
then build binaries for the E8-HE and E8-HP. Use this Manage Solution tab to
switch between Target Types and use the hammer icon in the CMSIS View to build
the application.

After the binaries are built, switch to the Explorer View (CTRL+SHIFT+E).
The binaries will be located in the out directory, for example:
    out/
        app_he/E7-HE/release/app_he.bin
        app_hp/E7-HP/release/app_hp.bin

# Programming the binaries
Use the below json to configure your ATOC.

{
  "DEVICE": {
    "disabled" : false,
    "binary": "app-device-config-no-hfxo.json",
    "version" : "1.0.0",
    "signed": false
  },
  "HE_TCM": {
    "disabled" : false,
    "binary": "M55_HE_img.bin",
    "loadAddress": "0x58000000",
    "version": "1.0.0",
    "cpu_id": "M55_HE",
    "flags": ["boot","load"],
    "signed": false
  },
  "HP_MRAM": {
    "disabled" : false,
    "binary": "M55_HP_img.bin",
    "mramAddress": "0x80200000",
    "version": "1.0.0",
    "cpu_id": "M55_HP",
    "flags": ["boot"],
    "signed": false
  }
}
