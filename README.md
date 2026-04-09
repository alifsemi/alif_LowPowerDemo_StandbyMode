# About: MCU STANDBY Mode & MHU Multicore Demo
For Ensembe E1, E3, E4, E5, E6, E7 or E8
  - RTSS-HE logs are on LP-UART (RX P7_6 and TX P7_7)
  - RTSS-HP logs are on UART4 (RX P12_1 and TX P12_2)

For Ensemble E1C or Balletto B1
  - RTSS-HE logs are on LP-UART (RX P2_0 and TX P7_1)

Changing the UART instance and pinmux is done via the retarget_config.h file.
```
    .
    ├── ...
    ├── app_he/
    │   ├── RTE/
    │   │   ├── Services/
    │   |   │   ├── AE1C1F4051920PH_M55_HE/retarget_config.h
    │   |   │   ├── AE722F80F55D5LS_M55_HE/retarget_config.h
    │   |   |   └── AE822FA0E5597LS0_M55_HE/retarget_config.h
    │   │   └── ...
    │   └── ...
    ├── app_hp/
    │   ├── RTE/
    │   │   ├── Services/
    │   |   │   ├── AE722F80F55D5LS_M55_HP/retarget_config.h
    │   |   |   └── AE822FA0E5597LS0_M55_HP/retarget_config.h
    │   │   └── ...
    │   └── ...
    └── ...
```

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

After the binaries are built, switch to the Explorer View (CTRL+SHIFT+E). For the
dual-core application demos, both binaries for HP and HE should be built prior to
programming the binaries. For a single-core application, only one of the binaries
is needed. The binaries will be located in the out directory, for example:
```
.
├── ...
├── out/
│   ├── app_he/
│   |   ├── E1C-HE/
│   |   |   ├── debug/app_he.bin
│   |   |   └── release/app_he.bin
│   |   ├── E7-HE/
│   |   |   ├── debug/app_he.bin
│   |   |   └── release/app_he.bin
│   |   └── E8-HE/
│   |       ├── debug/app_he.bin
│   |       └── release/app_he.bin
│   └── app_hp/
│       ├── E7-HP/
│       |   ├── debug/app_hp.bin
│       |   └── release/app_hp.bin
│       └── E8-HP/
│           ├── debug/app_hp.bin
│           └── release/app_hp.bin
└── ...
```


# Debugging the binaries
- Use the SE tools integration to install app-cpu-stubs-hfrc.json
- Switch to the Run and Debug view (CTRL+SHIFT+D)
- Choose "Debug" from the dropdown menu
- press F5 to start the debug session


# Programming the binaries
Use the SE tools integration to install M55_HP_HE_cfg.json. Do this by executing the task "Alif: Program with Security Toolkit (dual core)". If the target is not responding, it may help to execute the task "Alif: put chip in maintenance mode".

To program the binaries manually, use the below json to configure your ATOC. Copy the binaries to the app-release-exec/build/images folder and proceed with the usual steps of generating the ATOC and writing to MRAM.

```
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
    "mramAddress": "0x80000000",
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
```


# Power Measurement on Alif DevKit
Refer to the power measurement points described in the [aiPM Examples User Guide](https://github.com/alifsemi/alif_ensemble-vscode-aiPMExamples/blob/main/Documentation/aiPM_Examples.md)


# Other Notes
It is a good idea to mark launch.json and tasks.json as read-only files so that the Arm CMSIS Solution extention does not overwrite these files needlessly.

To erase the application, execute the task "Alif: put chip in maintenance mode and erase".