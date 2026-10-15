# HG-UV98 Community Resources

## Groups.io - HG-UV98 Users

**Main Discussion Group:** https://groups.io/g/HG-UV98-users
- **Members:** 132+
- **Topics:** 106+
- **Description:** A group for users of the HG-UV98 APRS Handheld

### Email
- **Post:** HG-UV98-users@groups.io
- **Subscribe:** HG-UV98-users+subscribe@groups.io
- **Unsubscribe:** HG-UV98-users+unsubscribe@groups.io
- **Owner:** HG-UV98-users+owner@groups.io
- **Help:** HG-UV98-users+help@groups.io

### Recent Topics of Interest
1. **Firmware flash question** (#113261593) - Issues with flashing firmware
2. **File for latest firmware** (#110361877) - Latest firmware file requests
3. **Linux config software** (#110102221) - Cross-platform programming tools
4. **Source code available - Github link** (#109570377) - 12 messages about source code
5. **Source code** (#109550170) - 3 messages about source code availability

## GitHub Repositories

### Main Repositories
| Repo | Description | Stars |
|------|-------------|-------|
| [marrold/HG-UV98](https://github.com/marrold/HG-UV98) | Original community repo, issues & resources | Base |
| [pwnj0hn/HG-UV98](https://github.com/pwnj0hn/HG-UV98) | Firmware binaries + software + manual | Mirrors marrold |
| [agamez/hg-uv98-firmware](https://github.com/agamez/hg-uv98-firmware) | Keil source code project | - |

### Related Firmware
- **KT2KT Firmware:** V1.0.1_190619-KT.zip (UI improvements)
- **APRS Firmware 2020:** September 23rd 2020 (grid fix)
- **APRS Firmware 2022:** UV98_868_220430_S4 (GPS fix)

## Blog Posts & Articles

### HVDN Notebook (Hudson Valley Digital Network)
| Article | Date | URL |
|---------|------|-----|
| [Review: Lanch HG-UV98 APRS](https://hvdnnotebook.blogspot.com/2019/10/review-lanch-hg-uv98-aprs-2m70cm-ht.html) | Oct 2019 | Initial review by Steve Bossert (K2GOG) |
| [Sure to OK: Fixing the HG-UV98](https://hvdnnotebook.blogspot.com/2021/02/sure-to-ok-fixing-hg-uv98.html) | Feb 2021 | KT2KT firmware update guide |
| [Firmware Update: Lanch HG-UV98 APRS](https://hvdnnotebook.blogspot.com/2020/09/firmware-update-lanch-hg-uv98-aprs-radio.html) | Sep 2020 | Sep 2020 APRS firmware release |

### Reddit
- [First Look at HG-UV98](https://www.reddit.com/r/amateurradio/comments/c0k0bh/first_look_at_hguv98_first_chinese_aprs_handheld/) by ornotermes (SA5KPR)
- [Tear Down of HG-UV98](https://www.reddit.com/r/amateurradio/comments/c3wurz/teardown_of_hguv98_the_first_chinese_aprs_handheld/) by ornotermes (SA5KPR)

### Venus I-Tech (Official Reseller)
- Product page: https://www.venus-itech.com/product/hg-uv98-handheld-with-aprs/
- Comments section with community discussion

## Key Community Contributors

| Callsign/Name | Contribution |
|---------------|-------------|
| **Kenny KT2KT** | Modified firmware (Sure→OK), UI improvements |
| **Steve Bossert (K2GOG)** | HVDN Notebook reviews, documentation |
| **ornotermes (SA5KPR)** | Reddit teardown/review, marrold/HG-UV98 management |
| **BH4TDV** | Taobao seller, firmware development (WeChat: 13013684000) |
| **Wuxi Venus** | Manufacturer (Wuxi Venus Information Technology) |

## Known Issues & Workarounds (Community-Reported)

### Firmware Bricking
- Multiple reports of "Host UpData..." lock after failed flash
- **Symptom:** Radio shows "Host UpData..." and won't boot
- **Workaround:** None reliably confirmed; may require hardware JTAG
- **Prevention:** Use stock firmware first, then KT2KT update; ensure stable connection

### ARISS/ISS Path Issue
- Radio requires hop number in path (e.g., ARISS-0)
- Cannot use zero-hop paths (ARISS alone)
- Affects ISS packet operations

### Bluetooth Module Quality
- Later production batches may have incomplete soldering
- Symptoms: KISS TNC not working, Pinpoint/APRSdroid not connecting
- Affects WinLink compatibility
- May vary by manufacturing batch

### Coordinate Calculation Bug
- Fixed in September 2020 APRS firmware
- Grid square calculation was incorrect
- Distance calculations based on GPS were OK

### Smart Beacon Behavior
- Minimum interval: 10 seconds
- Standby: 3 minutes between beacons
- Adaptive based on direction change (>25°)

## Software Tools

### Windows
| Tool | Purpose | Source |
|------|---------|--------|
| CPS v2.0 | Channel programming | HG-UV98 EN V2.0 setup.rar |
| APRS Config | APRS settings | APRS_51Serial_20181009.rar |
| TeraTerm | Firmware update | https://osdn.net/projects/ttssh2/ |
| HyperTerminal | Firmware update | Hilgraeve trial |
| CH340 Drivers | USB serial | Included in package |

### Android
| App | Purpose |
|-----|---------|
| APRSdroid | APRS tracking, TNC via Bluetooth |
| Pinpoint APRS | APRS tracking, KISS TNC |

### Linux
- Community discussion ongoing (topic #110102221)
- APRSdroid for Android
- Linux-based APRS tools may work with USB cable

## Resources Index

### Manuals & Documentation
- [English User Manual](https://ba4tb.qth.com/download/HG-UV98%20Manual.pdf)
- [HVDN Review (2019)](https://hvdnnotebook.blogspot.com/2019/10/review-lanch-hg-uv98-aprs-2m70cm-ht.html)
- [KT2KT Update Guide (2021)](https://hvdnnotebook.blogspot.com/2021/02/sure-to-ok-fixing-hg-uv98.html)

### Downloads (HVDN Archive)
- APRS Firmware Sep 2020: `HG-UV98 September 23rd 2020 APRS firmware.zip`
- KT2KT Firmware: `V1.0.1_190619-KT.zip`
- CPS Software: `HG UV-98 Radio Programming Software (v1)`
- APRS Software: `HG UV-98 APRS Configuration Software (v1)`
- USB Drivers: `CH34x_Install_Windows_v3_4.zip`