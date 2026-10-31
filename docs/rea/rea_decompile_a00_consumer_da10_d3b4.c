openjdk version "21.0.12.1" 2026-08-18
OpenJDK Runtime Environment (build 21.0.12.1+1-1-deb13u1-Debian)
OpenJDK 64-Bit Server VM (build 21.0.12.1+1-1-deb13u1-Debian, mixed mode)
INFO  Using log config file: jar:file:/opt/ghidra_12.1.4_PUBLIC/Ghidra/Framework/Generic/lib/Generic.jar!/generic.log4j.xml (LoggingInitialization)  
INFO  Using log file: /home/sslayer/.config/ghidra/ghidra_12.1.4_PUBLIC/application.log (LoggingInitialization)  
INFO  Loading user preferences: /home/sslayer/.config/ghidra/ghidra_12.1.4_PUBLIC/preferences (Preferences)  
INFO  Searching for classes... (ClassSearcher)  
INFO  Class search complete (478 ms) (ClassSearcher)  
INFO  Initializing SSL Context (DefaultSSLContextInitializer)  
INFO  Initializing Random Number Generator... (SecureRandomFactory)  
INFO  Random Number Generator initialization complete: NativePRNGNonBlocking (SecureRandomFactory)  
INFO  Trust manager disabled, cacerts have not been set (DefaultTrustManagerFactory)  
INFO  Headless startup complete (1240 ms) (AnalyzeHeadless)  
INFO  Class searcher loaded 59 extension points (20 false positives) (ClassSearcher)  
INFO  HEADLESS Script Paths:
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/Base/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/FileFormats/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Processors/8051/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/DecompilerDependent/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/SwiftDemangler/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Debug/Debugger/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Processors/PIC/ghidra_scripts
    /tmp/opencode
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/FunctionID/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/Decompiler/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/MicrosoftCodeAnalyzer/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/GnuDemangler/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/SystemEmulation/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/BytePatterns/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Processors/Atmel/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Processors/DATA/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/WildcardAssembler/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/PDB/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Processors/JVM/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/VersionTracking/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Debug/Debugger-rmi-trace/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/PyGhidra/ghidra_scripts
    /opt/ghidra_12.1.4_PUBLIC/Ghidra/Features/BSim/ghidra_scripts (HeadlessAnalyzer)  
INFO  HEADLESS: execution starts (HeadlessAnalyzer)  
INFO  Opening existing project: /tmp/opencode/ghproj/hguv98 (HeadlessAnalyzer)  
INFO  Opening project: /tmp/opencode/ghproj/hguv98 (HeadlessProject)  
INFO  REPORT: Processing project file: /MAIN-MCU-HG-UV98-V1.0.1-190416.bin (HeadlessAnalyzer)  
INFO  REPORT: Execute script: ReaCreateDecompMulti.java '0x0801d3b4' '0x0801da10'  (HeadlessAnalyzer)  
INFO  SCRIPT: /tmp/opencode/ReaCreateDecompMulti.java (HeadlessAnalyzer)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801d3b4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

uint recovery_fn(byte param_1,byte param_2,byte param_3,uint param_4,char param_5)

{
  undefined4 uVar1;
  byte bVar2;
  byte bVar3;
  
  FUN_0801d840(1);
  uVar1 = _DAT_0801d834;
  FUN_08023788(_DAT_0801d834,2);
  FUN_0801b9c8(param_1,param_2,((param_4 & 0xff) + (uint)param_1) - 1 & 0xffff,
               (ushort)param_3 + (ushort)param_2 + -1);
  FUN_0801ba6a(0x2c);
  for (bVar3 = 0; bVar3 < param_3; bVar3 = bVar3 + 1) {
    for (bVar2 = 0; (uint)bVar2 < (param_4 & 0xff); bVar2 = bVar2 + 1) {
      if (param_5 == '\0') {
        FUN_0801ba8c(0);
        FUN_0801ba8c(0);
      }
      else if (param_5 == '\x04') {
        if (*(char *)(_DAT_0801d830 + 0x18) == '\0') {
          FUN_0801ba8c(0);
          FUN_0801ba8c(0);
        }
        else {
          FUN_0801ba8c(0xef);
          FUN_0801ba8c(0xbe);
        }
      }
      else if (param_5 == '\x01') {
        FUN_0801ba8c(0);
        FUN_0801ba8c(0x1f);
      }
      else if (param_5 == '\x03') {
        FUN_0801ba8c(0xe0);
        FUN_0801ba8c(0x20);
      }
      else if (param_5 == '\a') {
        FUN_0801ba8c(0xf0);
        FUN_0801ba8c(0x1e);
      }
      else if (param_5 == '\x06') {
        FUN_0801ba8c(0x20);
        FUN_0801ba8c(0xc2);
      }
      else if (param_5 == '\x05') {
        FUN_0801ba8c(0x13);
        FUN_0801ba8c(0xa2);
      }
    }
  }
  FUN_08023780(uVar1,2);
  FUN_0801d840(0);
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801da10 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  undefined4 uVar1;
  
  recovery_fn(0,0,0xa0,1,4);
  recovery_fn(0,0,1,0x7f,4);
  recovery_fn(0,0x9f,1,0x80,4);
  recovery_fn(0x7f,0,0x9f,1,4);
  recovery_fn(0x11,0,0xa0,1,4);
  if ((*_DAT_0801e2c4 == '\0') && (*(char *)(_DAT_0801e2c8 + 0xc) == '\x01')) {
    recovery_fn(0x3f,0,0x9f,1,4);
  }
  recovery_fn(0x6e,0,0xa0,1,4);
  recovery_fn();
  recovery_fn();
  recovery_fn(*(undefined1 *)(_DAT_0801e2cc + 0x36));
  FUN_0801fc50();
  FUN_0801cd32(0x6f,0x1e,0x78,0x10);
  FUN_0801cd32(0x6f,1,5,0x10);
  FUN_0801cd32(0x6f,0x9d,2,0x10);
  if (*DAT_0801e5b4 == '\x01') {
    FUN_0801cb60(0x6f,6,DAT_0801e5b8,4,3);
    uVar1 = 3;
    FUN_0801cb60(0x6f,0x7d,DAT_0801e5bc,4);
  }
  else {
    FUN_0801cb60(0x6f,6,DAT_0801e5c4,4,3);
    uVar1 = 3;
    FUN_0801cb60(0x6f,0x7d,DAT_0801e5c8,4);
  }
  return uVar1;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  REPORT: Save succeeded for processed file: /MAIN-MCU-HG-UV98-V1.0.1-190416.bin (HeadlessAnalyzer)  
