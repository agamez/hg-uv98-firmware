INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080260a4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
undefined4 recovery_fn(char param_1,byte param_2,undefined4 param_3,undefined4 param_4)

{
  byte *pbVar1;
  byte *pbVar2;
  
  pbVar2 = DAT_08026698;
  pbVar1 = DAT_08026690;
  if (((*DAT_08026690 < 5) && (*(char *)(DAT_08026694 + 0x17) != '\0')) &&
     (((*DAT_08026698 & 0x7f) >> 6 == 0 || (9 < param_2)))) {
    *(byte *)(DAT_0802669c + (uint)*DAT_08026690) = param_2;
    *pbVar1 = *pbVar1 + 1;
    if (*DAT_080266a0 == '\0') {
      *DAT_080266a0 = '\x01';
      FUN_08025ff8();
      if ((*DAT_080266a4 != '\0') && ((pbVar2[5] & 3) >> 1 != 0)) {
        FUN_08025780(0,1,0);
      }
      if ((*DAT_080266a8 != '\0') && ((pbVar2[5] & 3) >> 1 != 0)) {
        FUN_08025780(1,0,0);
      }
      FUN_08023788(DAT_0802668c,0x4000);
      FUN_08025fc0(10);
      FUN_08023780(DAT_08026688,2);
      FUN_08025fc0(1);
      if (param_1 == '\x01') {
        *DAT_080266ac = param_2 + 0x3b;
      }
      else {
        if (param_1 != '\0') {
          return param_4;
        }
        *DAT_080266ac = param_2;
      }
      pbVar1 = DAT_080266b0;
      *DAT_080266b0 = *DAT_080266ac;
      *DAT_080266b4 = 1;
      FUN_0802601e(*pbVar1);
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019910 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  byte *pbVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  undefined4 uStack_34;
  undefined4 uStack_30;
  undefined4 uStack_2c;
  undefined4 uStack_28;
  undefined4 uStack_24;
  undefined4 uStack_14;
  
  pbVar1 = DAT_0801a37c;
  uStack_14 = param_4;
  if ((*DAT_0801a37c & 0x7f) >> 6 != 0) {
    DAT_0801a37c[1] = DAT_0801a37c[1] & 0xfe;
    pbVar1[5] = pbVar1[5] & 0xfe;
    iVar2 = DAT_0801a380;
    *(undefined2 *)(DAT_0801a380 + 0x7e) = 0;
    if ((param_1 != 0xb) && (param_1 != 0xc)) {
      *pbVar1 = *pbVar1 & 0xbf;
      pbVar1[2] = pbVar1[2] & 0xfe;
      pbVar1[1] = pbVar1[1] & 0x7f;
      if (*(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 8) == '\x01') {
        FUN_08019d4c(0xff);
        FUN_0801a6ea();
      }
      else {
        FUN_0801a7aa(*(undefined1 *)(DAT_0801a388 + 0x40),
                     DAT_0801a38c + (uint)*(byte *)(DAT_0801a388 + 0x40) * 0x1e);
      }
      recovery_fn(*DAT_08019d98,0x10);
      return;
    }
    *(byte *)(iVar2 + 0x76) = param_1;
    FUN_08023788(DAT_0801a384,2);
  }
  iVar2 = DAT_0801a388;
  if (param_1 < 10) {
    FUN_08023000();
    FUN_0801ea04(param_1);
  }
  else if (param_1 == 10) {
    FUN_08023000();
    iVar2 = DAT_0801a380;
    *(undefined1 *)(DAT_0801a380 + 0x129) = 0;
    *(undefined1 *)(iVar2 + 1) = 0;
    iVar4 = DAT_0801a388;
    *(undefined1 *)(iVar2 + 0xf0) = *(undefined1 *)(DAT_0801a388 + 0x40);
    iVar3 = DAT_0801a394;
    *(undefined1 *)(DAT_0801a394 + 2) = *(undefined1 *)(iVar2 + 0x132);
    if (0x35 < *(byte *)(iVar2 + 0x132)) {
      *(undefined1 *)(iVar2 + 0x132) = 0;
    }
    FUN_0801b15c(iVar3,4,10);
    FUN_08014530(0x35);
    FUN_0801cdfc();
    FUN_0801d8c0(&uStack_34,DAT_0801a38c + (uint)*(byte *)(iVar4 + 0x40) * 0x1e,0x1e);
    FUN_0801ab36(*(undefined1 *)(iVar4 + 0x40),uStack_34,uStack_30,uStack_2c);
    recovery_fn(*DAT_08019d98,0x39);
  }
  else if (param_1 == 0xb) {
    FUN_08023000();
    iVar2 = DAT_0801a380;
    *(undefined1 *)(DAT_0801a380 + 1) = 0;
    *(undefined1 *)(iVar2 + 0x76) = 0xb;
    if (((int)((uint)pbVar1[1] << 0x18) < 0) || ((int)((uint)pbVar1[2] << 0x1f) < 0)) {
      FUN_0801e240(*(undefined1 *)(DAT_0801a388 + 0x40));
    }
    iVar2 = DAT_0801a388;
    if (*(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 8) == '\x01') {
      *(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 0x41) =
           *(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 0x41) + '\x01';
      iVar4 = FUN_080200b0((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41,0xb,0x80,0);
      if (iVar4 != 0) {
        FUN_0801a7d4(*(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41),
                     DAT_0801a38c + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
        FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
        if ((*pbVar1 & 0xf) >> 3 == 0) {
          FUN_08022a98();
          FUN_0801a6ea();
          FUN_08023408(*(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41));
        }
        FUN_08012f0c();
      }
    }
    else {
      FUN_080201aa(0xb);
    }
  }
  else if (param_1 == 0xc) {
    FUN_08023000();
    iVar2 = DAT_0801a380;
    *(undefined1 *)(DAT_0801a380 + 1) = 0;
    *(undefined1 *)(iVar2 + 0x76) = 0xc;
    if (((int)((uint)pbVar1[1] << 0x18) < 0) || ((int)((uint)pbVar1[2] << 0x1f) < 0)) {
      FUN_0801e240(*(undefined1 *)(DAT_0801a388 + 0x40));
    }
    iVar2 = DAT_0801a388;
    if (*(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 8) == '\x01') {
      *(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 0x41) =
           *(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 0x41) + -1;
      iVar4 = FUN_080200b0((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41,0xc,0x80,0);
      if (iVar4 != 0) {
        FUN_0801a7d4(*(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41),
                     DAT_0801a38c + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
        FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
        if ((*pbVar1 & 0xf) >> 3 == 0) {
          FUN_08022a98();
          FUN_0801a6ea();
          FUN_08023408(*(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 0x41));
        }
        FUN_08012f0c();
      }
    }
    else {
      FUN_080201aa(0xc);
    }
  }
  else if (param_1 == 0xd) {
    FUN_08023000();
    iVar2 = DAT_0801a380;
    if (*(char *)(DAT_0801a380 + 1) != '\0') {
      *(char *)(DAT_0801a380 + 1) = *(char *)(DAT_0801a380 + 1) + -1;
      if (*(char *)(iVar2 + 1) == '\x03') {
        *(char *)(iVar2 + 1) = *(char *)(iVar2 + 1) + -1;
      }
      iVar4 = DAT_0801a388;
      if (*(char *)(iVar2 + 1) == '\0') {
        *(undefined1 *)(DAT_0801a390 + 2) = 1;
      }
      else if (*(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a388 + 8) == '\0') {
        *(undefined1 *)((uint)*(byte *)(iVar2 + 1) + iVar2 + 2) = 0x2d;
        if (*(char *)(iVar4 + 0xc) == '\x01') {
          if (*(char *)(iVar4 + 0x40) == '\0') {
            uStack_24 = 6;
            uStack_28 = 7;
            FUN_0801cc0c(0x1d,0x16,iVar2 + 2,4);
            uStack_24 = 3;
            uStack_28 = 2;
            FUN_0801cc0c(0x23,0x6c,iVar2 + 9,4);
          }
          else {
            uStack_24 = 6;
            uStack_28 = 7;
            FUN_0801cc0c(0x4c,0x16,iVar2 + 2,4);
            uStack_24 = 3;
            uStack_28 = 2;
            FUN_0801cc0c(0x52,0x6c,iVar2 + 9,4);
          }
        }
        else {
          uStack_24 = 9;
          uStack_28 = 4;
          FUN_0801cc0c(0x32,0xe,iVar2 + 2,4);
          uStack_24 = 9;
          uStack_28 = 3;
          FUN_0801cc0c(0x32,0x46,iVar2 + 6,4);
          uStack_24 = 4;
          uStack_28 = 2;
          FUN_0801cc0c(0x40,0x78,iVar2 + 9,4);
        }
      }
      else {
        *(undefined1 *)(iVar2 + 1) = 0;
        *(undefined1 *)(DAT_0801a390 + 2) = 1;
      }
    }
  }
  else if (param_1 == 0xe) {
    if (*(char *)((uint)*(byte *)(DAT_0801a388 + 0x40) + DAT_0801a380 + 0xba) == '\0') {
      *(undefined1 *)(DAT_0801a380 + (uint)*(byte *)(DAT_0801a388 + 0x40) + 0xba) = 1;
    }
    else {
      *(undefined1 *)(DAT_0801a380 + (uint)*(byte *)(DAT_0801a388 + 0x40) + 0xba) = 0;
    }
    FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
    FUN_08022a98();
  }
  else if (param_1 == 0xf) {
    FUN_080214e4(1);
  }
  else if (param_1 == 0x10) {
    FUN_08023000();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080264b4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1)

{
  if (param_1 == '\x02') {
    recovery_fn(*_DAT_080266bc,0x17);
  }
  else if (param_1 == '\x03') {
    recovery_fn(*_DAT_080266bc,0x1a);
  }
  else if (param_1 == '\x04') {
    recovery_fn(*_DAT_080266bc,0x2a);
  }
  else if (param_1 == '\x05') {
    recovery_fn(*_DAT_080266bc,0x16);
  }
  else if (param_1 == '\x06') {
    recovery_fn(*_DAT_080266bc,0x1f);
  }
  else if (param_1 == '\a') {
    recovery_fn(*_DAT_080266bc,0x2b);
  }
  else if (param_1 == '\b') {
    recovery_fn(*_DAT_080266bc,0x1b);
  }
  else if (param_1 == '\f') {
    recovery_fn(*_DAT_080266bc,0x15);
  }
  else if (param_1 == '\r') {
    recovery_fn(*_DAT_080266bc,0x14);
  }
  else if (param_1 == '\x0e') {
    recovery_fn(*_DAT_080266bc,0x15);
  }
  else if (param_1 == '\x0f') {
    recovery_fn(*_DAT_080266bc,0x14);
  }
  else if (param_1 == '\x11') {
    recovery_fn(*_DAT_080266bc,0x20);
  }
  else if (param_1 == '\x12') {
    recovery_fn(*_DAT_080266bc,0xc);
  }
  else if (param_1 == '\x17') {
    recovery_fn(*_DAT_080266bc,0x2d);
  }
  else if (param_1 == '\x18') {
    recovery_fn(*_DAT_080266bc,0x1c);
  }
  else if (param_1 == '\x1b') {
    recovery_fn(*_DAT_080266bc,0x33);
  }
  else if (param_1 == '\x1f') {
    recovery_fn(*_DAT_080266bc,0x2f);
  }
  else if (param_1 == ' ') {
    recovery_fn(*_DAT_080266bc,0x2f);
  }
  else if (param_1 == '!') {
    recovery_fn(*_DAT_080266bc,0x34);
  }
  else if (param_1 == '\"') {
    recovery_fn(*_DAT_080266bc,0x34);
  }
  else if (param_1 == '#') {
    recovery_fn(*_DAT_080266bc,0x35);
  }
  else if (param_1 == '$') {
    recovery_fn(*_DAT_080266bc,0x35);
  }
  else if (param_1 == '%') {
    recovery_fn(*_DAT_080266bc,0x18);
  }
  else if (param_1 == '&') {
    recovery_fn(*_DAT_080266bc,0x19);
  }
  else if (param_1 == '\'') {
    recovery_fn(*_DAT_080266bc,0x28);
  }
  else if (((param_1 != ')') && (param_1 != '*')) && (param_1 == '2')) {
    recovery_fn(*_DAT_080266bc,0x30);
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08023678 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(uint *param_1,ushort *param_2)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  uint uVar4;
  
  uVar4 = *(byte *)((int)param_2 + 3) & 0xf;
  if ((int)((uint)*(byte *)((int)param_2 + 3) << 0x1b) < 0) {
    uVar4 = (byte)param_2[1] | uVar4;
  }
  if ((char)*param_2 != '\0') {
    uVar1 = *param_1;
    for (uVar3 = 0; uVar3 < 8; uVar3 = uVar3 + 1) {
      uVar2 = 1 << (uVar3 & 0xff);
      if ((*param_2 & uVar2) == uVar2) {
        uVar1 = uVar1 & ~(0xf << (uVar3 << 2 & 0xff)) | uVar4 << (uVar3 << 2 & 0xff);
        if (*(char *)((int)param_2 + 3) == '(') {
          param_1[5] = 1 << (uVar3 & 0xff);
        }
        if (*(char *)((int)param_2 + 3) == 'H') {
          param_1[4] = 1 << (uVar3 & 0xff);
        }
      }
    }
    *param_1 = uVar1;
  }
  if (0xff < *param_2) {
    uVar1 = param_1[1];
    for (uVar3 = 0; uVar3 < 8; uVar3 = uVar3 + 1) {
      uVar2 = 1 << (uVar3 + 8 & 0xff);
      if ((*param_2 & uVar2) == uVar2) {
        uVar1 = uVar1 & ~(0xf << (uVar3 << 2 & 0xff)) | uVar4 << (uVar3 << 2 & 0xff);
        if (*(char *)((int)param_2 + 3) == '(') {
          param_1[5] = 1 << (uVar3 + 8 & 0xff);
        }
        if (*(char *)((int)param_2 + 3) == 'H') {
          param_1[4] = 1 << (uVar3 + 8 & 0xff);
        }
      }
    }
    param_1[1] = uVar1;
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08020da6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  char *pcVar2;
  char *pcVar3;
  
  iVar1 = _DAT_08020fa0;
  if ((*DAT_08021668 == '\0') && ((*(byte *)(_DAT_08020fa0 + 9) & 7) >> 2 == 0)) {
    *(byte *)(_DAT_08020fa0 + 9) = *(byte *)(_DAT_08020fa0 + 9) | 4;
    pcVar3 = DAT_08021088;
    if ((*DAT_08021088 != '\0') && ((*(byte *)(iVar1 + 5) & 3) >> 1 != 0)) {
      FUN_08025780(0,1,0);
    }
    pcVar2 = DAT_08021080;
    if ((*DAT_08021080 != '\0') && ((*(byte *)(iVar1 + 5) & 3) >> 1 != 0)) {
      FUN_08025780(1,0,0);
    }
    FUN_08029468(600,0x4b0);
    FUN_0801baa4(0x32);
    FUN_08029454();
    FUN_0801baa4(200);
    FUN_08029468(600,0x4b0);
    FUN_0801baa4(0x32);
    FUN_08029454();
    if ((*pcVar3 != '\0') && ((*(byte *)(iVar1 + 5) & 3) >> 1 != 0)) {
      *(byte *)(iVar1 + 5) = *(byte *)(iVar1 + 5) & 0xfd;
      *DAT_0802108c = 0;
    }
    if ((*pcVar2 != '\0') && ((*(byte *)(iVar1 + 5) & 7) >> 2 != 0)) {
      *(byte *)(iVar1 + 5) = *(byte *)(iVar1 + 5) & 0xfb;
      *DAT_08021084 = 0;
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
