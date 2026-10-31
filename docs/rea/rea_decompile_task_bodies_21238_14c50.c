INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802d178 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  int iVar1;
  undefined4 in_r3;
  
  recovery_fn();
  iVar1 = recovery_fn();
  if (iVar1 != -1) {
    func_0x0801b738();
    do {
      do {
        do {
          recovery_fn(0x1e);
          iVar1 = _DAT_0802d1d8;
          *(short *)(_DAT_0802d1d8 + 0x166) = *(short *)(_DAT_0802d1d8 + 0x166) + 1;
        } while (*(char *)(iVar1 + 0x119) != '\0');
        recovery_fn();
        recovery_fn();
        recovery_fn();
      } while (*(short *)(iVar1 + 0x166) != 0x11);
      *(undefined2 *)(iVar1 + 0x166) = 0;
      recovery_fn();
    } while( true );
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801b664 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  undefined4 in_r3;
  
  do {
    do {
      func_0x08023fb0(0,0,0,1,in_r3);
      iVar1 = _DAT_0801b9a0;
    } while (*(char *)(_DAT_0801b9a0 + 0x119) != '\0');
    *(int *)(_DAT_0801b9a0 + 0x19) = *(int *)(_DAT_0801b9a0 + 0x19) + 1;
    *(short *)(iVar1 + 0x1d) = *(short *)(iVar1 + 0x1d) + 1;
    if (*(int *)(iVar1 + 0x19) == 8) {
      *(undefined4 *)(iVar1 + 0x19) = 0;
      func_0x080226f4();
      func_0x08026200();
      func_0x08024c24();
      func_0x08023542();
      if ((*_DAT_0801b9a4 & 0x7f) >> 6 == 0) {
        func_0x080257a8();
        func_0x080258f8();
        func_0x08025acc();
      }
      FUN_08021a94();
      func_0x08022320();
      func_0x08025a4c();
    }
    if (*(short *)(iVar1 + 0x1d) == 500) {
      *(undefined2 *)(iVar1 + 0x1d) = 0;
      func_0x0801e09c();
      func_0x0801deb6();
      func_0x0801e1a0();
      FUN_08025d08();
      func_0x080225dc();
      func_0x080224f4();
      func_0x0801df70();
    }
  } while( true );
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801b704 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  do {
    do {
      func_0x08023fb0(0,0,0,500);
    } while (*(char *)(_DAT_0801b9a0 + 0x119) != '\0');
    func_0x080234cc();
  } while( true );
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08014dea (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  int iVar2;
  byte *pbVar3;
  undefined4 in_r3;
  
  do {
    do {
      func_0x08023fb0(0,0,0,10,in_r3);
      iVar1 = DAT_08014f38;
    } while (*(char *)(DAT_08014f38 + 0x119) != '\0');
    *(short *)(DAT_08014f38 + 0x7c) = *(short *)(DAT_08014f38 + 0x7c) + 1;
    func_0x08024106();
    pbVar3 = _DAT_08015b00;
    iVar2 = DAT_08014f3c;
    if (((int)((uint)*_DAT_08015b00 << 0x19) < 0) || ((int)((uint)_DAT_08015b00[3] << 0x1f) < 0)) {
      if ((*_DAT_08015b00 & 0x7f) >> 6 == 0) {
        if (((_DAT_08015b00[3] & 1) != 0) && (0x14 < *(ushort *)(iVar1 + 0x7c))) {
          *(undefined2 *)(iVar1 + 0x7c) = 0;
        }
      }
      else {
        func_0x080203d0();
        func_0x080257a8();
        func_0x080258f8();
        func_0x08025acc();
      }
    }
    else {
      if (*(char *)(DAT_08014f3c + 0xc) == '\x01') {
        if ((((*(char *)(DAT_08014f3c + 1) == '\0') || ((_DAT_08015b00[2] & 7) >> 2 != 0)) ||
            (*DAT_08014f34 != '\0')) || ((_DAT_08015b00[0xb] & 0x3f) >> 5 != 0)) {
          _DAT_08015b00[3] = _DAT_08015b00[3] & 0xdf;
          FUN_08025496();
        }
        else {
          FUN_08025496();
          if ((pbVar3[3] & 0x3f) >> 5 != 0) {
            *(short *)(iVar1 + 0x86) = *(short *)(iVar1 + 0x86) + 1;
          }
          if (*_DAT_08015b30 == '\0' && *_DAT_08015b34 == '\0') {
            if (*(short *)(iVar1 + 0x86) == 0x14) {
              FUN_08025e74(0,1);
              FUN_08025e74(0,0);
            }
            else if ((ushort)((ushort)*(byte *)(iVar2 + 1) * 0x14 + 0x28) <=
                     *(ushort *)(iVar1 + 0x86)) {
              FUN_08022a98();
              *(undefined2 *)(iVar1 + 0x7c) = 0;
              *(undefined2 *)(iVar1 + 0x86) = 0;
              pbVar3[3] = pbVar3[3] & 0xdf;
            }
          }
          else {
            *(undefined2 *)(iVar1 + 0x86) = 0;
            pbVar3[3] = pbVar3[3] & 0xdf;
          }
        }
      }
      else {
        func_0x080252ac();
      }
      if (0x14 < *(ushort *)(iVar1 + 0x7c)) {
        *(undefined2 *)(iVar1 + 0x7c) = 0;
      }
    }
    func_0x080240e8();
  } while( true );
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08021238 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1)

{
  byte *pbVar1;
  int iVar2;
  int iVar3;
  byte bVar4;
  char cVar5;
  undefined4 uStack_3c;
  undefined4 uStack_38;
  undefined4 uStack_34;
  
  iVar3 = _DAT_08021ba8;
  iVar2 = _DAT_08021b98;
  pbVar1 = _DAT_08021b94;
  if (param_1 == '\0') {
    if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
      *(char *)(_DAT_08021b98 + 0x19) = *(char *)(_DAT_08021b98 + 0x19) + '\x01';
      if (9 < *(byte *)(iVar2 + 0x19)) {
        *(undefined1 *)(iVar2 + 0x19) = 0;
      }
      FUN_08024cf8((ushort)*(byte *)(iVar2 + 0x19) * 0x10 + 0x5f);
      FUN_0801a6ea();
    }
    else {
      FUN_08019910(0xd);
    }
  }
  else if (param_1 == '\x01') {
    FUN_08023000();
    pbVar1 = _DAT_08021b94;
    if (*_DAT_08021bac == '\0') {
      if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
        FUN_080260a4(*DAT_0802165c,0xf);
        FUN_0801baa4(100);
        *pbVar1 = *pbVar1 | 0x40;
        *(undefined1 *)(_DAT_08021b9c + 0x76) = 0xb;
        bVar4 = FUN_0801f28e(*(undefined1 *)
                              (_DAT_08021ba8 + (uint)*(byte *)(_DAT_08021b98 + 0x40) * 0x1e + 3));
        pbVar1[4] = pbVar1[4] & 0xf7 | (bVar4 & 1) << 3;
      }
      else {
        FUN_08019910(0xd);
      }
    }
  }
  else if (param_1 == '\x02') {
    if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
      cVar5 = FUN_0801f28e(*(undefined1 *)
                            ((uint)*(byte *)(_DAT_08021b98 + 0x40) * 0x1e + _DAT_08021ba8 + 3));
      if ((pbVar1[6] & 0x3f) >> 5 == 0) {
        *(undefined1 *)(_DAT_08021b9c + (uint)*(byte *)(iVar2 + 0x40) + 0x10d) = 0;
        pbVar1[6] = pbVar1[6] | 0x20;
        if (cVar5 == '\x01') {
          pbVar1[1] = pbVar1[1] & 0x7f | *_DAT_08021bb0 << 7;
        }
        else {
          pbVar1[2] = pbVar1[2] & 0xfe | *_DAT_08021bb4 & 1;
        }
        pbVar1[1] = pbVar1[1] & 0xdf | (*(byte *)(iVar2 + 0x40) & 1) << 5;
        pbVar1[1] = pbVar1[1] & 0xef | (pbVar1[1] >> 5 & 1) << 4;
        FUN_0801d8c0(&uStack_3c,iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e,0x1e);
        FUN_0801ab36(*(undefined1 *)(iVar2 + 0x40),uStack_3c,uStack_38,uStack_34);
        FUN_08025d08();
        FUN_0801db54(*(undefined1 *)(iVar2 + 0x40));
      }
      else {
        pbVar1[6] = pbVar1[6] & 0xdf;
        if (cVar5 == '\x01') {
          *_DAT_08021bb0 = '\0';
        }
        else {
          *_DAT_08021bb4 = 0;
        }
        FUN_0801e240(*(undefined1 *)(iVar2 + 0x40));
        *(undefined1 *)(_DAT_08021b9c + (uint)*(byte *)(iVar2 + 0x40) + 0x10d) =
             *(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar2 + 5);
        FUN_08022044(*(undefined1 *)(iVar2 + 0x40),cVar5,0);
      }
    }
    else {
      FUN_08019910(0xd);
    }
  }
  else if (param_1 == '\x03') {
    if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
      if ((_DAT_08021b94[6] & 0x1f) >> 4 == 0) {
        _DAT_08021b94[6] = _DAT_08021b94[6] | 0x10;
        FUN_08023780(DAT_0802203c,0x4000);
      }
      else {
        _DAT_08021b94[6] = _DAT_08021b94[6] & 0xef;
        FUN_08023788(DAT_0802203c,0x4000);
      }
    }
    else {
      FUN_08019910(0xd);
    }
  }
  else if (param_1 == '\x04') {
    if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
      *(undefined1 *)(_DAT_08021b9c + 1) = 0;
      iVar2 = _DAT_08021b98;
      if (*(char *)(_DAT_08021b98 + 0x40) == '\0') {
        *(undefined1 *)(_DAT_08021b98 + 0x40) = 1;
      }
      else {
        *(undefined1 *)(_DAT_08021b98 + 0x40) = 0;
      }
      FUN_08022a98();
      FUN_0801a6ea();
      if (*(char *)(iVar2 + 0xc) == '\x01') {
        FUN_08022930(*(undefined1 *)(iVar2 + 0x40));
      }
      else {
        FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
      }
    }
    else {
      FUN_08019910(0xd);
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08014c50 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 < 10) {
    if (*(char *)((uint)*DAT_08014f34 * 0x10 + _DAT_08014f5c + 0xc) == '\0') {
      func_0x08014950(param_1);
    }
    else if (*(char *)((uint)*DAT_08014f34 * 0x10 + _DAT_08014f5c + 0xc) == '\x01') {
      func_0x080147cc(param_1);
    }
    else if (*(char *)((uint)*DAT_08014f34 * 0x10 + _DAT_08014f5c + 0xc) == '\x02') {
      func_0x08014a7a(param_1);
    }
  }
  return;
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
      FUN_080260a4(*DAT_08019d98,0x10);
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
    FUN_080260a4(*DAT_08019d98,0x39);
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
