INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080234cc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  uint uVar1;
  undefined4 in_r3;
  
  if ((*_DAT_0802358c != '\0') || (*_DAT_08023590 != '\0')) {
    if ((*_DAT_0802358c != '\0') && (uVar1 = FUN_0802692e(1,0x1f), (uVar1 & 0xfff) == 0xfff)) {
      FUN_080272e2(1,1);
      FUN_0801baa4(0x1e);
    }
    if ((*_DAT_08023590 != '\0') && (uVar1 = FUN_0802692e(0,0x1f), (uVar1 & 0xfff) == 0xfff)) {
      FUN_080272e2(0,1);
      FUN_0801baa4(0x1e);
    }
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080226f4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  ushort uVar1;
  int extraout_r2;
  undefined4 unaff_r7;
  char in_CY;
  
  FUN_08029ab8(*(undefined4 *)(DAT_0802307c + 0x137),_DAT_08023080);
  if ((in_CY == '\0') &&
     (uVar1 = *(ushort *)(extraout_r2 + 0x13c), *(ushort *)(extraout_r2 + 0x13c) = uVar1 + 1,
     0x753 < uVar1)) {
    *(undefined2 *)(extraout_r2 + 0x13c) = 0;
    recovery_fn(*DAT_08022a88,0x24);
    FUN_0801baa4(2000);
  }
  return unaff_r7;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08026200 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  undefined4 uVar1;
  byte *pbVar2;
  char *pcVar3;
  char *pcVar4;
  byte *pbVar5;
  int iVar6;
  undefined4 in_r3;
  
  pcVar4 = _DAT_080266a0;
  uVar1 = _DAT_08026688;
  if ((*_DAT_080266a0 != '\0') &&
     (iVar6 = FUN_0802375e(_DAT_08026688,0x80), pbVar2 = _DAT_080266b8, iVar6 == 1)) {
    *_DAT_080266b8 = *_DAT_080266b8 + 1;
    pbVar5 = _DAT_080266b4;
    if (*_DAT_080266b4 == *pbVar2) {
      *pbVar2 = 0;
      pcVar3 = _DAT_0802669c;
      pbVar2 = _DAT_08026690;
      if (*_DAT_08026690 < 2) {
        if (*_DAT_08026690 == 1) {
          *_DAT_08026690 = *_DAT_08026690 - 1;
        }
        FUN_08023788(uVar1,2);
        *pcVar4 = '\0';
        FUN_08029504();
        if ((*_DAT_080266a4 != '\0') && ((*(byte *)(_DAT_08026698 + 5) & 3) >> 1 != 0)) {
          *(byte *)(_DAT_08026698 + 5) = *(byte *)(_DAT_08026698 + 5) & 0xfd;
          *_DAT_080266c0 = 0;
        }
        if ((*_DAT_080266a8 != '\0') && ((*(byte *)(_DAT_08026698 + 5) & 7) >> 2 != 0)) {
          *(byte *)(_DAT_08026698 + 5) = *(byte *)(_DAT_08026698 + 5) & 0xfb;
          *_DAT_080266c4 = 0;
        }
      }
      else {
        FUN_0801d8c0(_DAT_0802669c,_DAT_0802669c + 1,*_DAT_08026690 - 1);
        *pbVar2 = *pbVar2 - 1;
        if (*_DAT_080266bc == '\x01') {
          *_DAT_080266b0 = *pcVar3 + ';';
        }
        else {
          *_DAT_080266b0 = *pcVar3;
        }
        *pbVar5 = 1;
        recovery_fn(*_DAT_080266b0);
      }
    }
    else {
      *_DAT_080266b0 = *(char *)(_DAT_080266ac + (uint)*pbVar2);
      func_0x080261a6();
    }
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08024c24 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  byte bVar1;
  int iVar2;
  char *pcVar3;
  int iVar4;
  ushort uVar5;
  
  iVar4 = DAT_08024e28;
  iVar2 = _DAT_08024d64;
  if (((*(char *)(_DAT_08024d64 + 4) != '\0') && (*DAT_08024e2c == '\0' && *DAT_08024e30 == '\0'))
     && (*_DAT_08024e48 == '\0')) {
    if (*(short *)(DAT_08024e28 + 0x10f) == 0) {
      if (*(char *)(DAT_08024e28 + 0x111) == '\0') {
        if (*DAT_08024e38 == '\0') {
          uVar5 = func_0x08029632(6);
          pcVar3 = _DAT_08024d68;
          if (uVar5 < *(ushort *)(_DAT_08024e4c + (uint)*(byte *)(iVar2 + 4) * 2 + -2)) {
            if ((*_DAT_08024d68 != '\0') &&
               (uVar5 = *(ushort *)(iVar4 + 0x88), *(ushort *)(iVar4 + 0x88) = uVar5 + 1,
               (ushort)((*(byte *)(iVar2 + 0x1c) + 1) * 0x7d) < uVar5)) {
              *(undefined2 *)(iVar4 + 0x88) = 0;
              *pcVar3 = '\0';
              *_DAT_08024e50 = 0;
              FUN_08024a1a();
            }
          }
          else {
            *(undefined2 *)(iVar4 + 0x88) = 0;
            bVar1 = *_DAT_08024e50;
            *_DAT_08024e50 = bVar1 + 1;
            if ((2 < bVar1) && (*_DAT_08024d68 == '\0')) {
              *_DAT_08024d68 = '\x01';
              FUN_080247cc();
            }
          }
        }
      }
      else {
        *(char *)(DAT_08024e28 + 0x111) = *(char *)(DAT_08024e28 + 0x111) + -1;
      }
    }
    else {
      *(short *)(DAT_08024e28 + 0x10f) = *(short *)(DAT_08024e28 + 0x10f) + -1;
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08023542 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  undefined4 unaff_r7;
  
  if (*(char *)(_DAT_08023594 + 0x13b) != '\0') {
    *(undefined1 *)(_DAT_08023594 + 0x13b) = 0;
    recovery_fn();
  }
  return unaff_r7;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
