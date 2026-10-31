INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802692e (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
ulonglong recovery_fn(undefined1 param_1,byte param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  char cVar2;
  short sVar3;
  byte bVar4;
  uint uVar5;
  bool bVar6;
  
  uVar1 = DAT_08027470;
  uVar5 = 0;
  FUN_08023788(DAT_08027470,0x8000);
  FUN_08026838(param_1);
  sVar3 = 10;
  do {
    bVar6 = sVar3 != 0;
    sVar3 = sVar3 + -1;
  } while (bVar6);
  FUN_08026712(param_2 | 0x80);
  FUN_080266f6();
  sVar3 = 4;
  do {
    bVar6 = sVar3 != 0;
    sVar3 = sVar3 + -1;
  } while (bVar6);
  FUN_08023788(uVar1,0x8000);
  sVar3 = 4;
  do {
    bVar6 = sVar3 != 0;
    sVar3 = sVar3 + -1;
  } while (bVar6);
  FUN_08023780(uVar1,0x8000);
  sVar3 = 4;
  do {
    bVar6 = sVar3 != 0;
    sVar3 = sVar3 + -1;
  } while (bVar6);
  for (bVar4 = 0; bVar4 < 0x10; bVar4 = bVar4 + 1) {
    uVar5 = uVar5 << 1;
    cVar2 = FUN_0802375e(DAT_0802746c,0x200);
    if (cVar2 != '\0') {
      uVar5 = uVar5 | 1;
    }
    FUN_08023788(uVar1,0x8000);
    sVar3 = 4;
    do {
      bVar6 = sVar3 != 0;
      sVar3 = sVar3 + -1;
    } while (bVar6);
    FUN_08023780(uVar1,0x8000);
    sVar3 = 4;
    do {
      bVar6 = sVar3 != 0;
      sVar3 = sVar3 + -1;
    } while (bVar6);
  }
  FUN_08026838(4);
  FUN_08023780(uVar1,0x8000);
  FUN_080266d4();
  FUN_08023788(uVar1,0x8000);
  return CONCAT44(param_4,uVar5) & 0xffffffff0000ffff;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080272e2 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
undefined4 recovery_fn(undefined1 param_1,char param_2,undefined4 param_3,undefined4 param_4)

{
  undefined1 *puVar1;
  undefined4 uVar2;
  
  uVar2 = FUN_0802b1e0();
  puVar1 = DAT_08028004;
  if (param_2 == '\x01') {
    *DAT_08028004 = 0x1f;
    *(undefined2 *)(puVar1 + 2) = 0x9ba9;
    FUN_0802689a(param_1);
  }
  else {
    *DAT_08028004 = 0x1f;
    *(undefined2 *)(puVar1 + 2) = 0x9aa9;
    FUN_0802689a(param_1);
  }
  FUN_0802b1e8(uVar2);
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08023610 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
uint recovery_fn(uint *param_1,uint *param_2,uint param_3)

{
  uint uVar1;
  uint uVar2;
  uint in_r12;
  uint uVar3;
  bool bVar4;
  
  uVar1 = 0;
  while (((uint)param_1 & 3) != 0) {
    bVar4 = param_3 != 0;
    param_3 = param_3 - 1;
    uVar2 = param_3;
    if (bVar4) {
      in_r12 = (uint)(byte)*param_2;
      uVar1 = (byte)*param_1 - in_r12;
      uVar2 = uVar1;
      param_2 = (uint *)((int)param_2 + 1);
      param_1 = (uint *)((int)param_1 + 1);
    }
    if (uVar2 != 0) {
      return uVar1;
    }
  }
  if (((uint)param_2 & 3) == 0) {
    do {
      uVar3 = param_3;
      uVar2 = uVar3 - 4;
      bVar4 = uVar2 == 0;
      if (3 < uVar3) {
        uVar1 = *param_1;
        in_r12 = *param_2;
        bVar4 = uVar1 == in_r12;
        param_2 = param_2 + 1;
        param_1 = param_1 + 1;
      }
      param_3 = uVar2;
    } while (bVar4);
    param_3 = uVar3;
    if (uVar2 < 0xfffffffc) {
      uVar1 = uVar1 << 0x18 | (uVar1 >> 8 & 0xff) << 0x10 | (uVar1 >> 0x10 & 0xff) << 8 |
              uVar1 >> 0x18;
      uVar3 = in_r12 << 0x18 | (in_r12 >> 8 & 0xff) << 0x10 | (in_r12 >> 0x10 & 0xff) << 8 |
              in_r12 >> 0x18;
      uVar2 = uVar1 - uVar3;
      bVar4 = uVar2 != 0;
      if (uVar1 < uVar3) {
        uVar2 = 0xffffffff;
      }
      if (uVar1 >= uVar3 && bVar4) {
        uVar2 = 1;
      }
      return uVar2;
    }
  }
  do {
    uVar3 = param_3;
    uVar2 = uVar3 - 1;
    if (uVar3 != 0) {
      uVar1 = (uint)(byte)*param_1 - (uint)(byte)*param_2;
      uVar2 = uVar1;
      param_2 = (uint *)((int)param_2 + 1);
      param_1 = (uint *)((int)param_1 + 1);
    }
    param_3 = uVar3 - 1;
  } while (uVar2 == 0);
  if (uVar3 == 0) {
    uVar1 = 0;
  }
  return uVar1;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801d9a0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
uint * recovery_fn(int param_1,uint param_2,uint param_3)

{
  undefined1 uVar1;
  undefined2 uVar2;
  uint *puVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  
  uVar5 = (param_3 & 0xff) << 8;
  uVar4 = param_3 & 0xff | uVar5;
  uVar5 = uVar4 | (param_3 & 0xffff00ff | uVar5) << 0x10;
  puVar3 = (uint *)(param_1 + param_2);
  uVar6 = (uint)puVar3 & 3;
  uVar1 = (undefined1)param_3;
  uVar2 = (undefined2)uVar4;
  if (uVar6 != 0) {
    uVar4 = param_2 - uVar6;
    if (param_2 < uVar6) {
      if (uVar4 + uVar6 != 0) {
        puVar3 = (uint *)((int)puVar3 + -1);
        *(undefined1 *)puVar3 = uVar1;
      }
      if ((uVar4 + uVar6 & 2) != 0) {
        puVar3 = (uint *)((int)puVar3 + -1);
        *(undefined1 *)puVar3 = uVar1;
      }
      return puVar3;
    }
    uVar6 = (uint)puVar3 & 2;
    if ((int)puVar3 * -0x80000000 < 0) {
      puVar3 = (uint *)((int)puVar3 + -1);
      *(undefined1 *)puVar3 = uVar1;
    }
    param_2 = uVar4;
    if (uVar6 != 0) {
      puVar3 = (uint *)((int)puVar3 + -2);
      *(undefined2 *)puVar3 = uVar2;
    }
  }
  do {
    uVar4 = param_2;
    param_2 = uVar4 - 0x10;
    if (0xf < uVar4) {
      puVar3[-1] = uVar5;
      puVar3[-2] = uVar5;
      puVar3[-3] = uVar5;
      puVar3 = puVar3 + -4;
      *puVar3 = uVar5;
    }
  } while (0xf < uVar4 && param_2 != 0);
  if ((param_2 & 8) != 0) {
    puVar3[-1] = uVar5;
    puVar3 = puVar3 + -2;
    *puVar3 = uVar5;
  }
  if ((int)(uVar4 * 0x20000000) < 0) {
    puVar3 = puVar3 + -1;
    *puVar3 = uVar5;
  }
  if ((uVar4 * 0x20000000 & 0x40000000) != 0) {
    puVar3 = (uint *)((int)puVar3 + -2);
    *(undefined2 *)puVar3 = uVar2;
  }
  if ((int)(uVar4 * -0x80000000) < 0) {
    puVar3 = (uint *)((int)puVar3 + -1);
    *(undefined1 *)puVar3 = uVar1;
  }
  return puVar3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08023000 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  int iVar1;
  int iVar2;
  undefined4 in_r3;
  
  iVar2 = _DAT_08023560;
  iVar1 = _DAT_0802307c;
  if (*(char *)((uint)*(byte *)(_DAT_08023560 + 0x40) + _DAT_0802307c + 0xba) != '\0') {
    *(undefined1 *)((uint)*(byte *)(_DAT_08023560 + 0x40) + _DAT_0802307c + 0xba) = 0;
    FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
    recovery_fn();
  }
  if (*(char *)((uint)*(byte *)(iVar2 + 0x40) + iVar1 + 0xbc) != '\0') {
    *(undefined1 *)((uint)*(byte *)(iVar2 + 0x40) + iVar1 + 0xbc) = 0;
    FUN_0801e296(*(undefined1 *)(iVar2 + 0x40));
    recovery_fn();
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
