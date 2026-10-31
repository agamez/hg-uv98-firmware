INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080201aa (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1)

{
  byte *pbVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  undefined4 uVar5;
  undefined4 uStack_44;
  undefined4 uStack_40;
  undefined4 uStack_3c;
  
  iVar4 = DAT_08020b30;
  iVar3 = DAT_080209c8;
  iVar2 = DAT_080203c4;
  uVar5 = FUN_0801a8aa((uint)*(byte *)(DAT_080203c4 + 0x40) * 0x1e + DAT_080209c8);
  *(undefined4 *)(iVar4 + 0x3b) = uVar5;
  FUN_0801f3bc(*(undefined4 *)(iVar4 + 0x3b),
               *(undefined2 *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 2 + 0x2e),
               iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
  uVar5 = FUN_0801a8aa((uint)*(byte *)(iVar2 + 0x40) * 0x1e + iVar3);
  *(undefined4 *)(iVar4 + 0x3b) = uVar5;
  pbVar1 = DAT_080203b0;
  if ((*DAT_080203b0 & 3) >> 1 == 0) {
    DAT_080203b0[4] = DAT_080203b0[4] & 0xfb;
  }
  else {
    DAT_080203b0[4] = DAT_080203b0[4] | 4;
  }
  if (param_1 == '\v') {
    if ((pbVar1[4] & 7) >> 2 == 0) {
      *(uint *)(iVar4 + 0x3b) =
           (uint)*(ushort *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 2 + 0x2e) +
           *(int *)(iVar4 + 0x3b);
    }
    else {
      *(int *)(iVar4 + 0x3b) = *(int *)(iVar4 + 0x3b) + 100000;
    }
    if ((*(uint *)(iVar4 + 0x3b) < *(uint *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x62)) ||
       (*(uint *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x5a) < *(uint *)(iVar4 + 0x3b))) {
      *(undefined4 *)(iVar4 + 0x3b) =
           *(undefined4 *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x62);
    }
  }
  else if (param_1 == '\f') {
    if ((pbVar1[4] & 7) >> 2 == 0) {
      *(uint *)(iVar4 + 0x3b) =
           *(int *)(iVar4 + 0x3b) -
           (uint)*(ushort *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 2 + 0x2e);
    }
    else {
      *(int *)(iVar4 + 0x3b) = _DAT_08020f9c + *(int *)(iVar4 + 0x3b);
    }
    if ((*(uint *)(iVar4 + 0x3b) < *(uint *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x62)) ||
       (*(uint *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x5a) < *(uint *)(iVar4 + 0x3b))) {
      *(undefined4 *)(iVar4 + 0x3b) =
           *(undefined4 *)(iVar4 + (uint)*(byte *)(iVar2 + 0x40) * 4 + 0x5a);
    }
  }
  FUN_0801a8da(*(undefined4 *)(iVar4 + 0x3b),iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
  if (-1 < (char)*pbVar1) {
    FUN_08022854(*(undefined1 *)(iVar2 + 0x40),iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
  }
  FUN_0801d8c0((uint)*(byte *)(iVar2 + 0x40) * 0x1e + iVar3 + 4,
               iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e,4);
  if ((*pbVar1 & 0xf) >> 3 == 0) {
    FUN_0801d8c0(&uStack_44,iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e,0x1e);
    FUN_0801ab36(*(undefined1 *)(iVar2 + 0x40),uStack_44,uStack_40,uStack_3c);
    if ((*pbVar1 & 0x7f) >> 6 == 0) {
      FUN_0801a7aa(*(undefined1 *)(iVar2 + 0x40),iVar3 + (uint)*(byte *)(iVar2 + 0x40) * 0x1e);
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080214e4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1)

{
  byte *pbVar1;
  int iVar2;
  byte *pbVar3;
  int iVar4;
  
  pbVar1 = _DAT_08021b94;
  if ((*_DAT_08021b94 & 0x7f) >> 6 == 0) {
    if (param_1 == '\0') {
      FUN_08014128(3);
    }
    iVar4 = DAT_08022040;
    pbVar3 = _DAT_08021bac;
    iVar2 = _DAT_08021b98;
    if (param_1 == '\x01') {
      if (*_DAT_08021bac == 0) {
        FUN_0801cd32(1,1,0x9e,0x10);
        FUN_0801cdfc();
        pbVar1[9] = pbVar1[9] | 0xc0;
        pbVar1[0xb] = pbVar1[0xb] & 0xfd;
        FUN_08014530(0x67);
        FUN_080100e4(0xff);
      }
      else if (*_DAT_08021bac == 0x67) {
        if (*(char *)(_DAT_08021b98 + 0x2c) == '\0') {
          FUN_0801b198(0);
        }
        else {
          (**(code **)((uint)*_DAT_08021bac * 0x10 + DAT_08022040 + 4))(10);
          if (pbVar3[2] != 0) {
            (**(code **)((uint)*pbVar3 * 0x10 + iVar4 + 8))(10);
            pbVar3[2] = 0;
          }
        }
      }
      else if (*_DAT_08021bac == 0x68) {
        (**(code **)((uint)*_DAT_08021bac * 0x10 + DAT_08022040 + 4))(0xf);
        if (pbVar3[2] != 0) {
          (**(code **)((uint)*pbVar3 * 0x10 + iVar4 + 8))(0xf);
          pbVar3[2] = 0;
        }
      }
      else if (*_DAT_08021bac == 0x6a) {
        (**(code **)((uint)*_DAT_08021bac * 0x10 + DAT_08022040 + 4))(0xf);
        if (pbVar3[2] != 0) {
          (**(code **)((uint)*pbVar3 * 0x10 + iVar4 + 8))(0xf);
          pbVar3[2] = 0;
        }
      }
    }
    else if (param_1 == '\x02') {
      if ((pbVar1[6] & 0x1f) >> 4 == 0) {
        pbVar1[6] = pbVar1[6] | 0x10;
        FUN_08023780(DAT_0802203c,0x4000);
      }
      else {
        pbVar1[6] = pbVar1[6] & 0xef;
        FUN_08023788(DAT_0802203c,0x4000);
      }
    }
    else if (param_1 == '\x03') {
      *(char *)(_DAT_08021b98 + 0x19) = *(char *)(_DAT_08021b98 + 0x19) + '\x01';
      if (9 < *(byte *)(iVar2 + 0x19)) {
        *(undefined1 *)(iVar2 + 0x19) = 0;
      }
      FUN_08024cf8((ushort)*(byte *)(iVar2 + 0x19) * 0x10 + 0x5f);
    }
  }
  else {
    recovery_fn(0xd);
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802601e (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(uint param_1)

{
  undefined4 uVar1;
  byte bVar2;
  
  FUN_08023780(_DAT_08026688,2);
  uVar1 = _DAT_0802668c;
  FUN_08023788(_DAT_0802668c,0x4000);
  FUN_08025fc0(6);
  for (bVar2 = 0; bVar2 < 8; bVar2 = bVar2 + 1) {
    if ((int)(param_1 << 0x1f) < 0) {
      FUN_08023780(uVar1,0x4000);
      FUN_08025fa8(6);
      FUN_08023788(uVar1,0x4000);
      FUN_08025fa8(2);
    }
    else {
      FUN_08023780(uVar1,0x4000);
      FUN_08025fa8(2);
      FUN_08023788(uVar1,0x4000);
      FUN_08025fa8(6);
    }
    param_1 = (param_1 & 0xff) >> 1;
  }
  FUN_08023780(uVar1,0x4000);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801ea04 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  int iVar1;
  int iVar2;
  undefined1 uVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  undefined4 uVar7;
  undefined1 auStack_30 [3];
  undefined1 auStack_2d [3];
  undefined1 uStack_2a;
  undefined1 uStack_29;
  
  iVar6 = DAT_0801f058;
  if (*(char *)(DAT_0801f058 + 1) == '\x03') {
    *(char *)(DAT_0801f058 + 1) = *(char *)(DAT_0801f058 + 1) + '\x01';
  }
  *(char *)((uint)*(byte *)(iVar6 + 1) + iVar6 + 2) = param_1 + '0';
  iVar1 = DAT_0801f024;
  if (*(char *)(iVar6 + 1) == '\0') {
    if ((*(char *)((uint)*(byte *)(DAT_0801f024 + 0x40) + DAT_0801f024 + 8) == '\x01') &&
       ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0)) {
      FUN_0801d9a0(iVar6 + 3,0x14,0);
      if (*(byte *)(iVar6 + 2) < 0x32) {
        FUN_0801d8c0(iVar6 + 3,0x801ea94,2);
      }
      else {
        *(undefined1 *)(iVar6 + 3) = *(undefined1 *)(iVar6 + 2);
        *(undefined1 *)(iVar6 + 2) = 0x20;
        *(undefined1 *)(iVar6 + 4) = 0x2d;
        *(char *)(iVar6 + 1) = *(char *)(iVar6 + 1) + '\x01';
      }
    }
    else {
      FUN_0801d9a0(iVar6 + 3,0x14,0);
      FUN_0801d8c0(iVar6 + 3,DAT_0801f728,8);
    }
    if (*(char *)(iVar1 + 0x40) == '\0') {
      if ((*(char *)(iVar1 + 8) == '\x01') && ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0)) {
        if (*(char *)(iVar1 + 0xc) == '\x01') {
          FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
        }
        else {
          FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
        }
      }
      else if (*(char *)(iVar1 + 0xc) == '\x01') {
        FUN_0801cc0c(0x1d,0x16,iVar6 + 2,4,7,6);
        if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
          FUN_0801cc0c(0x23,0x6c,iVar6 + 9,4,2,3);
        }
      }
      else {
        FUN_0801cc0c(0x32,0xe,iVar6 + 2,4,4,9);
        FUN_0801cc0c(0x32,0x46,iVar6 + 6,4,3,9);
        if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
          FUN_0801cc0c(0x40,0x78,iVar6 + 9,4,2,4);
        }
      }
    }
    else if ((*(char *)(iVar1 + 9) == '\x01') && ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0)) {
      if (*(char *)(iVar1 + 0xc) == '\x01') {
        FUN_0801cc0c(0x56,0x85,iVar6 + 2,4,3,1);
      }
      else {
        FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
      }
    }
    else if (*(char *)(iVar1 + 0xc) == '\x01') {
      FUN_0801cc0c(0x4c,0x16,iVar6 + 2,4,7,6);
      if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
        FUN_0801cc0c(0x52,0x6c,iVar6 + 9,4,2,3);
      }
    }
    else {
      FUN_0801cc0c(0x32,0xe,iVar6 + 2,4,4,9);
      FUN_0801cc0c(0x32,0x46,iVar6 + 6,4,3,9);
      if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
        FUN_0801cc0c(0x40,0x78,iVar6 + 9,4,2,4);
      }
    }
  }
  *(char *)(iVar6 + 1) = *(char *)(iVar6 + 1) + '\x01';
  iVar2 = DAT_0801f958;
  *(undefined1 *)(DAT_0801f958 + 2) = 0;
  iVar1 = DAT_0801f024;
  if (*(char *)(DAT_0801f024 + 0x40) == '\0') {
    if ((*(char *)(DAT_0801f024 + 8) == '\x01') && ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0))
    {
      if (*(char *)(DAT_0801f024 + 0xc) == '\x01') {
        FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
      }
      else {
        FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
      }
      if (*(char *)(iVar6 + 1) == '\x03') {
        *(undefined1 *)(iVar6 + 1) = 0;
        uVar3 = FUN_0801a426(iVar6 + 2,3);
        iVar4 = FUN_0801fd1e(uVar3,0);
        if (iVar4 != 0) {
          *(undefined1 *)((uint)*(byte *)(iVar1 + 0x40) + iVar1 + 0x41) = uVar3;
          FUN_0801a7d4(*(undefined1 *)((uint)*(byte *)(iVar1 + 0x40) + iVar1 + 0x41),DAT_0801f968);
          FUN_08022a98();
          FUN_0801a6ea();
        }
        *(undefined1 *)(iVar2 + 2) = 1;
      }
    }
    else if (*(char *)(DAT_0801f024 + 0xc) == '\x01') {
      FUN_0801cc0c(0x1d,0x16,iVar6 + 2,4,7,6);
      if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
        FUN_0801cc0c(0x23,0x6c,iVar6 + 9,4,2,3);
      }
    }
    else {
      FUN_0801cc0c(0x32,0xe,iVar6 + 2,4,4,9);
      FUN_0801cc0c(0x32,0x46,iVar6 + 6,4,3,9);
      if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
        FUN_0801cc0c(0x40,0x78,iVar6 + 9,4,2,4);
      }
    }
  }
  else if ((*(char *)(DAT_0801f024 + 9) == '\x01') &&
          ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0)) {
    if (*(char *)(DAT_0801f024 + 0xc) == '\x01') {
      FUN_0801cc0c(0x56,0x85,iVar6 + 2,4,3,1);
    }
    else {
      FUN_0801cc0c(0x27,0x85,iVar6 + 2,4,3,1);
    }
    if (*(char *)(iVar6 + 1) == '\x03') {
      *(undefined1 *)(iVar6 + 1) = 0;
      uVar3 = FUN_0801a426(iVar6 + 2,3);
      iVar4 = FUN_0801fd1e(uVar3,0);
      if (iVar4 != 0) {
        *(undefined1 *)((uint)*(byte *)(iVar1 + 0x40) + iVar1 + 0x41) = uVar3;
        FUN_0801a7d4(*(undefined1 *)((uint)*(byte *)(iVar1 + 0x40) + iVar1 + 0x41),DAT_0801f5c8);
        FUN_08022a98();
        FUN_0801a6ea();
      }
      *(undefined1 *)(iVar2 + 2) = 1;
    }
  }
  else if (*(char *)(DAT_0801f024 + 0xc) == '\x01') {
    FUN_0801cc0c(0x4c,0x16,iVar6 + 2,4,7,6);
    if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
      FUN_0801cc0c(0x52,0x6c,iVar6 + 9,4,2,3);
    }
  }
  else {
    FUN_0801cc0c(0x32,0xe,iVar6 + 2,4,4,9);
    FUN_0801cc0c(0x32,0x46,iVar6 + 6,4,3,9);
    if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
      FUN_0801cc0c(0x40,0x78,iVar6 + 9,4,2,4);
    }
  }
  if (*(char *)(iVar6 + 1) == '\a') {
    *(undefined1 *)(iVar6 + 1) = 0;
    iVar4 = DAT_0801f3f4;
    if ((*(byte *)(DAT_0801f3f4 + 9) & 0x1f) >> 3 == 0) {
      FUN_0801d8c0(auStack_30,iVar6 + 2,3);
      FUN_0801d8c0(auStack_2d,iVar6 + 6,5);
      uStack_2a = 0x30;
      uStack_29 = 0x30;
      uVar7 = FUN_0801a426(auStack_30,8);
      iVar4 = FUN_0801f33e(uVar7);
      iVar6 = DAT_0801f968;
      if (iVar4 == 0) {
        FUN_0801a52c((uint)*(byte *)(iVar1 + 0x40) * 0x1e + DAT_0801f968,auStack_30,8);
        FUN_0801a4dc((uint)*(byte *)(iVar1 + 0x40) * 0x1e + iVar6,4);
        FUN_08023084(auStack_30,iVar6 + (uint)*(byte *)(iVar1 + 0x40) * 0x1e);
        FUN_08022854(*(undefined1 *)(iVar1 + 0x40),iVar6 + (uint)*(byte *)(iVar1 + 0x40) * 0x1e);
        FUN_0801d8c0((uint)*(byte *)(iVar1 + 0x40) * 0x1e + iVar6 + 4,
                     iVar6 + (uint)*(byte *)(iVar1 + 0x40) * 0x1e,4);
        FUN_08022a98();
        FUN_0801a7aa(*(undefined1 *)(iVar1 + 0x40),iVar6 + (uint)*(byte *)(iVar1 + 0x40) * 0x1e);
        FUN_08012f0c();
      }
    }
    else {
      iVar5 = FUN_08023610(iVar6 + 2,iVar1 + 0x30,3);
      if ((iVar5 == 0) && (iVar6 = FUN_08023610(iVar6 + 6,iVar1 + 0x33,3), iVar6 == 0)) {
        *(byte *)(iVar4 + 9) = *(byte *)(iVar4 + 9) & 0xe7 | 0x10;
        FUN_08022cb4();
      }
    }
    *(undefined1 *)(iVar2 + 2) = 1;
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
