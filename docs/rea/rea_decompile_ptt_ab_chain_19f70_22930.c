INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019f70 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(int param_1,int param_2,int param_3)

{
  int unaff_r4;
  int unaff_r5;
  undefined4 uStack_c;
  undefined4 uStack_8;
  undefined4 uStack_4;
  
  *(byte *)(param_2 + 0xc) =
       *(byte *)(param_1 * (uint)*(byte *)(unaff_r4 + 0x40) + unaff_r5 + 0xc) & 0xfe |
       ~*(byte *)(param_3 + unaff_r5 + 0xc) & 1;
  FUN_0801d8c0(&uStack_c,unaff_r5 + param_1 * (uint)*(byte *)(unaff_r4 + 0x40),0x1e);
  FUN_0801acfc(*(undefined1 *)(unaff_r4 + 0x40),uStack_c,uStack_8,uStack_4);
  func_0x0801f104(*(undefined1 *)(unaff_r4 + 0x40));
  if (*(char *)(unaff_r4 + 0x40) == '\0') {
    FUN_0801f57c(0x32,0x16,*(byte *)(unaff_r5 + 0xc) & 1,1);
  }
  else {
    FUN_0801f57c(0x61,0x16,*(byte *)(unaff_r5 + 0x2a) & 1,1);
  }
  FUN_0801db54(*(undefined1 *)(unaff_r4 + 0x40));
  FUN_0801baa4(200);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08022930 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  undefined4 uVar2;
  
  if ((*(byte *)(_DAT_08022a90 + 9) & 0x1f) >> 3 == 0) {
    if (param_1 == '\0') {
      FUN_0801d072(0x1f,0xd,_DAT_0802355c,4,param_3,param_4);
      uVar2 = 3;
      uVar1 = 1;
      FUN_0801cc0c(0x4e,0xd,0x8022ba0,4);
    }
    else {
      uVar2 = 3;
      uVar1 = 1;
      FUN_0801cc0c(0x1f,0xd,0x8022ba0,4);
      FUN_0801d072(0x4e,0xd,_DAT_0802355c,4);
    }
  }
  else if (param_1 == '\0') {
    uVar2 = 3;
    uVar1 = 1;
    FUN_0801cc0c(0x4e,0xd,0x8022ba0,4);
  }
  else {
    uVar2 = 3;
    uVar1 = 1;
    FUN_0801cc0c(0x1f,0xd,0x8022ba0,4);
  }
  return CONCAT44(uVar2,uVar1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801e286 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
undefined4 recovery_fn(void)

{
  undefined4 unaff_r7;
  
  FUN_0801e5e0();
  return unaff_r7;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801e240 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1)

{
  if (*_DAT_0801e2c4 == '\0') {
    if (*(char *)(_DAT_0801e2c8 + 0xc) == '\x01') {
      if (param_1 == '\0') {
        FUN_0801cd32(0x32,0x1e,0x5a,10);
      }
      else {
        FUN_0801cd32(0x61,0x1e,0x5a,10);
      }
    }
    else {
      FUN_0801cd32(0x55,0x1e,0x5a,10);
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08022a24 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  byte bVar2;
  uint uVar3;
  
  iVar1 = _DAT_08023564;
  bVar2 = FUN_0801f28e(*(undefined1 *)(_DAT_08023564 + 3));
  uVar3 = FUN_0801f28e(*(undefined1 *)(iVar1 + 0x21));
  if (bVar2 == uVar3) {
    *(byte *)(_DAT_08022a90 + 2) = *(byte *)(_DAT_08022a90 + 2) & 0xfd;
    FUN_08025f04(0,bVar2);
    FUN_08025e74(1,'\x01' - bVar2);
  }
  else {
    *(byte *)(_DAT_08022a90 + 2) = *(byte *)(_DAT_08022a90 + 2) | 2;
    FUN_08025f04(0,bVar2);
    FUN_08025f04(1,'\x01' - bVar2);
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08022a98 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  int iVar2;
  int iVar3;
  undefined4 uStack_34;
  undefined4 uStack_30;
  undefined4 uStack_2c;
  
  recovery_fn();
  iVar3 = _DAT_08023568;
  iVar2 = _DAT_08023564;
  iVar1 = DAT_08022b60;
  if (((*(byte *)(_DAT_08023568 + 2) & 3) >> 1 == 0) || (*(char *)(DAT_08022b60 + 0xc) != '\x01')) {
    FUN_0801d8c0(&uStack_34,_DAT_08023564 + (uint)*(byte *)(DAT_08022b60 + 0x40) * 0x1e,0x1e);
    FUN_0801ab36(*(undefined1 *)(iVar1 + 0x40),uStack_34,uStack_30,uStack_2c);
    *(byte *)(iVar3 + 1) = *(byte *)(iVar3 + 1) & 0xef | (*(byte *)(iVar1 + 0x40) & 1) << 4;
    *(byte *)(iVar3 + 1) = *(byte *)(iVar3 + 1) & 0xdf | (*(byte *)(iVar1 + 0x40) & 1) << 5;
  }
  else {
    FUN_0801d8c0(&uStack_34,(uint)*(byte *)(DAT_08022b60 + 0x40) * -0x1e + _DAT_08023564 + 0x1e,0x1e
                );
    FUN_0801ab36('\x01' - *(char *)(iVar1 + 0x40),uStack_34,uStack_30,uStack_2c);
    FUN_0801d8c0(&uStack_34,iVar2 + (uint)*(byte *)(iVar1 + 0x40) * 0x1e,0x1e);
    FUN_0801ab36(*(undefined1 *)(iVar1 + 0x40),uStack_34,uStack_30,uStack_2c);
    *(byte *)(iVar3 + 1) = *(byte *)(iVar3 + 1) & 0xef | (*(byte *)(iVar1 + 0x40) & 1) << 4;
    *(byte *)(iVar3 + 1) = *(byte *)(iVar3 + 1) & 0xdf | (*(byte *)(iVar1 + 0x40) & 1) << 5;
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
