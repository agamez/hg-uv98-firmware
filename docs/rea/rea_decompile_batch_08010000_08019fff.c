INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801065c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(uint param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  uint uVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  iVar2 = DAT_08010a84;
  iVar1 = DAT_08010a7c;
  uVar3 = param_1 & 0xff;
  uStack_24 = param_2;
  uStack_20 = param_3;
  uStack_1c = param_4;
  if (uVar3 == 10) {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      FUN_08014128(6);
      FUN_08014530(0x69);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*DAT_08010a90 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08010a94,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,DAT_08010a98,4);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08010a9c + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xb) {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(DAT_08010a84 + 0x129) % 5) * 0x10 + 0x16,2,0x80109ac,4,3);
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + -1;
      if (99 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 99;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 5 == 4) {
        FUN_08014128(5);
        FUN_0801cdfc();
        if (*DAT_08010a90 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08010a94,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,DAT_08010a98,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 5;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 5) * 0x10 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xc) {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(DAT_08010a84 + 0x129) % 5) * 0x10 + 0x16,2,0x80109ac,4,3);
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + '\x01';
      if (99 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 0;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 5 == 0) {
        FUN_08014128(5);
        FUN_0801cdfc();
        if (*DAT_08010a90 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08010a94,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,DAT_08010a98,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 5;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 5) * 0x10 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xd) {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      FUN_08014530(0x67);
      FUN_080100e4(0xff);
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xf) {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      *(undefined1 *)(DAT_08010a84 + 0x129) = 0;
      FUN_08014128(6);
      FUN_08014530(0x6a);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*DAT_08010a90 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08010a94,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,DAT_08010a98,4);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08010a9c + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  return CONCAT44(uStack_24,param_1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08010aa0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(uint param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  uint uVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  iVar2 = _DAT_08011440;
  iVar1 = DAT_08011300;
  uVar3 = param_1 & 0xff;
  uStack_24 = param_2;
  uStack_20 = param_3;
  uStack_1c = param_4;
  if (uVar3 == 10) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_08014128(6);
      FUN_08014530(0x8c);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*_DAT_08011444 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08011448,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08011450 + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xb) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(_DAT_08011440 + 0x129) % 6) * 0x30 + 0x16 & 0xff,2,0x8010e10,4,3
                  );
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + -1;
      if (0x18 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 0x18;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 2 == 0) {
        FUN_08014128(0x12);
        FUN_0801cdfc();
        if (*_DAT_08011444 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08011448,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 2;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 2) * 0x30 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xc) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(_DAT_08011440 + 0x129) % 2) * 0x30 + 0x16,2,0x8010e10,4,3);
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + '\x01';
      if (0x18 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 0;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 2 == 0) {
        FUN_08014128(5);
        FUN_0801cdfc();
        if (*_DAT_08011444 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08011448,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 2;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 2) * 0x30 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xd) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      FUN_08014530(0x6a);
      FUN_0801219c(0xff);
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xf) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      *(undefined1 *)(_DAT_08011440 + 0x129) = 0;
      FUN_08014128(0x14);
      FUN_08014530(0x6a);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*_DAT_08011444 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08011448,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4);
      }
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08011450 + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  return CONCAT44(uStack_24,param_1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08010e20 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(undefined4 param_1)

{
  int iVar1;
  int iVar2;
  undefined1 auStack_34 [12];
  undefined4 uStack_28;
  
  iVar2 = _DAT_08011454;
  uStack_28 = param_1;
  recovery_fn(_DAT_08011454,1000,0);
  FUN_0801d8c0(iVar2 + 5,_DAT_08011458,0x96);
  recovery_fn(auStack_34,10,0);
  iVar1 = _DAT_08011440;
  FUN_080235d0(auStack_34,DAT_080112fc,*(byte *)(_DAT_08011440 + 0x129) + 1);
  FUN_0801cb60(1,6,auStack_34,4,3);
  FUN_0801d072(((uint)*(byte *)(iVar1 + 0x129) % 6) * 0x30 + 0x16 & 0xff,2,DAT_080112f8,4);
  func_0x0801d68c(0,0x16,0xe,iVar2 + 5,4,2,6);
  *(byte *)(DAT_08011300 + 0xc) = *(byte *)(DAT_08011300 + 0xc) & 0xfd;
  FUN_0801cd32(0x32,1,0x9e,4);
  FUN_0801fc50();
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08010e6a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int unaff_r11;
  undefined4 uVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  
  FUN_080235d0(&stack0xfffffff8,DAT_080112fc,*(byte *)(unaff_r11 + 0x129) + 1,param_4,param_3);
  FUN_0801cb60(1,6,&stack0xfffffff8,4,3);
  FUN_0801d072(((uint)*(byte *)(unaff_r11 + 0x129) % 6) * 0x30 + 0x16 & 0xff,2,DAT_080112f8,4);
  uVar3 = 6;
  uVar2 = 2;
  uVar1 = 4;
  func_0x0801d68c(0,0x16,0xe);
  *(byte *)(DAT_08011300 + 0xc) = *(byte *)(DAT_08011300 + 0xc) & 0xfd;
  FUN_0801cd32(0x32,1,0x9e,4,uVar1,uVar2,uVar3);
  FUN_0801fc50();
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08010ef8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(uint param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  uint uVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  iVar2 = _DAT_08011440;
  iVar1 = DAT_08011300;
  uVar3 = param_1 & 0xff;
  uStack_24 = param_2;
  uStack_20 = param_3;
  uStack_1c = param_4;
  if (uVar3 == 10) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_08014128(6);
      FUN_08014530(0x69);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*_DAT_08011444 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08011448,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08011450 + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xb) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(_DAT_08011440 + 0x129) % 5) * 0x10 + 0x16,2,0x801124c,4,3);
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + -1;
      if (99 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 99;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 5 == 4) {
        FUN_08014128(5);
        FUN_0801cdfc();
        if (*_DAT_08011444 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08011448,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 5;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 5) * 0x10 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xc) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cb60(((uint)*(byte *)(_DAT_08011440 + 0x129) % 5) * 0x10 + 0x16,2,0x801124c,4,3);
      *(char *)(iVar2 + 0x129) = *(char *)(iVar2 + 0x129) + '\x01';
      if (99 < *(byte *)(iVar2 + 0x129)) {
        *(undefined1 *)(iVar2 + 0x129) = 0;
      }
      if ((uint)*(byte *)(iVar2 + 0x129) % 5 == 0) {
        FUN_08014128(5);
        FUN_0801cdfc();
        if (*_DAT_08011444 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08011448,4,3);
        }
        else {
          FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4,3);
        }
      }
      *(byte *)(iVar2 + 0x12a) = *(byte *)(iVar2 + 0x129) / 5;
      FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 5) * 0x10 + 0x16,2,DAT_080112f8,4);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,DAT_080112fc,*(byte *)(iVar2 + 0x129) + 1);
      param_1 = 3;
      FUN_0801cb60(1,6,&uStack_24,4);
    }
    else {
      FUN_0801baa4(10);
    }
  }
  else if (uVar3 == 0xd) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      FUN_08014530(0x67);
      FUN_080100e4(0xff);
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  else if (uVar3 == 0xf) {
    if ((*(byte *)(DAT_08011300 + 0xc) & 3) >> 1 == 0) {
      *(undefined1 *)(_DAT_08011440 + 0x129) = 0;
      FUN_08014128(6);
      FUN_08014530(0x6a);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*_DAT_08011444 == '\x01') {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08011448,4);
      }
      else {
        param_1 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) | 0xc0;
      FUN_0801baa4(5);
      *(undefined1 *)(_DAT_08011450 + 2) = 0;
    }
    else {
      FUN_0801baa4(100);
    }
  }
  return CONCAT44(uStack_24,param_1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08011254 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  int iVar2;
  undefined4 in_r3;
  undefined1 auStack_28 [12];
  undefined4 uStack_1c;
  
  iVar1 = _DAT_08011454;
  uStack_1c = in_r3;
  iVar2 = recovery_fn(_DAT_08011454,0x801143c,2);
  if (iVar2 == 0) {
    recovery_fn(auStack_28,10,0);
    iVar2 = _DAT_08011440;
    FUN_080235d0(auStack_28,DAT_080112fc,*(byte *)(_DAT_08011440 + 0x129) + 1);
    FUN_0801cb60(1,6,auStack_28,4,3);
    FUN_0801d072(((uint)*(byte *)(iVar2 + 0x129) % 5) * 0x10 + 0x16,2,DAT_080112f8,4);
    FUN_0801d4f8(0,0x16,0xe,iVar1 + 5,4,2);
    *(byte *)(_DAT_08011f90 + 0xc) = *(byte *)(_DAT_08011f90 + 0xc) & 0xfd;
    FUN_0801cd32(0x32,1,0x9e,4);
    FUN_0801fc50();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08011304 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined4 uStack_10;
  
  iVar2 = _DAT_08011f90;
  iVar1 = _DAT_08011440;
  uStack_10 = param_4;
  if (param_1 == '\n') {
    FUN_0801b198(0);
  }
  else if (param_1 == '\v') {
    if (*(char *)(_DAT_08011f94 + 8) == '\0') {
      *(char *)(_DAT_08011440 + 0x12a) = *(char *)(_DAT_08011440 + 0x12a) + -1;
      if (3 < *(byte *)(iVar1 + 0x12a)) {
        *(undefined1 *)(iVar1 + 0x12a) = 3;
      }
    }
    else {
      *(char *)(_DAT_08011440 + 0x12a) = *(char *)(_DAT_08011440 + 0x12a) + -1;
      if (4 < *(byte *)(iVar1 + 0x12a)) {
        *(undefined1 *)(iVar1 + 0x12a) = 4;
      }
    }
    *(undefined1 *)(_DAT_08011450 + 2) = 1;
  }
  else if (param_1 == '\f') {
    if (*(char *)(_DAT_08011f94 + 8) == '\0') {
      *(char *)(_DAT_08011440 + 0x12a) = *(char *)(_DAT_08011440 + 0x12a) + '\x01';
      if (3 < *(byte *)(iVar1 + 0x12a)) {
        *(undefined1 *)(iVar1 + 0x12a) = 0;
      }
    }
    else {
      *(char *)(_DAT_08011440 + 0x12a) = *(char *)(_DAT_08011440 + 0x12a) + '\x01';
      if (4 < *(byte *)(iVar1 + 0x12a)) {
        *(undefined1 *)(iVar1 + 0x12a) = 0;
      }
    }
    *(undefined1 *)(_DAT_08011450 + 2) = 1;
  }
  else if (param_1 == '\r') {
    if ((*(byte *)(_DAT_08011f90 + 0xb) & 1) == 0) {
      FUN_08014128(5);
      FUN_08014530(0x68);
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      if (*_DAT_08011444 == '\x01') {
        FUN_0801cb60(0x30,0x2c,_DAT_08011448,4);
      }
      else {
        FUN_0801cb60(0x30,0x2c,_DAT_0801144c,4);
      }
      uStack_10 = 3;
      *(byte *)(iVar2 + 9) = *(byte *)(iVar2 + 9) | 0xc0;
      *(undefined1 *)(_DAT_08011450 + 2) = 0;
    }
    else {
      *(byte *)(_DAT_08011f90 + 0xb) = *(byte *)(_DAT_08011f90 + 0xb) & 0xfe;
      FUN_0801b198(0);
    }
  }
  else if (param_1 == '\x0f') {
    FUN_0801b198(0);
  }
  return uStack_10;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08011f98 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined4 uStack_10;
  
  iVar2 = _DAT_08012194;
  iVar1 = _DAT_0801210c;
  uStack_10 = param_4;
  if ((*(byte *)(_DAT_08012194 + 0xc) & 3) >> 1 == 0) {
    if (param_1 == '\n') {
      FUN_0801b198(0);
    }
    else if (param_1 == '\v') {
      if (*(char *)(_DAT_08012198 + 8) == '\0') {
        *(char *)(_DAT_0801210c + 0x12a) = *(char *)(_DAT_0801210c + 0x12a) + -1;
        if (3 < *(byte *)(iVar1 + 0x12a)) {
          *(undefined1 *)(iVar1 + 0x12a) = 3;
        }
      }
      else {
        *(char *)(_DAT_0801210c + 0x12a) = *(char *)(_DAT_0801210c + 0x12a) + -1;
        if (4 < *(byte *)(iVar1 + 0x12a)) {
          *(undefined1 *)(iVar1 + 0x12a) = 4;
        }
      }
      *(undefined1 *)(_DAT_08012cf0 + 2) = 1;
    }
    else if (param_1 == '\f') {
      if (*(char *)(_DAT_08012198 + 8) == '\0') {
        *(char *)(_DAT_0801210c + 0x12a) = *(char *)(_DAT_0801210c + 0x12a) + '\x01';
        if (3 < *(byte *)(iVar1 + 0x12a)) {
          *(undefined1 *)(iVar1 + 0x12a) = 0;
        }
      }
      else {
        *(char *)(_DAT_0801210c + 0x12a) = *(char *)(_DAT_0801210c + 0x12a) + '\x01';
        if (4 < *(byte *)(iVar1 + 0x12a)) {
          *(undefined1 *)(iVar1 + 0x12a) = 0;
        }
      }
      *(undefined1 *)(_DAT_08012cf0 + 2) = 1;
    }
    else if (param_1 == '\r') {
      if ((*(byte *)(_DAT_08012194 + 0xb) & 1) == 0) {
        FUN_08014128(5);
        FUN_08014530(0x68);
        FUN_0801cd32(1,1,0x9e,0x10);
        FUN_0801cdfc();
        if (*_DAT_080120f4 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08012ce8,4);
        }
        else {
          FUN_0801cb60(0x30,0x2c,_DAT_08012cec,4);
        }
        uStack_10 = 3;
        *(byte *)(iVar2 + 9) = *(byte *)(iVar2 + 9) | 0xc0;
        *(undefined1 *)(_DAT_08012cf0 + 2) = 0;
      }
      else {
        *(byte *)(_DAT_08012194 + 0xb) = *(byte *)(_DAT_08012194 + 0xb) & 0xfe;
        FUN_0801b198(0);
      }
    }
    else if (param_1 == '\x0f') {
      FUN_0801b198(0);
    }
  }
  else {
    FUN_0801baa4(200);
  }
  return uStack_10;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08012fcc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135ac);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801300c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_080135b0 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013044 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135bc);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013088 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_080135c0 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080130c4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135c4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013104 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080135c8 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801313c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_080135cc);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801317c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080135d0 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080131b4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135d4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080131f0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,6,*(undefined4 *)(_DAT_080135d8 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013228 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135dc);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801326c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x3b,*(int *)(_DAT_080135e0 + (uint)*_DAT_080135b4 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080132ac (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135e4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080132e8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080135e8 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013318 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_080135ec);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013354 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_080135f0 + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013384 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_10;
  
  uStack_10 = param_3;
  if (param_1 == 10) {
    FUN_0801cdfc();
    if (*(char *)(_DAT_080135b8 + 5) == '\0') {
      FUN_08014128(8);
      if (*_DAT_080135b4 == '\x01') {
        FUN_0801cb60(0x30,0x1e,_DAT_080135f4,4);
      }
      else {
        FUN_0801cb60(0x30,0x1e,_DAT_080135f8,4);
      }
      uStack_10 = 3;
      FUN_0801baa4(500);
    }
    else {
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return CONCAT44(param_4,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013408 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080135fc + (uint)*_DAT_080135b4 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_080135b8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013438 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_10;
  
  uStack_10 = param_3;
  if (param_1 == 10) {
    FUN_0801cdfc();
    if (*(char *)(_DAT_080135b8 + 5) == '\0') {
      FUN_08014128(7);
      if (*_DAT_080135b4 == '\x01') {
        uStack_10 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08013600,4);
      }
      else {
        uStack_10 = 3;
        FUN_0801cb60(0x30,0x2c,_DAT_08013604,4);
      }
    }
    else {
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return CONCAT44(param_4,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013610 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

ulonglong recovery_fn(int param_1,ushort param_2)

{
  int iVar1;
  short sVar2;
  ushort uVar3;
  ushort uVar4;
  
  iVar1 = DAT_08014124;
  sVar2 = 0;
  uVar3 = 0;
  recovery_fn(DAT_08014124,0x15b,0);
  if ((*(char *)(param_1 + 5) == 'W') && (*(char *)(param_1 + 6) == ',')) {
    *(undefined1 *)(iVar1 + 8) = 1;
    for (uVar4 = 0; uVar4 < param_2; uVar4 = uVar4 + 1) {
      if ((*(char *)(param_1 + (uint)uVar4) == ',') ||
         ((*(char *)(param_1 + (uint)uVar4) == '\r' &&
          (*(char *)((uint)uVar4 + param_1 + 1) == '\n')))) {
        if (sVar2 != 0) {
          if (sVar2 == 2) {
            FUN_0801d8c0(iVar1 + 9,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 3) {
            FUN_0801d8c0(iVar1 + 0x24,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 4) {
            FUN_0801d8c0(iVar1 + 0x3f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 5) {
            FUN_0801d8c0(iVar1 + 0x49,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 6) {
            FUN_0801d8c0(iVar1 + 0x58,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 7) {
            FUN_0801d8c0(iVar1 + 0x5f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 8) {
            FUN_0801d8c0(iVar1 + 0x67,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 9) {
            FUN_0801d8c0(iVar1 + 0x74,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 10) {
            FUN_0801d8c0(iVar1 + 0x81,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xb) {
            FUN_0801d8c0(iVar1 + 0x85,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xc) {
            FUN_0801d8c0(iVar1 + 0x89,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xd) {
            FUN_0801d8c0(iVar1 + 0x8d,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xe) {
            FUN_0801d8c0(iVar1 + 0x97,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xf) {
            FUN_0801d8c0(iVar1 + 0x9f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x10) {
            FUN_0801d8c0(iVar1 + 0xab,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x11) {
            FUN_0801d8c0(iVar1 + 0xb7,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x12) {
            FUN_0801d8c0(iVar1 + 0xc3,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x13) {
            FUN_0801d8c0(iVar1 + 0xcf,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x14) {
            FUN_0801d8c0(iVar1 + 0xdb,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
        }
        uVar3 = 0;
        sVar2 = sVar2 + 1;
      }
      uVar3 = uVar3 + 1;
    }
  }
  else {
    if (*(char *)(iVar1 + 8) != '\0') {
      *(byte *)(_DAT_080144a8 + 0xc) = *(byte *)(_DAT_080144a8 + 0xc) | 0x20;
    }
    *(undefined1 *)(iVar1 + 8) = 0;
    for (uVar4 = 0; uVar4 < param_2; uVar4 = uVar4 + 1) {
      if ((*(char *)(param_1 + (uint)uVar4) == ',') ||
         ((*(char *)(param_1 + (uint)uVar4) == '\r' &&
          (*(char *)((uint)uVar4 + param_1 + 1) == '\n')))) {
        if (sVar2 != 0) {
          if (sVar2 == 1) {
            FUN_0801d8c0(iVar1 + 0x3f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 2) {
            FUN_0801d8c0(iVar1 + 0x49,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 3) {
            FUN_0801d8c0(iVar1 + 0x58,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 4) {
            FUN_0801d8c0(iVar1 + 0x5f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 5) {
            FUN_0801d8c0(iVar1 + 0x67,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 6) {
            FUN_0801d8c0(iVar1 + 0x74,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 7) {
            FUN_0801d8c0(iVar1 + 0x81,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 8) {
            FUN_0801d8c0(iVar1 + 0x85,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 9) {
            FUN_0801d8c0(iVar1 + 0x89,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 10) {
            FUN_0801d8c0(iVar1 + 0x8d,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xb) {
            FUN_0801d8c0(iVar1 + 0x97,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xc) {
            FUN_0801d8c0(iVar1 + 0x9f,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xd) {
            FUN_0801d8c0(iVar1 + 0xab,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xe) {
            FUN_0801d8c0(iVar1 + 0xb7,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0xf) {
            FUN_0801d8c0(iVar1 + 0xc3,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x10) {
            FUN_0801d8c0(iVar1 + 0xcf,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
          else if (sVar2 == 0x11) {
            FUN_0801d8c0(iVar1 + 0xdb,(((uint)uVar4 + param_1) - (uint)uVar3) + 1,uVar3 - 1);
          }
        }
        uVar3 = 0;
        sVar2 = sVar2 + 1;
      }
      uVar3 = uVar3 + 1;
    }
  }
  return (ulonglong)CONCAT24(param_2,param_1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08014d60 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(void)

{
  uint uVar1;
  byte bVar2;
  char cVar3;
  code *UNRECOVERED_JUMPTABLE;
  char cStack00000000;
  undefined1 uStack00000001;
  byte bStack00000002;
  byte bStack00000004;
  byte bStack00000005;
  int in_stack_00000008;
  undefined1 uStack0000000c;
  undefined1 uStack0000000d;
  byte bStack0000000e;
  
  cVar3 = '\0';
  for (bVar2 = bStack00000004 - bStack00000002; bVar2 <= bStack00000004; bVar2 = bVar2 + 1) {
    FUN_0801cb60(cVar3 + cStack00000000,uStack00000001,
                 *(undefined4 *)(in_stack_00000008 + (uint)bVar2 * 4),uStack0000000c,bStack0000000e)
    ;
    cVar3 = cVar3 + '\x11';
  }
  uVar1 = (uint)bStack0000000e;
  func_0x0801ce0c(bStack00000005 * '\x11' + cStack00000000,uStack00000001,
                  *(undefined4 *)
                   (in_stack_00000008 + (_bStack00000004 & 0xff) * 4 +
                   ((uint)bStack00000002 - (uint)bStack00000005) * -4),uStack0000000d);
                    /* WARNING: Could not recover jumptable at 0x08014de6. Too many branches */
                    /* WARNING: Treating indirect jump as call */
  (*UNRECOVERED_JUMPTABLE)(uVar1);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08014f60 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(uint param_1)

{
  undefined1 *puVar1;
  int iVar2;
  uint uVar3;
  int iVar4;
  char cVar5;
  byte bVar6;
  int iVar7;
  undefined4 uStack_4c;
  undefined4 uStack_48;
  undefined4 uStack_44;
  char cStack_2c;
  uint uStack_28;
  
  iVar7 = _DAT_08015c40;
  uVar3 = param_1 & 0xff;
  uStack_28 = param_1;
  if (uVar3 == 10) {
    *(undefined1 *)(_DAT_08015c40 + 1) = 0;
    cVar5 = *(char *)(_DAT_08015c3c + 5) + (*_DAT_08015c74 - *(char *)(_DAT_08015c3c + 2));
    bVar6 = cVar5 + 1;
    if ((((bVar6 == 0x2c) && (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\0')) ||
        ((bVar6 == 3 && (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\x01')))) ||
       ((bVar6 == 0xb && (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\0')))) {
      recovery_fn();
    }
    else {
      if ((bVar6 == 0x29) || (bVar6 == 0x2a)) {
        cStack_2c = FUN_0801f28e(*(undefined1 *)
                                  (_DAT_08015c7c + (uint)(byte)_DAT_08015c78[0x40] * 0x1e + 3));
        if (cStack_2c == '\x01') {
          if (*_DAT_08015b30 == '\0') {
            recovery_fn();
            return;
          }
          func_0x08014ca2();
          recovery_fn(*_DAT_08015c80,0x13);
        }
        else {
          if (*_DAT_08015b34 == '\0') {
            recovery_fn();
            return;
          }
          func_0x08014ca2();
          recovery_fn(*_DAT_08015c80,0x13);
        }
      }
      else {
        func_0x08014ca2();
      }
      iVar4 = _DAT_08015e60;
      puVar1 = _DAT_08015c78;
      if (bVar6 == 1) {
        *(undefined1 *)(_DAT_08015e60 + 2) = *_DAT_08015c78;
        FUN_0801b15c(iVar4,4,0x1e);
        FUN_08014530(1);
      }
      else if (cVar5 != -1) {
        if (bVar6 == 3) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 2];
          FUN_0801b15c(iVar4,4,6);
          FUN_08014530(3);
        }
        else if (bVar6 < 3) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[1];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(2);
        }
        else if (bVar6 == 5) {
          *(byte *)(_DAT_08015e60 + 2) = *(byte *)(_DAT_08015e64 + 0xc) & 1;
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(5);
        }
        else if (bVar6 < 5) {
          *(byte *)(_DAT_08015e60 + 2) =
               (byte)(((uint)*(byte *)(_DAT_08015e64 + 0xc) << 0x1e) >> 0x1f);
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(4);
        }
        else if (bVar6 == 7) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[4];
          FUN_0801b15c(iVar4,4,9);
          FUN_08014530(7);
        }
        else if (bVar6 < 7) {
          if (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\0') {
            *(byte *)(_DAT_08015e60 + 2) =
                 *(byte *)(_DAT_08015c7c + (uint)*(byte *)(iVar7 + 0xf0) * 0x1e + 0xe) & 3;
            FUN_0801b15c(iVar4,4,2);
            FUN_08014530(6);
          }
          else {
            func_0x08014cd4();
          }
        }
        else if (bVar6 == 9) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[7];
          FUN_0801b15c(iVar4,4,3);
          FUN_08014530(9);
        }
        else if (bVar6 < 9) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 5];
          FUN_0801b15c(iVar4,4,9);
          FUN_08014530(8);
        }
        else if (bVar6 == 0xb) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[(byte)_DAT_08015c78[0x40] + 10];
          FUN_0801b15c(iVar4,4,2);
          FUN_08014530(0xb);
        }
        else if (bVar6 < 0xb) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[(byte)_DAT_08015c78[0x40] + 8];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(10);
        }
        else if (bVar6 == 0xd) {
          *(undefined1 *)(iVar7 + 1) = 0;
          func_0x0801f028(*(undefined1 *)(iVar7 + 0xf0));
          FUN_08014530(0xd);
          iVar4 = _DAT_08015e64;
          if (1 < *(byte *)(_DAT_08015e64 + 0x11)) {
            *(undefined1 *)(_DAT_08015e64 + 0x11) = 0;
            *(undefined2 *)(iVar4 + 10) = 0xffff;
          }
          func_0x08020b38(*(undefined1 *)(iVar4 + 0x11),*(undefined2 *)(iVar4 + 10),iVar4 + 0x13);
          iVar2 = _DAT_08015e60;
          *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(iVar4 + 0x13);
          FUN_0801b15c(iVar2,4,0x33);
          iVar7 = iVar7 + 0x46;
          recovery_fn(iVar7,0x14,0);
          if (*(char *)(iVar2 + 2) == '3') {
            FUN_080235d0(iVar7,PTR_s_DEFINE__3d__dHZ_08015e74,*(ushort *)(iVar4 + 10) / 10,
                         (uint)*(ushort *)(iVar4 + 10) % 10);
          }
          else {
            FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
          }
        }
        else if (bVar6 < 0xd) {
          *(undefined1 *)(iVar7 + 1) = 0;
          func_0x0801f028(*(undefined1 *)(iVar7 + 0xf0));
          FUN_08014530(0xc);
          iVar4 = _DAT_08015e64;
          if (*(byte *)(_DAT_08015e64 + 0x11) < 2) {
            *(undefined1 *)(_DAT_08015e64 + 0x11) = 0;
            *(undefined2 *)(iVar4 + 10) = 0xffff;
          }
          func_0x08020b38(*(undefined1 *)(iVar4 + 0x11),*(undefined2 *)(iVar4 + 10),iVar4 + 0x13);
          iVar2 = _DAT_08015e60;
          *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(iVar4 + 0x13);
          FUN_0801b15c(iVar2,4,0xd3);
          iVar7 = iVar7 + 0x46;
          recovery_fn(iVar7,0x14,0);
          if (*(char *)(iVar4 + 0x11) == '\x02') {
            if (*(char *)(iVar2 + 2) == -0x2d) {
              FUN_080235d0(iVar7,PTR_s_DEFINE_D_03oN_08015e68,*(short *)(iVar4 + 10) + -0x2800);
            }
            else {
              FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
            }
          }
          else if (*(char *)(iVar4 + 0x11) == '\x03') {
            if (*(char *)(iVar2 + 2) == -0x2d) {
              FUN_080235d0(iVar7,PTR_s_DEFINE_D_03oI_08015e70,*(short *)(iVar4 + 10) + 0x5800);
            }
            else {
              FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
            }
          }
          else {
            FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
          }
        }
        else if (bVar6 == 0xf) {
          *(undefined1 *)(iVar7 + 1) = 0;
          func_0x0801f028(*(undefined1 *)(iVar7 + 0xf0));
          FUN_08014530(0xf);
          iVar4 = _DAT_08015e64;
          if (1 < *(byte *)(_DAT_08015e64 + 0x10)) {
            *(undefined1 *)(_DAT_08015e64 + 0x10) = 0;
            *(undefined2 *)(iVar4 + 8) = 0xffff;
          }
          func_0x08020b38(*(undefined1 *)(iVar4 + 0x10),*(undefined2 *)(iVar4 + 8),iVar4 + 0x12);
          iVar2 = _DAT_08015e60;
          *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(iVar4 + 0x12);
          FUN_0801b15c(iVar2,4,0x33);
          iVar7 = iVar7 + 0x46;
          recovery_fn(iVar7,0x14,0);
          if (*(char *)(iVar2 + 2) == '3') {
            FUN_080235d0(iVar7,PTR_s_DEFINE__3d__dHZ_08015e74,*(ushort *)(iVar4 + 8) / 10,
                         (uint)*(ushort *)(iVar4 + 8) % 10);
          }
          else {
            FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
          }
        }
        else if (bVar6 < 0xf) {
          *(undefined1 *)(iVar7 + 1) = 0;
          func_0x0801f028(*(undefined1 *)(iVar7 + 0xf0));
          FUN_08014530(0xe);
          iVar4 = _DAT_08015e64;
          if (*(byte *)(_DAT_08015e64 + 0x10) < 2) {
            *(undefined1 *)(_DAT_08015e64 + 0x10) = 0;
            *(undefined2 *)(iVar4 + 8) = 0xffff;
          }
          func_0x08020b38(*(undefined1 *)(iVar4 + 0x10),*(undefined2 *)(iVar4 + 8),iVar4 + 0x12);
          iVar2 = _DAT_08015e60;
          *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(iVar4 + 0x12);
          FUN_0801b15c(iVar2,4,0xd3);
          iVar7 = iVar7 + 0x46;
          recovery_fn(iVar7,0x14,0);
          if (*(char *)(iVar4 + 0x10) == '\x02') {
            if (*(char *)(iVar2 + 2) == -0x2d) {
              FUN_080235d0(iVar7,PTR_s_DEFINE_D_03oN_08015e68,*(short *)(iVar4 + 8) + -0x2800);
            }
            else {
              FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
            }
          }
          else if (*(char *)(iVar4 + 0x10) == '\x03') {
            if (*(char *)(iVar2 + 2) == -0x2d) {
              FUN_080235d0(iVar7,PTR_s_DEFINE_D_03oI_08015e70,*(short *)(iVar4 + 8) + 0x5800);
            }
            else {
              FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
            }
          }
          else {
            FUN_0801d8c0(iVar7,PTR_s_DEFINE_08015e6c,6);
          }
        }
        else if (bVar6 == 0x11) {
          if (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\0') {
            uVar3 = *(uint *)(iVar7 + (uint)*(byte *)(iVar7 + 0xf0) * 4 + 0xf1);
            FUN_080235d0(iVar7 + 2,_DAT_08016128,
                         *(uint *)(iVar7 + (uint)*(byte *)(iVar7 + 0xf0) * 4 + 0xf1) / _DAT_08016124
                         ,uVar3 - _DAT_08016124 * (uVar3 / _DAT_08016124));
            FUN_08014530(0x11);
          }
          else {
            func_0x08014cd4();
          }
        }
        else if (bVar6 < 0x11) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0xc];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x10);
        }
        else if (bVar6 == 0x13) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x17];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x13);
        }
        else if (bVar6 < 0x13) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x15];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x12);
        }
        else if (bVar6 == 0x15) {
          *(undefined1 *)(iVar7 + 0x3f) = _DAT_08015c78[0x18];
          iVar7 = _DAT_08015e60;
          *(undefined1 *)(_DAT_08015e60 + 2) = puVar1[0x18];
          FUN_0801b15c(iVar7,4,1);
          FUN_08014530(0x15);
        }
        else if (bVar6 < 0x15) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x16];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x14);
        }
        else if (bVar6 == 0x17) {
          *(byte *)(_DAT_08015e60 + 2) =
               (byte)(((uint)*(byte *)(_DAT_08015e64 + 0xd) << 0x1d) >> 0x1f);
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x17);
        }
        else if (bVar6 < 0x17) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x19];
          FUN_0801b15c(iVar4,4,9);
          FUN_08014530(0x16);
        }
        else if (bVar6 == 0x19) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1b];
          FUN_0801b15c(iVar4,4,10);
          FUN_08014530(0x19);
        }
        else if (bVar6 < 0x19) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1a];
          FUN_0801b15c(iVar4,4,0x28);
          FUN_08014530(0x18);
        }
        else if (bVar6 == 0x1b) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1d];
          FUN_0801b15c(iVar4,4,2);
          FUN_08014530(0x1b);
        }
        else if (bVar6 < 0x1b) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1c];
          FUN_0801b15c(iVar4,4,9);
          FUN_08014530(0x1a);
        }
        else if (bVar6 == 0x1d) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1f];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x1d);
        }
        else if (bVar6 < 0x1d) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x1e];
          FUN_0801b15c(iVar4,4,2);
          FUN_08014530(0x1c);
        }
        else if (bVar6 == 0x1f) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x21];
          FUN_0801b15c(iVar4,4,4);
          FUN_08014530(0x1f);
        }
        else if (bVar6 < 0x1f) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x20];
          FUN_0801b15c(iVar4,4,3);
          FUN_08014530(0x1e);
        }
        else if (bVar6 == 0x21) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x23];
          FUN_0801b15c(iVar4,4,3);
          FUN_08014530(0x21);
        }
        else if (bVar6 < 0x21) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x22];
          FUN_0801b15c(iVar4,4,4);
          FUN_08014530(0x20);
        }
        else if (bVar6 == 0x23) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x25];
          FUN_0801b15c(iVar4,4,5);
          FUN_08014530(0x23);
        }
        else if (bVar6 < 0x23) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x24];
          FUN_0801b15c(iVar4,4,3);
          FUN_08014530(0x22);
        }
        else if (bVar6 == 0x25) {
          func_0x0801f028(*(undefined1 *)(iVar7 + 0xf0));
          iVar4 = FUN_0801fd76(0);
          if (iVar4 == 0) {
            *(undefined2 *)(iVar7 + 0x40) = 1;
          }
          else {
            *(ushort *)(iVar7 + 0x40) = (ushort)(byte)_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 0x41];
          }
          FUN_08014530(0x25);
        }
        else if (bVar6 < 0x25) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x26];
          FUN_0801b15c(iVar4,4,5);
          FUN_08014530(0x24);
        }
        else if (bVar6 == 0x27) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x27];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x27);
        }
        else if (bVar6 < 0x27) {
          *(undefined1 *)(iVar7 + 0x3f) = _DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 0x41];
          FUN_08014530(0x26);
        }
        else if (bVar6 == 0x29) {
          *_DAT_080162c8 = 0;
          *_DAT_080162cc = 0;
          *(undefined1 *)(iVar7 + 0x76) = 0xb;
          iVar4 = _DAT_08015c7c;
          *(undefined1 *)((uint)*(byte *)(iVar7 + 0xf0) * 0x1e + _DAT_08015c7c + 0x10) = 1;
          *(undefined2 *)(iVar7 + 0x106) = 1;
          FUN_08021bb8(*(undefined1 *)(iVar7 + 0x76));
          FUN_0801d8c0(&uStack_4c,iVar4 + (uint)*(byte *)(iVar7 + 0xf0) * 0x1e,0x1e);
          FUN_0801ab36(*(undefined1 *)(iVar7 + 0xf0),uStack_4c,uStack_48,uStack_44);
          *(byte *)(_DAT_08015b00 + 7) = *(byte *)(_DAT_08015b00 + 7) | 1;
          FUN_08014530(0x29);
        }
        else if (bVar6 < 0x29) {
          FUN_08014530(0x28);
        }
        else if (bVar6 == 0x2b) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x28];
          FUN_0801b15c(iVar4,4,2);
          FUN_08014530(0x2b);
        }
        else if (bVar6 < 0x2b) {
          *_DAT_080162c8 = 0;
          *_DAT_080162cc = 0;
          *(undefined1 *)(iVar7 + 0x76) = 0xb;
          iVar4 = _DAT_08015c7c;
          *(undefined1 *)((uint)*(byte *)(iVar7 + 0xf0) * 0x1e + _DAT_08015c7c + 0x10) = 2;
          *(undefined2 *)(iVar7 + 0x106) = 1;
          FUN_08021bb8(*(undefined1 *)(iVar7 + 0x76));
          FUN_0801d8c0(&uStack_4c,iVar4 + (uint)*(byte *)(iVar7 + 0xf0) * 0x1e,0x1e);
          FUN_0801ab36(*(undefined1 *)(iVar7 + 0xf0),uStack_4c,uStack_48,uStack_44);
          *(byte *)(_DAT_08015b00 + 7) = *(byte *)(_DAT_08015b00 + 7) | 1;
          FUN_08014530(0x2a);
        }
        else if (bVar6 == 0x2d) {
          *(ushort *)(iVar7 + 0x40) = (ushort)(byte)_DAT_08015c78[0x29];
          FUN_08014530(0x2d);
        }
        else if (bVar6 < 0x2d) {
          if (_DAT_08015c78[*(byte *)(iVar7 + 0xf0) + 8] == '\x01') {
            *(byte *)(_DAT_08015e60 + 2) =
                 (byte)(((uint)*(byte *)(_DAT_08015c7c + (uint)*(byte *)(iVar7 + 0xf0) * 0x1e + 0xd)
                        << 0x1e) >> 0x1f);
            FUN_0801b15c(iVar4,4,1);
            FUN_08014530(0x2c);
          }
          else {
            recovery_fn();
          }
        }
        else if (bVar6 == 0x2f) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x3b];
          FUN_0801b15c(iVar4,4,0x19);
          FUN_08014530(0x2f);
        }
        else if (bVar6 < 0x2f) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x2a];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x2e);
        }
        else if (bVar6 == 0x31) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x3d];
          FUN_0801b15c(iVar4,4,0x19);
          FUN_08014530(0x31);
        }
        else if (bVar6 < 0x31) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x3c];
          FUN_0801b15c(iVar4,4,0x19);
          FUN_08014530(0x30);
        }
        else if (bVar6 == 0x32) {
          *(undefined1 *)(_DAT_08015e60 + 2) = _DAT_08015c78[0x2b];
          FUN_0801b15c(iVar4,4,1);
          FUN_08014530(0x32);
        }
      }
    }
  }
  else if (9 < uVar3) {
    if (uVar3 == 0xc) {
      recovery_fn();
      *(char *)(_DAT_08015c40 + 0x131) = (char)*(undefined2 *)(_DAT_08015c3c + 0x12);
    }
    else if (uVar3 < 0xc) {
      recovery_fn();
      *(char *)(_DAT_08015c40 + 0x131) = (char)*(undefined2 *)(_DAT_08015c3c + 0x12);
    }
    else if (uVar3 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015b04 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x31,*(undefined4 *)(_DAT_08016550 + (uint)*_DAT_08015c80 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08015c3c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015b38 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  byte bVar1;
  int iVar2;
  int iVar3;
  
  iVar2 = _DAT_08015c40;
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_08015c40 + 1) = 0;
    *(char *)(iVar2 + 0x132) =
         *(char *)(_DAT_08015c3c + 5) + (*_DAT_08015c74 - *(char *)(_DAT_08015c3c + 2));
    func_0x08014ca2();
    iVar3 = _DAT_08015e60;
    bVar1 = *(byte *)(iVar2 + 0x132);
    if (bVar1 == 0) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(iVar2 + 0x131);
      FUN_0801b15c(iVar3,4,0x31);
      FUN_08014530(0x36);
    }
    else if (bVar1 == 2) {
      FUN_08014530(0x37);
    }
    else if (bVar1 < 2) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x3e);
      FUN_0801b15c(iVar3,4,1);
      FUN_08014530(0x66);
    }
    else if (bVar1 == 4) {
      FUN_08014530(0x39);
    }
    else if (bVar1 < 4) {
      FUN_08014530(0x38);
    }
    else if (bVar1 == 6) {
      FUN_08014530(0x3b);
    }
    else if (bVar1 < 6) {
      FUN_08014530(0x3a);
    }
    else if (bVar1 == 8) {
      FUN_08014530(0x3c);
    }
    else if (bVar1 < 8) {
      FUN_08014530(0x3d);
    }
    else if (bVar1 == 10) {
      FUN_08014530(0x87);
    }
    else if (bVar1 < 10) {
      FUN_08014530(0x83);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
      *(char *)(_DAT_08015c40 + 0x132) = (char)*(undefined2 *)(_DAT_08015c3c + 0x12);
    }
    else if (param_1 < 0xc) {
      recovery_fn();
      *(char *)(_DAT_08015c40 + 0x132) = (char)*(undefined2 *)(_DAT_08015c3c + 0x12);
    }
    else if (param_1 == 0xd) {
      FUN_0801b198(0);
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015c44 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,10,*(undefined4 *)(_DAT_08016798 + (uint)*_DAT_08015c80 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015c84 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  byte bVar3;
  int iVar4;
  
  iVar2 = _DAT_080167d4;
  if (param_1 == 10) {
    iVar4 = _DAT_080167d4 + 2;
    recovery_fn(iVar4,0x14,0);
    *(undefined1 *)(iVar2 + 1) = 0;
    bVar3 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    iVar1 = _DAT_08015e60;
    if (bVar3 == 0) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x3f);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x61);
    }
    else if (bVar3 == 2) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x40);
      FUN_0801b15c(iVar1,4,0x1a);
      FUN_08014530(0x62);
    }
    else if (bVar3 < 2) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x2b);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(0x4f);
    }
    else if (bVar3 == 4) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x2d);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(100);
    }
    else if (bVar3 < 4) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x2c);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(99);
    }
    else if (bVar3 == 6) {
      FUN_0801d8c0(iVar4,_DAT_080168e0,8);
      *(undefined1 *)(iVar2 + 0x129) = 0;
      FUN_08014530(0x4d);
    }
    else if (bVar3 < 6) {
      *(undefined1 *)(_DAT_08015e60 + 2) = *(undefined1 *)(_DAT_08016864 + 0x2e);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x65);
    }
    else if (bVar3 == 8) {
      FUN_0801d8c0(iVar2 + 0x40,_DAT_08016918,2);
      FUN_0801a4dc(iVar2 + 0x40,2);
      FUN_080235d0(iVar4,_DAT_08016a60,*(undefined2 *)(iVar2 + 0x40));
      FUN_08014530(0x86);
    }
    else if (bVar3 < 8) {
      FUN_0801d8c0(iVar4,_DAT_080168e4,9);
      *(undefined1 *)(iVar2 + 0x129) = 0;
      FUN_08014530(0x4e);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015e2e (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,8,*(undefined4 *)(_DAT_0801691c + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08015e78 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  byte bVar4;
  
  iVar1 = _DAT_080167d4;
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_080167d4 + 0x12a) = 0;
    *(undefined1 *)(iVar1 + 1) = 0;
    bVar4 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    uVar3 = _DAT_08016a64;
    iVar2 = _DAT_08016a2c;
    if (bVar4 == 0) {
      FUN_0801d8c0(iVar1 + 2,_DAT_08016a28,6);
      FUN_08014530(0x47);
    }
    else if (bVar4 == 2) {
      *(undefined1 *)(iVar1 + 2) = *(undefined1 *)(_DAT_08016864 + 0x27);
      FUN_08014530(0x4b);
    }
    else if (bVar4 < 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x23);
      FUN_0801b15c(iVar2,4,0xf);
      FUN_08014530(0x48);
    }
    else if (bVar4 == 4) {
      *(char *)(_DAT_08016a2c + 2) = *(char *)(_DAT_08016864 + 0x28) + -0x21;
      FUN_0801b15c(iVar2,4,0x5d);
      FUN_08014530(0x4c);
    }
    else if (bVar4 < 4) {
      recovery_fn(_DAT_08016a64,0x50,0);
      FUN_0801d8c0(uVar3,_DAT_08016a68,0x40);
      FUN_08014530(0x58);
    }
    else if (bVar4 == 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x25);
      FUN_0801b15c(iVar2,4,6);
      FUN_08014530(0x6c);
    }
    else if (bVar4 < 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x24);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x49);
    }
    else if (bVar4 == 8) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1fb);
      FUN_0801b15c(iVar2,4,9);
      FUN_08014530(0x5a);
    }
    else if (bVar4 < 8) {
      FUN_0801d8c0(iVar1 + 2,_DAT_08016ac4,6);
      FUN_08014530(0x59);
    }
    else if (bVar4 == 10) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x203);
      FUN_0801b15c(iVar2,4,9);
      FUN_08014530(0x5c);
    }
    else if (bVar4 < 10) {
      FUN_0801d8c0(iVar1 + 2,_DAT_08016af8,6);
      FUN_08014530(0x5b);
    }
    else if (bVar4 == 0xc) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x51);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x56);
    }
    else if (bVar4 < 0xc) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x50);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x55);
    }
    else if (bVar4 == 0xe) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x53);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x6d);
    }
    else if (bVar4 < 0xe) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x4f);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x54);
    }
    else if (bVar4 == 0xf) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x52);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x57);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080160f4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0xf,*(undefined4 *)(_DAT_08016afc + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801612c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  byte bVar3;
  
  iVar1 = _DAT_080167d4;
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_080167d4 + 0x12a) = 0;
    *(undefined1 *)(iVar1 + 1) = 0;
    bVar3 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    iVar2 = _DAT_08016a2c;
    if (bVar3 == 0) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x17);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x6e);
    }
    else if (bVar3 == 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x16);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x70);
    }
    else if (bVar3 < 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x18);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x6f);
    }
    else if (bVar3 == 4) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x19);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x71);
    }
    else if (bVar3 < 4) {
      *(undefined2 *)(iVar1 + 0x40) = *(undefined2 *)(_DAT_08016864 + 0x14);
      FUN_0801a4dc(iVar1 + 0x40,2);
      FUN_080235d0(iVar1 + 2,_DAT_08016c9c,*(undefined2 *)(iVar1 + 0x40));
      FUN_08014530(0x50);
    }
    else if (bVar3 == 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1e4);
      FUN_0801b15c(iVar2,4,1);
      FUN_08014530(0x73);
    }
    else if (bVar3 < 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1a);
      FUN_0801b15c(iVar2,4,0x3b);
      FUN_08014530(0x72);
    }
    else if (bVar3 == 7) {
      FUN_0801d8c0(_DAT_08016e78,_DAT_08016d08,0x20);
      FUN_08014530(0x74);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016296 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,5,*(undefined4 *)(_DAT_08016d0c + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080162d0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  byte bVar2;
  
  iVar1 = _DAT_080167d4;
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_080167d4 + 0x12a) = 0;
    *(undefined1 *)(iVar1 + 1) = 0;
    bVar2 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    iVar1 = _DAT_08016a2c;
    if (bVar2 == 0) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x2c);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(0x44);
    }
    else if (bVar2 == 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x37);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x6b);
    }
    else if (bVar2 < 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1eb);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(0x79);
    }
    else if (bVar2 == 4) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1e7);
      FUN_0801b15c(iVar1,4,7);
      FUN_08014530(0x75);
    }
    else if (bVar2 < 4) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1b);
      FUN_0801b15c(iVar1,4,8);
      FUN_08014530(0x51);
    }
    else if (bVar2 == 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0xcf);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x80);
    }
    else if (bVar2 < 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1e8);
      FUN_0801b15c(iVar1,4,7);
      FUN_08014530(0x76);
    }
    else if (bVar2 == 8) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x2e);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x46);
    }
    else if (bVar2 < 8) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x2d);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x45);
    }
    else if (bVar2 == 10) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x3a);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x82);
    }
    else if (bVar2 < 10) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x39);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x81);
    }
    else if (bVar2 == 0xc) {
      FUN_08014530(0x7b);
    }
    else if (bVar2 < 0xc) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016e0c + 0x38);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x7d);
    }
    else if (bVar2 == 0xe) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x13f);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(0x85);
    }
    else if (bVar2 < 0xe) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0xca);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x84);
    }
    else if (bVar2 == 0xf) {
      FUN_08014530(0x7c);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016520 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0xf,*(undefined4 *)(_DAT_08016f04 + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016554 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  byte bVar2;
  
  if (param_1 == 10) {
    bVar2 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    iVar1 = _DAT_08016a2c;
    if (bVar2 == 0) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1ea);
      FUN_0801b15c(iVar1,4,2);
      FUN_08014530(0x78);
    }
    else if (bVar2 == 2) {
      FUN_0801d8c0(_DAT_080171d8,_DAT_08016fc8,6);
      FUN_08014530(0x5d);
    }
    else if (bVar2 < 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x20b);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x5e);
    }
    else if (bVar2 == 4) {
      FUN_0801d8c0(_DAT_080171d8,_DAT_08016fcc,6);
      FUN_08014530(0x5f);
    }
    else if (bVar2 < 4) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x213);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x60);
    }
    else if (bVar2 == 6) {
      FUN_0801d8c0(_DAT_080171d8,_DAT_08017060,6);
      FUN_08014530(0x7a);
    }
    else if (bVar2 < 6) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x1e9);
      FUN_0801b15c(iVar1,4,9);
      FUN_08014530(0x77);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801667c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,6,*(undefined4 *)(_DAT_08017094 + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080166a8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  byte bVar2;
  
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_080167d4 + 1) = 0;
    bVar2 = *(char *)(_DAT_08016804 + 5) + (*_DAT_08016808 - *(char *)(_DAT_08016804 + 2));
    func_0x08014ca2();
    iVar1 = _DAT_08016a2c;
    if (bVar2 == 0) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x2a);
      FUN_0801b15c(iVar1,4,3);
      FUN_08014530(0x52);
    }
    else if (bVar2 == 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x4e);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x7e);
    }
    else if (bVar2 < 2) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0x29);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x53);
    }
    else if (bVar2 == 3) {
      *(undefined1 *)(_DAT_08016a2c + 2) = *(undefined1 *)(_DAT_08016864 + 0xb9);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x8d);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801676a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_08017098 + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801679c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08016e0c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080167d8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x1e,*(undefined4 *)(_DAT_0801720c + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08016804,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016868 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,6,*(undefined4 *)(_DAT_08017214 + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017388,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016896 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    if (*(char *)(_DAT_08016e0c + 0x2c) == '\0') {
      func_0x0801a69c(_DAT_08016e0c + 1);
      FUN_0801f6f4();
    }
    else {
      recovery_fn();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080168e8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017358 + (uint)*_DAT_08016920 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017388,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016924 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined1 uVar3;
  
  iVar1 = _DAT_08016a2c;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08016a2c,0);
    iVar2 = _DAT_0801738c;
    *(byte *)(_DAT_0801738c + 0xc) =
         *(byte *)(_DAT_0801738c + 0xc) & 0xfd | (*(byte *)(iVar1 + 2) & 1) << 1;
    iVar1 = _DAT_08017210;
    func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
    uVar3 = FUN_0801f28e(*(undefined1 *)(_DAT_0801752c + (uint)*(byte *)(iVar1 + 0xf0) * 0x1e + 3));
    FUN_08026c46(uVar3,(*(byte *)(iVar2 + 0xc) & 3) >> 1);
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080169a2 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017390 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017388,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080169d2 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_08016a2c;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08016a2c,0);
    *(byte *)(_DAT_0801738c + 0xc) =
         *(byte *)(_DAT_0801738c + 0xc) & 0xfe | *(byte *)(iVar1 + 2) & 1;
    func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016a30 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017398 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016a6c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_08016e0c;
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08016e0c + 4);
    if (*(char *)(iVar1 + 4) == '\0') {
      FUN_08023788(_DAT_0801761c,0x400);
    }
    else {
      FUN_08023780(_DAT_0801761c,0x400);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016ac8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,9,*(undefined4 *)(_DAT_08017530 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016b00 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  undefined1 uVar3;
  
  iVar2 = _DAT_08017210;
  uVar3 = FUN_0801f28e(*(undefined1 *)
                        (_DAT_0801752c + (uint)*(byte *)(_DAT_08017210 + 0xf0) * 0x1e + 3));
  iVar1 = _DAT_08016e0c;
  if (param_1 == 10) {
    func_0x0801a69c((uint)*(byte *)(iVar2 + 0xf0) + _DAT_08016e0c + 5);
    *(undefined1 *)((uint)*(byte *)(iVar2 + 0xf0) + iVar2 + 0x10d) =
         *(undefined1 *)((uint)*(byte *)(iVar2 + 0xf0) + iVar1 + 5);
    FUN_0801a91a(*(undefined1 *)(iVar2 + 0xf0),uVar3,
                 *(undefined1 *)((uint)*(byte *)(iVar2 + 0xf0) + iVar1 + 5));
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
      FUN_0801a91a(*(undefined1 *)(iVar2 + 0xf0),uVar3,
                   *(char *)(_DAT_08017388 + 5) + (*_DAT_08017624 - *(char *)(_DAT_08017388 + 2)));
    }
    else if (param_1 < 0xc) {
      recovery_fn();
      FUN_0801a91a(*(undefined1 *)(iVar2 + 0xf0),uVar3,
                   *(char *)(_DAT_08017388 + 5) + (*_DAT_08017624 - *(char *)(_DAT_08017388 + 2)));
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016bcc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,9,*(int *)(_DAT_08017628 + (uint)*_DAT_08017394 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016bfe (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08017630);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016c34 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017634 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016c64 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08017908);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016ca0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017358 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016cd0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801790c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016d10 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_08017638 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016d40 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08017910);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016d76 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x28,*(undefined4 *)(_DAT_0801763c + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016da6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08017ab4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016ddc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,10,*(undefined4 *)(_DAT_08017914 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016e10 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08017ae8);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016e46 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017358 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016e7c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_08017918;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08017918,0);
    *(byte *)(_DAT_0801738c + 0xd) =
         *(byte *)(_DAT_0801738c + 0xd) & 0xfb | (*(byte *)(iVar1 + 2) & 1) << 2;
    func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016ed4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08017358 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016f08 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  
  iVar2 = _DAT_08017918;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08017918,0);
    iVar1 = _DAT_0801738c;
    *(undefined1 *)(_DAT_0801738c + 0x12) = *(undefined1 *)(iVar2 + 2);
    if (*(char *)(iVar2 + 2) == '3') {
      *(byte *)(_DAT_08017aec + 3) = *(byte *)(_DAT_08017aec + 3) & 0x7f;
      func_0x08014ca2();
      FUN_0801cdfc();
      FUN_08014530(0x33);
    }
    else {
      if (*(char *)(iVar2 + 2) == '\0') {
        *(undefined1 *)(iVar1 + 0x10) = 0;
      }
      else {
        *(undefined1 *)(iVar1 + 0x10) = 1;
      }
      FUN_08020bfe(iVar1 + 8,*(undefined1 *)(iVar1 + 0x12),*(undefined1 *)(iVar1 + 0x10));
      func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016f96 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x33,*(undefined4 *)(_DAT_08017b6c + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08016fd0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  
  iVar2 = _DAT_08017918;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08017918,0);
    iVar1 = _DAT_0801738c;
    *(undefined1 *)(_DAT_0801738c + 0x13) = *(undefined1 *)(iVar2 + 2);
    if (*(char *)(iVar2 + 2) == '3') {
      *(byte *)(_DAT_08017aec + 3) = *(byte *)(_DAT_08017aec + 3) | 0x80;
      func_0x08014ca2();
      FUN_0801cdfc();
      FUN_08014530(0x33);
    }
    else {
      if (*(char *)(iVar2 + 2) == '\0') {
        *(undefined1 *)(iVar1 + 0x11) = 0;
      }
      else {
        *(undefined1 *)(iVar1 + 0x11) = 1;
      }
      FUN_08020bfe(iVar1 + 10,*(undefined1 *)(iVar1 + 0x13),*(undefined1 *)(iVar1 + 0x11));
      func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017064 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x33,*(undefined4 *)(_DAT_08017b6c + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801709c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  
  iVar3 = _DAT_08017918;
  uVar1 = _DAT_08017388;
  uStack_18 = param_3;
  uStack_14 = param_4;
  if (param_1 == '\n') {
    FUN_0801b074(_DAT_08017918,0);
    iVar2 = _DAT_0801738c;
    *(undefined1 *)(_DAT_0801738c + 0x12) = *(undefined1 *)(iVar3 + 2);
    if (*(char *)(iVar3 + 2) == -0x2d) {
      *(byte *)(_DAT_08017aec + 3) = *(byte *)(_DAT_08017aec + 3) & 0x7f;
      func_0x08014ca2();
      FUN_0801cdfc();
      FUN_08014530(0x34);
    }
    else {
      if (*(char *)(iVar2 + 0x12) == '\0') {
        *(undefined1 *)(iVar2 + 0x10) = 0;
      }
      else if (*(byte *)(iVar2 + 0x12) < 0x6a) {
        *(undefined1 *)(iVar2 + 0x10) = 2;
      }
      else {
        *(undefined1 *)(iVar2 + 0x10) = 3;
      }
      FUN_08020bfe(iVar2 + 8,*(undefined1 *)(iVar2 + 0x12),*(undefined1 *)(iVar2 + 0x10));
      func_0x0801f05c(*(undefined1 *)(_DAT_08017210 + 0xf0));
      func_0x08014cd4();
    }
  }
  else if (param_1 == '\v') {
    recovery_fn();
  }
  else if (param_1 == '\f') {
    recovery_fn();
  }
  else if (param_1 == '\r') {
    func_0x08014cd4();
  }
  else if (param_1 == '\x0f') {
    func_0x0801d8e0(&uStack_24,_DAT_08017388,0x14);
    recovery_fn(0,uStack_24,uStack_20,uStack_1c);
    func_0x0801d8e0(&uStack_24,uVar1,0x14);
    recovery_fn(1,uStack_24,uStack_20,uStack_1c);
    iVar3 = _DAT_0801738c;
    if (*(char *)(_DAT_0801738c + 0x10) == '\0') {
      *(undefined1 *)(_DAT_0801738c + 0x12) = 1;
      *(undefined1 *)(iVar3 + 0x10) = 2;
      iVar2 = _DAT_08017918;
      *(undefined1 *)(_DAT_08017918 + 2) = *(undefined1 *)(iVar3 + 0x12);
      FUN_0801b15c(iVar2,4,0xd3);
    }
    else if (*(char *)(_DAT_0801738c + 0x10) == '\x02') {
      *(undefined1 *)(_DAT_0801738c + 0x10) = 3;
      iVar2 = _DAT_08017918;
      FUN_0801b074(_DAT_08017918,0);
      *(undefined1 *)(iVar3 + 0x12) = *(undefined1 *)(iVar2 + 2);
      *(char *)(iVar2 + 2) = *(char *)(iVar3 + 0x12) + 'i';
      FUN_0801b15c(iVar2,4,0xd3);
    }
    else if (*(char *)(_DAT_0801738c + 0x10) == '\x03') {
      *(undefined1 *)(_DAT_0801738c + 0x12) = 0;
      *(undefined1 *)(iVar3 + 0x10) = 0;
      iVar2 = _DAT_08017918;
      *(undefined1 *)(_DAT_08017918 + 2) = *(undefined1 *)(iVar3 + 0x12);
      FUN_0801b15c(iVar2,4,0xd3);
    }
    *(undefined1 *)(_DAT_08017e10 + 2) = 1;
  }
  return CONCAT44(uStack_14,uStack_18);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080171dc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0xd3,*(undefined4 *)(_DAT_08017d50 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017218 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  
  iVar3 = _DAT_08017918;
  uVar1 = _DAT_08017388;
  uStack_18 = param_3;
  uStack_14 = param_4;
  if (param_1 == '\n') {
    FUN_0801b074(_DAT_08017918,0);
    iVar2 = _DAT_0801738c;
    *(undefined1 *)(_DAT_0801738c + 0x13) = *(undefined1 *)(iVar3 + 2);
    if (*(char *)(iVar2 + 0x13) == -0x2d) {
      *(byte *)(_DAT_08017aec + 3) = *(byte *)(_DAT_08017aec + 3) | 0x80;
      func_0x08014ca2();
      FUN_0801cdfc();
      FUN_08014530(0x34);
    }
    else {
      if (*(char *)(iVar2 + 0x13) == '\0') {
        *(undefined1 *)(iVar2 + 0x11) = 0;
      }
      else if (*(byte *)(iVar2 + 0x13) < 0x6a) {
        *(undefined1 *)(iVar2 + 0x11) = 2;
      }
      else {
        *(undefined1 *)(iVar2 + 0x11) = 3;
      }
      FUN_08020bfe(iVar2 + 10,*(undefined1 *)(iVar2 + 0x13),*(undefined1 *)(iVar2 + 0x11));
      func_0x0801f05c(*(undefined1 *)(_DAT_08017e14 + 0xf0));
      func_0x08014cd4();
    }
  }
  else if (param_1 == '\v') {
    recovery_fn();
  }
  else if (param_1 == '\f') {
    recovery_fn();
  }
  else if (param_1 == '\r') {
    func_0x08014cd4();
  }
  else if (param_1 == '\x0f') {
    func_0x0801d8e0(&uStack_24,_DAT_08017388,0x14);
    recovery_fn(0,uStack_24,uStack_20,uStack_1c);
    func_0x0801d8e0(&uStack_24,uVar1,0x14);
    recovery_fn(1,uStack_24,uStack_20,uStack_1c);
    iVar3 = _DAT_0801738c;
    if (*(char *)(_DAT_0801738c + 0x11) == '\0') {
      *(undefined1 *)(_DAT_0801738c + 0x13) = 1;
      *(undefined1 *)(iVar3 + 0x11) = 2;
      iVar2 = _DAT_08017918;
      *(undefined1 *)(_DAT_08017918 + 2) = *(undefined1 *)(iVar3 + 0x13);
      FUN_0801b15c(iVar2,4,0xd3);
    }
    else if (*(char *)(_DAT_0801738c + 0x11) == '\x02') {
      *(undefined1 *)(_DAT_0801738c + 0x11) = 3;
      iVar2 = _DAT_08017918;
      FUN_0801b074(_DAT_08017918,0);
      *(undefined1 *)(iVar3 + 0x13) = *(undefined1 *)(iVar2 + 2);
      *(char *)(iVar2 + 2) = *(char *)(iVar3 + 0x13) + 'i';
      FUN_0801b15c(iVar2,4,0xd3);
    }
    else if (*(char *)(_DAT_0801738c + 0x11) == '\x03') {
      *(undefined1 *)(_DAT_0801738c + 0x13) = 0;
      *(undefined1 *)(iVar3 + 0x11) = 0;
      iVar2 = _DAT_08017918;
      *(undefined1 *)(_DAT_08017918 + 2) = *(undefined1 *)(iVar3 + 0x13);
      FUN_0801b15c(iVar2,4,0xd3);
    }
    *(undefined1 *)(_DAT_08017e10 + 2) = 1;
  }
  return CONCAT44(uStack_14,uStack_18);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801735c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0xd3,*(undefined4 *)(_DAT_08017d50 + (uint)*_DAT_08017394 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08017618,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801739c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  ushort uVar2;
  uint uVar3;
  
  iVar1 = _DAT_08017e14;
  uVar3 = (uint)param_1;
  if (uVar3 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar3 - 1) {
    if (uVar3 == 10) {
      if ((*(byte *)(_DAT_08017aec + 4) & 0x7f) >> 6 == 0) {
        *_DAT_08017e18 = *_DAT_08017e18 + -1;
      }
      else {
        *(byte *)(_DAT_08017aec + 4) = *(byte *)(_DAT_08017aec + 4) & 0xbf;
      }
      func_0x0801f05c(*(undefined1 *)(_DAT_08017e14 + 0xf0));
      FUN_0801cdfc();
      func_0x08014cd4();
      return param_4;
    }
    if (uVar3 - 1 != 0xc) {
      return param_4;
    }
    *(byte *)(_DAT_08017aec + 4) = *(byte *)(_DAT_08017aec + 4) & 0xbf;
    FUN_0801cdfc();
    func_0x08014cd4();
    return param_4;
  }
  if (*(char *)(_DAT_08017e14 + 1) == '\0') {
    if (param_1 < 6) {
      if (2 < param_1) {
        return param_4;
      }
      *(byte *)((uint)*(byte *)(_DAT_08017e14 + 1) + _DAT_08017e14 + 2) = param_1 + 0x30;
      *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
      recovery_fn(iVar1 + 3,0x13,0);
      FUN_0801d8c0(iVar1 + 3,_DAT_08017e1c,4);
    }
    else {
      recovery_fn(_DAT_08017e14 + 2,0x14,0);
      *(undefined1 *)(iVar1 + 2) = 0x20;
      *(byte *)(iVar1 + 3) = param_1 + 0x30;
      *(undefined1 *)(iVar1 + 1) = 2;
      FUN_0801d8c0(iVar1 + 4,0x8017620,3);
    }
  }
  else {
    *(byte *)((uint)*(byte *)(_DAT_08017e14 + 1) + _DAT_08017e14 + 2) = param_1 + 0x30;
    *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
    if (*(char *)(iVar1 + 1) == '\x03') {
      *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
    }
  }
  FUN_0801cb60(0x40,0x1e,iVar1 + 2,4);
  if (*(char *)(iVar1 + 1) == '\x05') {
    *(undefined1 *)(iVar1 + 1) = 0;
    uVar2 = FUN_0801a426(iVar1 + 2,5);
    iVar1 = _DAT_08017e20;
    if ((0x26b < uVar2) && (uVar2 < 0xa29)) {
      if (*(char *)(_DAT_08017aec + 3) < '\0') {
        *(undefined1 *)(_DAT_08017e20 + 0x11) = 1;
        *(ushort *)(iVar1 + 10) = uVar2;
      }
      else {
        *(undefined1 *)(_DAT_08017e20 + 0x10) = 1;
        *(ushort *)(iVar1 + 8) = uVar2;
      }
    }
    *(undefined1 *)(_DAT_08017e10 + 2) = 1;
  }
  return 3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017534 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  undefined4 in_r3;
  undefined1 auStack_24 [12];
  undefined4 uStack_18;
  
  uStack_18 = in_r3;
  recovery_fn(auStack_24,10,0);
  if (*(char *)(_DAT_08017aec + 3) < '\0') {
    if (*(char *)(_DAT_08017e20 + 0x11) == '\x01') {
      FUN_080235d0(auStack_24,_DAT_08017e24,*(ushort *)(_DAT_08017e20 + 10) / 10,
                   (uint)*(ushort *)(_DAT_08017e20 + 10) % 10);
      FUN_0801cb60(0x40,0x1e,auStack_24,4,3);
      FUN_0801cb60(0x40,0x46,0x801762c,4,3);
    }
    else {
      FUN_0801cb60(0x40,0x1e,PTR_s______HZ_08018294,4,3);
    }
  }
  else if (*(char *)(_DAT_08017e20 + 0x10) == '\x01') {
    FUN_080235d0(auStack_24,_DAT_08017e24,*(ushort *)(_DAT_08017e20 + 8) / 10,
                 (uint)*(ushort *)(_DAT_08017e20 + 8) % 10);
    FUN_0801cb60(0x40,0x1e,auStack_24,4,3);
    FUN_0801cb60(0x40,0x46,0x801762c,4,3);
  }
  else {
    FUN_0801cb60(0x40,0x1e,PTR_s______HZ_08018294,4,3);
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017640 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  short sVar1;
  byte *pbVar2;
  int iVar3;
  uint uVar4;
  int iVar5;
  
  iVar3 = _DAT_08017e14;
  uVar4 = (uint)param_1;
  if (uVar4 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar4 - 1) {
    if (uVar4 == 10) {
      if ((_DAT_08017aec[4] & 0x7f) >> 6 == 0) {
        *_DAT_08017e18 = *_DAT_08017e18 + -1;
      }
      else {
        _DAT_08017aec[4] = _DAT_08017aec[4] & 0xbf;
      }
      func_0x0801f05c(*(undefined1 *)(_DAT_08017e14 + 0xf0));
      FUN_0801cdfc();
      func_0x08014cd4();
      return param_4;
    }
    if (uVar4 != 0xd) {
      if (uVar4 != 0x11) {
        return param_4;
      }
      iVar5 = _DAT_08017e14 + 2;
      recovery_fn(iVar5,0x14,0);
      if ((*_DAT_08017aec & 0x1f) >> 4 == 0) {
        *_DAT_08017aec = *_DAT_08017aec | 0x10;
        FUN_0801d8c0(iVar5,PTR_s_D___I_08018298,5);
        *(undefined1 *)(_DAT_08017e20 + 0x10) = 3;
      }
      else {
        *_DAT_08017aec = *_DAT_08017aec & 0xef;
        FUN_0801d8c0(iVar5,PTR_s_D___N_0801829c,5);
        *(undefined1 *)(_DAT_08017e20 + 0x10) = 2;
      }
      *(undefined1 *)(iVar3 + 1) = 0;
      FUN_0801cb60(0x40,0x1e,iVar5,4);
      return 3;
    }
    _DAT_08017aec[4] = _DAT_08017aec[4] & 0xbf;
    FUN_0801cdfc();
    func_0x08014cd4();
    return param_4;
  }
  if (*(char *)(_DAT_08017e14 + 1) == '\0') {
    iVar5 = _DAT_08017e14 + 2;
    recovery_fn(iVar5,0x14,0);
    if ((*_DAT_08017aec & 0x1f) >> 4 == 0) {
      FUN_0801d8c0(iVar5,PTR_s_D___N_0801829c,5);
    }
    else {
      FUN_0801d8c0(iVar5,PTR_s_D___I_08018298,5);
    }
    *(byte *)(iVar3 + 3) = param_1 + 0x30;
    *(undefined1 *)(iVar3 + 1) = 2;
    FUN_0801cb60(0x40,0x1e,iVar5,4);
  }
  else {
    *(byte *)((uint)*(byte *)(_DAT_08017e14 + 1) + _DAT_08017e14 + 2) = param_1 + 0x30;
    *(char *)(iVar3 + 1) = *(char *)(iVar3 + 1) + '\x01';
    iVar5 = iVar3 + 2;
    FUN_0801cb60(0x40,0x1e,iVar5,4);
    pbVar2 = _DAT_08017aec;
    if ((*_DAT_08017aec & 0x1f) >> 4 == 0) {
      if (*(char *)(iVar3 + 1) == '\x04') {
        *(undefined1 *)(iVar3 + 1) = 0;
        uVar4 = FUN_0801a3fa(iVar3 + 3,3);
        if (uVar4 < 0x200) {
          sVar1 = (short)uVar4 + 0x2800;
          recovery_fn(iVar3 + 0x46,0x14,0);
          FUN_0801d8c0(iVar3 + 0x46,iVar5,5);
          iVar3 = _DAT_08017e20;
          if ((char)pbVar2[3] < '\0') {
            *(short *)(_DAT_08017e20 + 10) = sVar1;
            *(undefined1 *)(iVar3 + 0x11) = 2;
          }
          else {
            *(short *)(_DAT_08017e20 + 8) = sVar1;
            *(undefined1 *)(iVar3 + 0x10) = 2;
          }
        }
        else {
          FUN_0801d8c0(iVar5,PTR_s_D___N_0801829c,5);
        }
        *(undefined1 *)(_DAT_08017e10 + 2) = 1;
      }
    }
    else if (*(char *)(iVar3 + 1) == '\x04') {
      *(undefined1 *)(iVar3 + 1) = 0;
      uVar4 = FUN_0801a3fa(iVar3 + 3,3);
      if (uVar4 < 0x200) {
        sVar1 = (short)uVar4 + -0x5800;
        recovery_fn(iVar3 + 0x46,0x14,0);
        FUN_0801d8c0(iVar3 + 0x46,iVar5,5);
        iVar3 = _DAT_08017e20;
        if ((char)pbVar2[3] < '\0') {
          *(short *)(_DAT_08017e20 + 10) = sVar1;
          *(undefined1 *)(iVar3 + 0x11) = 3;
        }
        else {
          *(short *)(_DAT_08017e20 + 8) = sVar1;
          *(undefined1 *)(iVar3 + 0x10) = 3;
        }
      }
      else {
        FUN_0801d8c0(iVar5,PTR_s_D___I_08018298,5);
      }
      *(undefined1 *)(_DAT_08017e10 + 2) = 1;
    }
  }
  return 3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801791c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  ushort uVar1;
  byte *pbVar2;
  undefined4 uVar3;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  uStack_24 = param_2;
  uStack_20 = param_3;
  uStack_1c = param_4;
  recovery_fn(&uStack_24,10,0,param_4,param_1);
  pbVar2 = _DAT_08017aec;
  if ((char)_DAT_08017aec[3] < '\0') {
    if (*(char *)(_DAT_08017e20 + 0x11) == '\x02') {
      uVar1 = *(ushort *)(_DAT_08017e20 + 10);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,PTR_s_D_03oN_080182a0,uVar1 - 0x2800);
      uVar3 = 3;
      FUN_0801cb60(0x40,0x1e,&uStack_24,4);
    }
    else if (*(char *)(_DAT_08017e20 + 0x11) == '\x03') {
      uVar1 = *(ushort *)(_DAT_08017e20 + 10);
      recovery_fn(&uStack_24,10,0);
      FUN_080235d0(&uStack_24,PTR_s_D_03oI_080182a4,uVar1 - 0xa800);
      uVar3 = 3;
      FUN_0801cb60(0x40,0x1e,&uStack_24,4);
    }
    else {
      uVar3 = 3;
      FUN_0801cb60(0x40,0x1e,PTR_s_D___N_0801829c,4);
      *pbVar2 = *pbVar2 & 0xef;
    }
  }
  else if (*(char *)(_DAT_08017e20 + 0x10) == '\x02') {
    uVar1 = *(ushort *)(_DAT_08017e20 + 8);
    recovery_fn(&uStack_24,10,0);
    FUN_080235d0(&uStack_24,PTR_s_D_03oN_080182a0,uVar1 - 0x2800);
    uVar3 = 3;
    FUN_0801cb60(0x40,0x1e,&uStack_24,4);
  }
  else if (*(char *)(_DAT_08017e20 + 0x10) == '\x03') {
    uVar1 = *(ushort *)(_DAT_08017e20 + 8);
    recovery_fn(&uStack_24,10,0);
    FUN_080235d0(&uStack_24,PTR_s_D_03oI_080182a4,uVar1 - 0xa800);
    uVar3 = 3;
    FUN_0801cb60(0x40,0x1e,&uStack_24,4);
  }
  else {
    uVar3 = 3;
    FUN_0801cb60(0x40,0x1e,PTR_s_D___N_0801829c,4);
    *pbVar2 = *pbVar2 & 0xef;
  }
  return CONCAT44(uStack_24,uVar3);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017a7c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018580);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017ab8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08018504 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017af0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_080186e0);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
      FUN_08024cf8(((uint)*(byte *)(_DAT_08018584 + 5) +
                    ((uint)*_DAT_08018588 - (uint)*(byte *)(_DAT_08018584 + 2)) & 0xff) * 0x10 +
                   0x5f);
    }
    else if (param_1 < 0xc) {
      recovery_fn();
      FUN_08024cf8(((uint)*(byte *)(_DAT_08018584 + 5) +
                    ((uint)*_DAT_08018588 - (uint)*(byte *)(_DAT_08018584 + 2)) & 0xff) * 0x10 +
                   0x5f);
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017b70 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,9,*(int *)(_DAT_0801858c + (uint)*_DAT_08018508 * 8) + 8);
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017ba4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801873c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017bda (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08018590 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017c0a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018778);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017c40 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080186e4 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017c70 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801877c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017ca6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,9,*(int *)(_DAT_08018740 + (uint)*_DAT_08018508 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017cd8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_08018780;
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018780 + 0x2b);
    func_0x0801b2e8(*(undefined1 *)(iVar1 + 0x2b));
    func_0x0801b738();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017d1e (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08018784 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017d54 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  
  iVar2 = _DAT_08018874;
  iVar1 = _DAT_08017e14;
  if (param_1 == 10) {
    if (*(char *)(_DAT_08018780 + (uint)*(byte *)(_DAT_08017e14 + 0xf0) + 8) == '\0') {
      FUN_0801b074(_DAT_08018874,0);
      *(byte *)(_DAT_0801893c + 0xe) =
           *(byte *)(_DAT_0801893c + 0xe) & 0xfc | *(byte *)(iVar2 + 2) & 3;
      func_0x0801f05c(*(undefined1 *)(iVar1 + 0xf0));
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      if (*(char *)(_DAT_08018780 + (uint)*(byte *)(_DAT_08017e14 + 0xf0) + 8) == '\0') {
        recovery_fn();
      }
    }
    else if (param_1 < 0xc) {
      if (*(char *)(_DAT_08018780 + (uint)*(byte *)(_DAT_08017e14 + 0xf0) + 8) == '\0') {
        recovery_fn();
      }
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017dde (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08018878 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08017e28 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  int iVar4;
  undefined1 uStack_30;
  undefined1 auStack_2f [2];
  undefined1 auStack_2d [9];
  
  iVar1 = DAT_08018940;
  uVar2 = (uint)param_1;
  if (uVar2 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar2 - 1) {
    if (uVar2 == 10) {
      FUN_0801cdfc();
      iVar1 = DAT_08018940;
      *(undefined1 *)(DAT_08018940 + 1) = 0;
      uVar2 = FUN_0801a426(iVar1 + 2,8);
      if (uVar2 < _DAT_08018944) {
        *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = uVar2;
        uVar2 = *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1);
        FUN_080235d0(iVar1 + 2,_DAT_080189ec,
                     *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) / _DAT_08018948,
                     uVar2 - _DAT_08018948 * (uVar2 / _DAT_08018948));
        FUN_0801a8da(*(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1),&uStack_30);
        FUN_0801d8c0(_DAT_08018780 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xd,&uStack_30,4);
        FUN_0801a6ea();
        FUN_0801f21a();
      }
      func_0x08014cd4();
      FUN_0801a76c(*(undefined1 *)(iVar1 + 0xf0),_DAT_0801893c);
      return;
    }
    if (uVar2 == 0xb) {
      *(undefined1 *)(DAT_08018940 + 1) = 0;
      uVar3 = _DAT_0801893c;
      FUN_0801f3bc(*(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1),
                   *(undefined2 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 2 + 0x2e),_DAT_0801893c);
      uVar3 = FUN_0801a8aa(uVar3);
      *(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = uVar3;
      *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) =
           (uint)*(ushort *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 2 + 0x2e) +
           *(int *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1);
      if (_DAT_08018944 <= *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1)) {
        *(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = 0;
      }
      uVar2 = *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1);
      FUN_080235d0(iVar1 + 2,_DAT_080189ec,
                   *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) / _DAT_08018948,
                   uVar2 - _DAT_08018948 * (uVar2 / _DAT_08018948));
      FUN_0801a8da(*(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1),&uStack_30);
      FUN_0801d8c0(_DAT_08018780 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xd,&uStack_30,4);
      FUN_0801a6ea();
      FUN_0801f21a();
      *(undefined1 *)(_DAT_080182a8 + 2) = 1;
      return;
    }
    if (uVar2 == 0xc) {
      *(undefined1 *)(DAT_08018940 + 1) = 0;
      uVar3 = _DAT_0801893c;
      FUN_0801f3bc(*(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1),
                   *(undefined2 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 2 + 0x2e),_DAT_0801893c);
      uVar3 = FUN_0801a8aa(uVar3);
      *(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = uVar3;
      *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) =
           *(int *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) -
           (uint)*(ushort *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 2 + 0x2e);
      if (_DAT_08018944 <= *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1)) {
        *(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = _DAT_08018a20;
      }
      uVar2 = *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1);
      FUN_080235d0(iVar1 + 2,_DAT_080189ec,
                   *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) / _DAT_08018948,
                   uVar2 - _DAT_08018948 * (uVar2 / _DAT_08018948));
      FUN_0801a8da(*(undefined4 *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1),&uStack_30);
      FUN_0801d8c0(_DAT_08018780 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xd,&uStack_30,4);
      FUN_0801a6ea();
      FUN_0801f21a();
      *(undefined1 *)(_DAT_080182a8 + 2) = 1;
      return;
    }
    if (uVar2 != 0xd) {
      return;
    }
    if (*(char *)(DAT_08018940 + 1) == '\0') {
      FUN_0801cdfc();
      func_0x08014cd4();
      return;
    }
    *(char *)(DAT_08018940 + 1) = *(char *)(DAT_08018940 + 1) + -1;
    if (*(char *)(iVar1 + 1) != '\x02') {
      return;
    }
    *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + -1;
    return;
  }
  *(byte *)((uint)*(byte *)(DAT_08018940 + 1) + DAT_08018940 + 2) = param_1 + 0x30;
  *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
  iVar4 = iVar1 + 2;
  FUN_0801cc0c(0x40,0x14,iVar4,4,8,3);
  if (*(char *)(iVar1 + 1) == '\x06') {
    *(undefined1 *)(iVar1 + 1) = 0;
    uStack_30 = 0;
    FUN_0801d8c0(auStack_2f,iVar4,2);
    FUN_0801d8c0(auStack_2d,iVar1 + 5,3);
    uVar3 = _DAT_0801893c;
    FUN_08023084(iVar4,_DAT_0801893c);
    uVar2 = FUN_0801a8aa(uVar3);
    if (uVar2 < _DAT_08018944) {
      *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) = uVar2;
    }
    uVar2 = *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1);
    FUN_080235d0(iVar4,_DAT_080189ec,
                 *(uint *)(iVar1 + (uint)*(byte *)(iVar1 + 0xf0) * 4 + 0xf1) / _DAT_08018948,
                 uVar2 - _DAT_08018948 * (uVar2 / _DAT_08018948));
    *(undefined1 *)(_DAT_080182a8 + 2) = 1;
  }
  if (*(char *)(iVar1 + 1) == '\x02') {
    *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018206 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  FUN_0801cc0c(0x40,0x14,_DAT_08018d20,4);
  return 0x30000000b;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018222 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018780 + (uint)*(byte *)(DAT_08018940 + 0xf0) + 10);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018264 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08018c50 + (uint)*_DAT_08018508 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018584,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080182ac (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  int iVar3;
  byte bVar4;
  uint uVar5;
  int iVar6;
  undefined1 auStack_34 [12];
  undefined4 uStack_28;
  
  iVar3 = DAT_08018940;
  iVar2 = _DAT_0801893c;
  iVar1 = _DAT_08018780;
  uVar5 = (uint)param_1;
  uStack_28 = param_4;
  if (uVar5 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar5 - 1) {
    if (uVar5 == 10) {
      if (*(char *)(DAT_08018940 + 1) == '\0') {
        if (*(char *)((uint)*(byte *)(_DAT_08018780 + 0x40) + _DAT_08018780 + 8) == '\x01') {
          recovery_fn(auStack_34,10,0);
          FUN_0801a818(*(undefined1 *)((uint)*(byte *)(iVar3 + 0xf0) + iVar1 + 0x41),auStack_34);
          func_0x0801a832(*(undefined2 *)(iVar3 + 0x40),auStack_34);
        }
        else {
          bVar4 = FUN_0801f28e(*(undefined1 *)(_DAT_0801893c + 3));
          iVar6 = FUN_0801a8aa(iVar2);
          if ((*(byte *)(iVar2 + 0xe) & 3) == 1) {
            if ((uint)(*(int *)(iVar3 + (uint)*(byte *)(iVar1 + 0x40) * 4 + 0xf1) + iVar6) <=
                *(uint *)(iVar3 + (uint)bVar4 * 4 + 0xa1)) {
              iVar6 = *(int *)(iVar3 + (uint)*(byte *)(iVar1 + 0x40) * 4 + 0xf1) + iVar6;
            }
          }
          else if (((*(byte *)(iVar2 + 0xe) & 3) == 2) &&
                  (*(uint *)(iVar3 + (uint)bVar4 * 4 + 0xa9) <=
                   (uint)(iVar6 - *(int *)(iVar3 + (uint)*(byte *)(iVar1 + 0x40) * 4 + 0xf1)))) {
            iVar6 = iVar6 - *(int *)(iVar3 + (uint)*(byte *)(iVar1 + 0x40) * 4 + 0xf1);
          }
          FUN_0801a8da(iVar6,iVar2 + 4);
          *(byte *)(iVar2 + 0xd) = *(byte *)(iVar2 + 0xd) & 0xfe;
          *(byte *)(iVar2 + 0xd) = *(byte *)(iVar2 + 0xd) & 0xfb;
          *(byte *)(iVar2 + 0xd) = *(byte *)(iVar2 + 0xd) | 2;
        }
        *(char *)((uint)*(byte *)(iVar3 + 0xf0) + iVar1 + 0x41) =
             (char)*(undefined2 *)(iVar3 + 0x40);
        func_0x0801a800(*(undefined1 *)((uint)*(byte *)(iVar3 + 0xf0) + iVar1 + 0x41),_DAT_0801893c)
        ;
        func_0x0802007c(*(undefined1 *)((uint)*(byte *)(iVar3 + 0xf0) + iVar1 + 0x41),_DAT_08018ed0)
        ;
        FUN_0801a6ea();
      }
      FUN_0801cdfc();
      func_0x08014cd4();
      return;
    }
    if (uVar5 == 0xb) {
      *(short *)(DAT_08018940 + 0x40) = *(short *)(DAT_08018940 + 0x40) + 1;
      if (0x80 < *(ushort *)(iVar3 + 0x40)) {
        *(undefined2 *)(iVar3 + 0x40) = 1;
      }
      *(undefined1 *)(_DAT_08018ecc + 2) = 1;
      return;
    }
    if (uVar5 == 0xc) {
      *(short *)(DAT_08018940 + 0x40) = *(short *)(DAT_08018940 + 0x40) + -1;
      if (*(short *)(iVar3 + 0x40) == 0) {
        *(undefined2 *)(iVar3 + 0x40) = 0x80;
      }
      *(undefined1 *)(_DAT_08018ecc + 2) = 1;
      return;
    }
    if (uVar5 != 0xd) {
      return;
    }
    FUN_0801cdfc();
    func_0x08014cd4();
    return;
  }
  if (*(char *)(DAT_08018940 + 1) == '\0') {
    FUN_0801d8c0(DAT_08018940 + 3,0x8018774,2);
  }
  *(byte *)((uint)*(byte *)(iVar3 + 1) + iVar3 + 2) = param_1 + 0x30;
  *(char *)(iVar3 + 1) = *(char *)(iVar3 + 1) + '\x01';
  FUN_0801cc0c(0x40,0x60,iVar3 + 2,4,6,3);
  if (*(char *)(iVar3 + 1) == '\x03') {
    *(undefined1 *)(iVar3 + 1) = 0;
    *(undefined1 *)(_DAT_08018ecc + 2) = 1;
    bVar4 = FUN_0801a426(iVar3 + 2,3);
    if ((bVar4 < 0x81) && (bVar4 != 0)) {
      *(ushort *)(iVar3 + 0x40) = (ushort)bVar4;
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801850c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  int iVar1;
  undefined4 in_r3;
  int iVar2;
  undefined4 uVar3;
  
  iVar1 = DAT_08018940;
  iVar2 = DAT_08018940 + 2;
  recovery_fn(iVar2,0x14,0,in_r3,in_r3);
  FUN_080235d0(iVar2,DAT_08018f48,*(undefined2 *)(iVar1 + 0x40));
  FUN_0801cb60(0x40,0x28,_DAT_08018f50,4,3);
  iVar1 = FUN_0801fd1e(*(undefined2 *)(iVar1 + 0x40),0);
  if (iVar1 == 0) {
    uVar3 = 3;
    FUN_0801cb60(0x40,0x60,iVar2,4);
  }
  else {
    uVar3 = 3;
    FUN_0801cb60(0x40,0x60,iVar2,3);
  }
  return uVar3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018594 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  int iVar2;
  
  iVar2 = DAT_08018940;
  uVar1 = _DAT_0801893c;
  if (param_1 == 10) {
    if (*(byte *)(DAT_08018940 + 0x3f) < 3) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    else {
      recovery_fn(_DAT_0801893c,0x1e,0xff);
      func_0x0801a800(*(undefined1 *)(iVar2 + 0x3f),uVar1);
      *(char *)(iVar2 + 0x3f) = *(char *)(iVar2 + 0x3f) + '\x01';
      func_0x0801fddc();
      FUN_080200b0(iVar2 + 0x3f,0xb,0x80,0);
      *(undefined1 *)(_DAT_08018780 + (uint)*(byte *)(iVar2 + 0xf0) + 0x41) =
           *(undefined1 *)(iVar2 + 0x3f);
      FUN_0801a6ea();
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      *(char *)(DAT_08018940 + 0x3f) = *(char *)(DAT_08018940 + 0x3f) + -1;
      if (0x80 < *(byte *)(iVar2 + 0x3f)) {
        *(undefined1 *)(iVar2 + 0x3f) = 0x80;
      }
      FUN_080200b0(iVar2 + 0x3f,0xc,0x80,0);
      *(undefined1 *)(_DAT_08018ecc + 2) = 1;
    }
    else if (param_1 < 0xc) {
      *(char *)(DAT_08018940 + 0x3f) = *(char *)(DAT_08018940 + 0x3f) + '\x01';
      if (0x80 < *(byte *)(iVar2 + 0x3f)) {
        *(undefined1 *)(iVar2 + 0x3f) = 1;
      }
      FUN_080200b0(iVar2 + 0x3f,param_1,0x80,0);
      *(undefined1 *)(_DAT_08018ecc + 2) = 1;
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018692 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(void)

{
  undefined4 in_r3;
  undefined1 auStack_24 [12];
  undefined4 uStack_18;
  
  uStack_18 = in_r3;
  recovery_fn(auStack_24,10,0);
  FUN_080235d0(auStack_24,DAT_08018f48,*(undefined1 *)(DAT_08018940 + 0x3f));
  FUN_0801cb60(0x40,0x28,DAT_08018f5c,4,3);
  FUN_0801cb60(0x40,0x60,auStack_24,4,3);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080186e8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_08018874;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08018874,0);
    *(byte *)(_DAT_0801893c + 0xd) =
         *(byte *)(_DAT_0801893c + 0xd) & 0xfd | (*(byte *)(iVar1 + 2) & 1) << 1;
    func_0x0801f05c(*(undefined1 *)(DAT_08018940 + 0xf0));
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018744 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08018f54 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018788 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  
  iVar1 = DAT_08018940;
  if (param_1 < 10) {
    *(byte *)(DAT_08018940 + 0x3f) = param_1;
    *(undefined1 *)(_DAT_08018f4c + 2) = 1;
  }
  else if (param_1 == 10) {
    FUN_08014530(*(undefined1 *)(DAT_08018940 + 0x134));
    iVar2 = _DAT_08018f60;
    FUN_0801d8c0(_DAT_08018f64 + (uint)*(byte *)(iVar1 + 1),
                 *(int *)(_DAT_08018f60 + (uint)*(byte *)(iVar1 + 299) * 4) +
                 (uint)*(byte *)(iVar1 + 0x3f) + -1,1);
    FUN_0801d8c0((uint)*(byte *)(iVar1 + 1) + iVar1 + 2,
                 *(int *)(iVar2 + (uint)*(byte *)(iVar1 + 299) * 4) + (uint)*(byte *)(iVar1 + 0x3f)
                 + -1,1);
    *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
  }
  else if (param_1 == 0xb) {
    *(char *)(DAT_08018940 + 0x3f) = *(char *)(DAT_08018940 + 0x3f) + -1;
    if (*(char *)(iVar1 + 0x3f) == '\0') {
      *(undefined1 *)(iVar1 + 0x3f) = 0x1a;
    }
    *(undefined1 *)(_DAT_08018f4c + 2) = 1;
  }
  else if (param_1 == 0xc) {
    *(char *)(DAT_08018940 + 0x3f) = *(char *)(DAT_08018940 + 0x3f) + '\x01';
    if (0x1a < *(byte *)(iVar1 + 0x3f)) {
      *(undefined1 *)(iVar1 + 0x3f) = 1;
    }
    *(undefined1 *)(_DAT_08018f4c + 2) = 1;
  }
  else if (param_1 == 0xd) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801887c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  undefined4 uVar4;
  
  FUN_0801cd32(0x40,2,0x14,0x20,param_3,param_4);
  iVar2 = _DAT_08018f60;
  iVar1 = DAT_08018940;
  FUN_0801cc0c(0x40,0x16,*(undefined4 *)(_DAT_08018f60 + (uint)*(byte *)(DAT_08018940 + 299) * 4),4,
               0xd,3);
  FUN_0801cc0c(0x50,0x16,*(int *)(iVar2 + (uint)*(byte *)(iVar1 + 299) * 4) + 0xd,4,0xd,3);
  FUN_0801cd32(0x40,0x7e,0x22,0x20);
  if (*(byte *)(iVar1 + 0x3f) < 0xe) {
    uVar4 = 3;
    uVar3 = 1;
    FUN_0801cebc(0x40,*(char *)(iVar1 + 0x3f) * '\b' + '\x0e',
                 *(int *)(iVar2 + (uint)*(byte *)(iVar1 + 299) * 4) + (uint)*(byte *)(iVar1 + 0x3f)
                 + -1,1);
  }
  else {
    uVar4 = 3;
    uVar3 = 1;
    FUN_0801cebc(0x50,*(char *)(iVar1 + 0x3f) * '\b' + -0x5a,
                 *(int *)(iVar2 + (uint)*(byte *)(iVar1 + 299) * 4) + (uint)*(byte *)(iVar1 + 0x3f)
                 + -1,1);
  }
  return CONCAT44(uVar4,uVar3);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801894c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  char *pcVar1;
  int iVar2;
  
  iVar2 = _DAT_08018f6c;
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018f6c + 0x15);
    pcVar1 = _DAT_08018f58;
    *_DAT_08018f58 = *(char *)(iVar2 + 0x15);
    FUN_0801cd32(0x6f,0x1e,0x78,0x10);
    if (*pcVar1 == '\x01') {
      FUN_0801cb60(0x6f,2,PTR_DAT_080190ac,4,3);
      param_3 = 3;
      FUN_0801cb60(0x6f,0x7d,PTR_DAT_080190b0,4);
    }
    else {
      FUN_0801cb60(0x6f,2,PTR_DAT_080190b4,4,3);
      param_3 = 3;
      FUN_0801cb60(0x6f,0x7d,_DAT_08019270,4);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return CONCAT44(param_4,param_3);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080189f0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_080190b8 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018a24 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined4 uStack_18;
  
  iVar2 = _DAT_08019128;
  uStack_18 = param_3;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_08019128,0);
    iVar1 = _DAT_08018f6c;
    if (*(char *)(iVar2 + 2) != *(char *)(_DAT_08018f6c + 0x18)) {
      uStack_18 = 4;
      FUN_0801d3b4(0,0,0xa0,0x80);
    }
    func_0x0801a69c(iVar1 + 0x18);
    FUN_0801da10();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return CONCAT44(param_4,uStack_18);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018a86 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0801912c + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018ab6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08019348);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018aec (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_08019130 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018b1c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801934c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018b52 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,4,*(undefined4 *)(_DAT_08019274 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018b82 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801945c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018bb8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,4,*(undefined4 *)(_DAT_08019274 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018be8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_080194b0);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018c1e (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_08019350 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018c54 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0801951c);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018c8a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_08019350 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018cba (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08019580);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018cf0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,5,*(undefined4 *)(_DAT_08019460 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018d24 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_080195b4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018d5a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,5,*(undefined4 *)(_DAT_08019460 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018d8a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08019670);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018dc0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08018f54 + (uint)*_DAT_08018f58 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08018f68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018df0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(byte param_1)

{
  if (param_1 != 10) {
    if (param_1 < 10) {
      return;
    }
    if (param_1 == 0xc) {
      recovery_fn();
      return;
    }
    if (param_1 < 0xc) {
      recovery_fn();
      return;
    }
    if (param_1 != 0xd) {
      return;
    }
  }
  FUN_0801cdfc();
  func_0x08014cd4();
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018e20 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  int iVar1;
  byte bVar2;
  undefined1 uVar3;
  undefined4 uVar4;
  undefined4 extraout_r1;
  undefined1 auStack_28 [12];
  
  iVar1 = _DAT_08019638;
  func_0x08028034(_DAT_0801963c,*(undefined4 *)(_DAT_08019638 + 0x137));
  bVar2 = FUN_0802810c();
  uVar4 = func_0x08028138((uint)bVar2 % 10);
  recovery_fn(auStack_28,10,0);
  func_0x0802815c(uVar4);
  uVar4 = extraout_r1;
  uVar3 = FUN_0802810c(*(undefined4 *)(iVar1 + 0x137));
  FUN_080235d0(auStack_28,_DAT_08019674,uVar3);
  if (*_DAT_08018f58 == '\x01') {
    FUN_0801cb60(0x20,0x1e,_DAT_08019678,4,3,uVar4);
  }
  else {
    FUN_0801cb60(0x20,0x1e,_DAT_08019784,4,3,uVar4);
  }
  FUN_0801cc0c(0x40,0x1e,auStack_28,4,6,3);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08018f70 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  if (param_1 != '\0') {
    if ((param_1 == '\x01') || (param_1 == '\n')) {
      FUN_0801cdfc();
      *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
      iVar3 = _DAT_08019908;
      iVar2 = _DAT_080196e8;
      iVar1 = _DAT_08019638;
      if (*(char *)(_DAT_08019900 + 0x28) == '\0') {
        *(undefined2 *)(_DAT_08019908 + 8) =
             *(undefined2 *)(_DAT_080196e8 + (uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + 8);
      }
      else if (*(char *)(_DAT_08019900 + 0x28) == '\x01') {
        *(undefined2 *)(_DAT_08019908 + 10) =
             *(undefined2 *)(_DAT_080196e8 + (uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + 8);
      }
      else if (*(char *)(_DAT_08019900 + 0x28) == '\x02') {
        *(undefined2 *)(_DAT_08019908 + 8) =
             *(undefined2 *)((uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + _DAT_080196e8 + 8);
        *(undefined2 *)(iVar3 + 10) =
             *(undefined2 *)((uint)*(byte *)(iVar1 + 0xf0) * 0x1e + iVar2 + 8);
      }
      func_0x0801f05c(*(undefined1 *)(_DAT_08019638 + 0xf0));
      func_0x08014cd4();
      return param_4;
    }
    if (param_1 == '\v') {
      *(undefined1 *)(_DAT_08019638 + 0x76) = 0xb;
      *(byte *)(DAT_08019904 + 9) =
           *(byte *)(DAT_08019904 + 9) & 0xdf | ~*(byte *)(DAT_08019904 + 9) & 0x20;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
      return param_4;
    }
    if (param_1 == '\f') {
      *(undefined1 *)(_DAT_08019638 + 0x76) = 0xc;
      *(byte *)(DAT_08019904 + 9) =
           *(byte *)(DAT_08019904 + 9) & 0xdf | ~*(byte *)(DAT_08019904 + 9) & 0x20;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
      return param_4;
    }
    if (param_1 != '\r') {
      *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
      return param_4;
    }
  }
  FUN_0801cdfc();
  *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
  func_0x08014cd4();
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080190bc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  if ((*(byte *)(DAT_08019904 + 7) & 1) == 0) {
    if ((*(byte *)(DAT_08019904 + 9) & 0x3f) >> 5 == 0) {
      FUN_0801cb60(0x50,10,DAT_0801990c,3,3);
      param_3 = 3;
      FUN_0801cb60(0x50,0x50,_DAT_08019d8c,4);
    }
    else {
      FUN_0801cb60(0x50,10,DAT_0801990c,4,3);
      param_3 = 3;
      FUN_0801cb60(0x50,0x50,_DAT_08019d8c,3);
    }
  }
  return CONCAT44(param_4,param_3);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019134 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  if (param_1 != '\0') {
    if ((param_1 == '\x01') || (param_1 == '\n')) {
      FUN_0801cdfc();
      *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
      iVar3 = _DAT_08019908;
      iVar2 = _DAT_080196e8;
      iVar1 = _DAT_08019638;
      if (*(char *)(_DAT_08019900 + 0x28) == '\0') {
        *(undefined2 *)(_DAT_08019908 + 8) =
             *(undefined2 *)(_DAT_080196e8 + (uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + 8);
      }
      else if (*(char *)(_DAT_08019900 + 0x28) == '\x01') {
        *(undefined2 *)(_DAT_08019908 + 10) =
             *(undefined2 *)(_DAT_080196e8 + (uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + 8);
      }
      else if (*(char *)(_DAT_08019900 + 0x28) == '\x02') {
        *(undefined2 *)(_DAT_08019908 + 8) =
             *(undefined2 *)((uint)*(byte *)(_DAT_08019638 + 0xf0) * 0x1e + _DAT_080196e8 + 8);
        *(undefined2 *)(iVar3 + 10) =
             *(undefined2 *)((uint)*(byte *)(iVar1 + 0xf0) * 0x1e + iVar2 + 8);
      }
      func_0x0801f05c(*(undefined1 *)(_DAT_08019638 + 0xf0));
      func_0x08014cd4();
      return param_4;
    }
    if (param_1 == '\v') {
      *(undefined1 *)(_DAT_08019638 + 0x76) = 0xb;
      *(byte *)(DAT_08019904 + 9) =
           *(byte *)(DAT_08019904 + 9) & 0xdf | ~*(byte *)(DAT_08019904 + 9) & 0x20;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
      return param_4;
    }
    if (param_1 == '\f') {
      *(undefined1 *)(_DAT_08019638 + 0x76) = 0xc;
      *(byte *)(DAT_08019904 + 9) =
           *(byte *)(DAT_08019904 + 9) & 0xdf | ~*(byte *)(DAT_08019904 + 9) & 0x20;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
      return param_4;
    }
    if (param_1 != '\r') {
      *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
      return param_4;
    }
  }
  FUN_0801cdfc();
  *(byte *)(DAT_08019904 + 7) = *(byte *)(DAT_08019904 + 7) & 0xfe;
  func_0x08014cd4();
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019278 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  if ((*(byte *)(DAT_08019904 + 7) & 1) == 0) {
    if ((*(byte *)(DAT_08019904 + 9) & 0x3f) >> 5 == 0) {
      FUN_0801cb60(0x50,10,DAT_0801990c,3,3);
      param_3 = 3;
      FUN_0801cb60(0x50,0x50,_DAT_08019d8c,4);
    }
    else {
      FUN_0801cb60(0x50,10,DAT_0801990c,4,3);
      param_3 = 3;
      FUN_0801cb60(0x50,0x50,_DAT_08019d8c,3);
    }
  }
  return CONCAT44(param_4,param_3);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080192e2 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08019d90);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019318 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08019d94 + (uint)*_DAT_08019d98 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08019d9c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019354 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  ushort uVar3;
  short sVar4;
  
  iVar1 = _DAT_08019638;
  if (param_1 == 10) {
    FUN_0801cdfc();
    *(char *)(_DAT_08019900 + 0x29) = (char)*(undefined2 *)(_DAT_08019638 + 0x40);
    FUN_0801a6ea();
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      sVar4 = *(short *)(_DAT_08019638 + 0x40);
      do {
        sVar4 = sVar4 + -1;
        if (sVar4 == 0) {
          uVar3 = 0x80;
          while( true ) {
            if ((uint)uVar3 < *(ushort *)(iVar1 + 0x40) + 1) {
              return param_4;
            }
            iVar2 = FUN_0801fd1e(uVar3,0);
            if (iVar2 != 0) break;
            uVar3 = uVar3 - 1;
          }
          *(ushort *)(iVar1 + 0x40) = uVar3;
          *(undefined1 *)(_DAT_08019788 + 2) = 1;
          return param_4;
        }
        iVar2 = FUN_0801fd1e(sVar4,0);
      } while (iVar2 == 0);
      *(short *)(iVar1 + 0x40) = sVar4;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
    }
    else if (param_1 < 0xc) {
      uVar3 = *(ushort *)(_DAT_08019638 + 0x40);
      do {
        uVar3 = uVar3 + 1;
        if (0x80 < uVar3) {
          uVar3 = 0;
          while( true ) {
            if (*(ushort *)(iVar1 + 0x40) + 1 < (uint)uVar3) {
              return param_4;
            }
            iVar2 = FUN_0801fd1e(uVar3,0);
            if (iVar2 != 0) break;
            uVar3 = uVar3 + 1;
          }
          *(ushort *)(iVar1 + 0x40) = uVar3;
          *(undefined1 *)(_DAT_08019788 + 2) = 1;
          return param_4;
        }
        iVar2 = FUN_0801fd1e(uVar3,0);
      } while (iVar2 == 0);
      *(ushort *)(iVar1 + 0x40) = uVar3;
      *(undefined1 *)(_DAT_08019788 + 2) = 1;
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019464 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  undefined4 in_r3;
  undefined1 auStack_2c [20];
  undefined4 uStack_18;
  
  uStack_18 = in_r3;
  recovery_fn(auStack_2c,0x14,0);
  FUN_080235d0(auStack_2c,_DAT_08019888,*(undefined2 *)(_DAT_08019638 + 0x40));
  FUN_0801cb60(0x40,0x14,_DAT_08019da0,4,3);
  FUN_0801cb60(0x40,0x4c,auStack_2c,4,3);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080194b4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08019da4);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080194ea (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_08019da8 + (uint)*_DAT_08019d98 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08019d9c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019520 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    if (*_DAT_08019788 == '/') {
      func_0x0801a69c(_DAT_08019dac);
    }
    else if (*_DAT_08019788 == '0') {
      func_0x0801a69c(_DAT_08019db0);
    }
    else if (*_DAT_08019788 == '1') {
      func_0x0801a69c(_DAT_08019db4);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019584 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,0x19,*(undefined4 *)(_DAT_08019db8 + (uint)*_DAT_08019d98 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08019d9c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080195b8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  byte bVar1;
  
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_08019638 + 1) = 0;
    bVar1 = *(char *)(_DAT_08019d9c + 5) + (*_DAT_08019dbc - *(char *)(_DAT_08019d9c + 2));
    func_0x08014ca2();
    if (bVar1 == 0) {
      FUN_08014128(0x11);
      FUN_08014530(0x3f);
    }
    else if (bVar1 == 2) {
      FUN_0801cdfc();
      FUN_08014530(0x42);
    }
    else if (bVar1 < 2) {
      FUN_08014530(0x3e);
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019640 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  uStack_10 = param_3;
  uStack_c = param_4;
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_08019dc0 + (uint)*_DAT_08019d98 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08019d9c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801967c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  if ((param_1 == '\n') || (param_1 == '\r')) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019696 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  undefined4 in_r3;
  undefined4 uVar1;
  undefined4 uVar2;
  
  if (*_DAT_08019d98 == '\x01') {
    FUN_0801cb60(0x20,10,_DAT_08019dc4,4,3,in_r3);
  }
  else {
    FUN_0801cb60(0x20,0x1e,_DAT_08019dc8,4,3,in_r3);
  }
  uVar2 = 3;
  uVar1 = 6;
  FUN_0801cc0c(0x40,0x1e,_DAT_0801a33c,4);
  return CONCAT44(uVar2,uVar1);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080196ec (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  if ((param_1 == '\n') || (param_1 == '\r')) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019706 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  undefined4 uVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  undefined1 auStack_2c [6];
  undefined1 auStack_26 [26];
  
  uVar1 = *_DAT_0801a340;
  uVar2 = *_DAT_0801a344;
  uVar3 = *_DAT_0801a348;
  if (*_DAT_08019d98 == '\x01') {
    FUN_0801cb60(0x20,8,PTR_s_Native_ID__0801a34c,4,3);
  }
  else {
    FUN_0801cb60(0x20,8,PTR_DAT_0801a350,4,3);
  }
  FUN_080235d0(auStack_2c,PTR_s__08X_08X_08X_0801a354,uVar3,uVar2,uVar1);
  FUN_0801cc0c(0x30,8,auStack_26,4,0x12,3);
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801978c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  if ((param_1 == '\n') || (param_1 == '\r')) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08019824 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  if ((param_1 == '\n') || (param_1 == '\r')) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801988c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
void recovery_fn(char param_1)

{
  if ((param_1 == '\n') || (param_1 == '\r')) {
    FUN_0801cdfc();
    func_0x08014cd4();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 080198a6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  undefined4 uVar1;
  
  FUN_080238d4();
  func_0x0802737c(0,0);
  uVar1 = _DAT_0801a374;
  FUN_08023788(_DAT_0801a374,0x2000);
  FUN_0801baa4(500);
  func_0x0802737c(0,1);
  FUN_0801baa4(1000);
  FUN_08023780(uVar1,0x2000);
  FUN_0801cdfc();
  FUN_0801cb60(0x30,0x20,_DAT_0801a378,4,3);
  do {
                    /* WARNING: Do nothing block with infinite loop */
  } while( true );
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
