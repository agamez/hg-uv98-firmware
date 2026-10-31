INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e130 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  char cVar1;
  int iVar2;
  
  iVar2 = _DAT_0800ea34;
  if (param_1 == 10) {
    cVar1 = *_DAT_0800e8e4;
    if (cVar1 == 'E') {
      func_0x0801a69c(_DAT_0800ea10);
    }
    else if (cVar1 == 'F') {
      func_0x0801a69c(_DAT_0800ea14);
    }
    else if (cVar1 == 'I') {
      func_0x0801a6ba(_DAT_0800ea18);
    }
    else if (cVar1 == 'T') {
      func_0x0801a6ba(_DAT_0800ea1c);
    }
    else if (cVar1 == 'U') {
      func_0x0801a6ba(_DAT_0800ea20);
    }
    else if (cVar1 == 'V') {
      func_0x0801a6ba(_DAT_0800ea24);
    }
    else if (cVar1 == 'W') {
      func_0x0801a6ba(_DAT_0800ea28);
    }
    else if (cVar1 == '^') {
      func_0x0801a6ba(_DAT_0800ea2c);
    }
    else if (cVar1 == '`') {
      func_0x0801a6ba(_DAT_0800ea30);
    }
    else if (cVar1 == 'a') {
      func_0x0801a6ba(_DAT_0800ea34 + 0x3f);
      if (*(char *)(iVar2 + 0x3f) == '\0') {
        FUN_0801cd32(1,0x5a,0x10,0x10);
      }
    }
    else if (cVar1 == 'm') {
      func_0x0801a6ba(_DAT_0800ea38);
    }
    else if (cVar1 == 'n') {
      func_0x0801a6ba(_DAT_0800ea3c);
    }
    else if (cVar1 == 'p') {
      func_0x0801a6ba(_DAT_0800ea40);
    }
    else if (cVar1 == 'q') {
      func_0x0801a6ba(_DAT_0800ea44);
    }
    else if (cVar1 == 's') {
      func_0x0801a6ba(_DAT_0800ea48);
    }
    else if (cVar1 == '}') {
      func_0x0801a69c(_DAT_0800ea4c);
    }
    else if (cVar1 == '~') {
      func_0x0801a6ba(_DAT_0800ea50);
    }
    else if (cVar1 == -0x7f) {
      func_0x0801a69c(_DAT_0800ea54);
    }
    else if (cVar1 == -0x7e) {
      func_0x0801a69c(_DAT_0800ea58);
    }
    else if (cVar1 == -0x7c) {
      func_0x0801a6ba(_DAT_0800ea5c);
    }
    else if (cVar1 == -0x73) {
      func_0x0801a6ba(_DAT_0800ea60);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e2a6 (GhidraScript)  
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
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800ea64 + (uint)*_DAT_0800ea68 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800ea6c,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e2d6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  iVar1 = _DAT_0800ea70;
  uVar2 = (uint)param_1;
  if (uVar2 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar2 - 1) {
    uStack_10 = param_3;
    uStack_c = param_4;
    if (uVar2 == 10) {
      FUN_0801a702();
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    else if (uVar2 == 0xb) {
      *(char *)((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2) =
           *(char *)((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2) + '\x01';
      if (0x7e < *(byte *)((uint)*(byte *)(iVar1 + 1) + iVar1 + 2)) {
        *(undefined1 *)(iVar1 + (uint)*(byte *)(iVar1 + 1) + 2) = 0x20;
      }
      *(undefined1 *)(_DAT_0800e8e4 + 2) = 1;
    }
    else if (uVar2 == 0xc) {
      *(char *)((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2) =
           *(char *)((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2) + -1;
      if (*(byte *)((uint)*(byte *)(iVar1 + 1) + iVar1 + 2) < 0x20) {
        *(undefined1 *)(iVar1 + (uint)*(byte *)(iVar1 + 1) + 2) = 0x7e;
      }
      *(undefined1 *)(_DAT_0800e8e4 + 2) = 1;
    }
    else if (uVar2 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    goto LAB_0800e396;
  }
  *(byte *)((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2) = param_1 + 0x30;
  uStack_c = 3;
  uStack_10 = 3;
  FUN_0801cc0c(0x40,0x47,iVar1 + 2,4);
LAB_0800e396:
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e398 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  if (*(char *)(_DAT_0800ea70 + 2) == -1) {
    FUN_0801cc0c(0x40,0x47,_DAT_0800ea74,4);
  }
  else {
    FUN_0801cc0c(0x40,0x47,_DAT_0800ea70 + 2,4);
  }
  return 0x300000001;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e3d4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined2 uVar3;
  uint uVar4;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  iVar2 = _DAT_0800ea70;
  uVar4 = (uint)param_1;
  if (uVar4 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar4 - 1) {
    uStack_20 = param_3;
    uStack_1c = param_4;
    if (uVar4 == 10) {
      if (*_DAT_0800e8e4 == 'P') {
        uVar3 = FUN_0801a426(_DAT_0800ea70 + 2,4);
        *(undefined2 *)(iVar2 + 0x40) = uVar3;
        iVar1 = _DAT_0800ea34;
        if ((4 < *(ushort *)(iVar2 + 0x40)) && (*(ushort *)(iVar2 + 0x40) < 0xe11)) {
          *(undefined2 *)(_DAT_0800ea34 + 0x14) = *(undefined2 *)(iVar2 + 0x40);
          FUN_0801a4dc(iVar1 + 0x14,2);
          FUN_0801a702();
        }
      }
      FUN_0801cdfc();
      func_0x08014cd4();
      if (*(char *)(_DAT_0800ea70 + 1) != '\0') {
        recovery_fn((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2,
                    0x14 - (uint)*(byte *)(_DAT_0800ea70 + 1),0);
      }
    }
    else if (uVar4 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    goto LAB_0800e4bc;
  }
  if (3 < *(byte *)(_DAT_0800ea70 + 1)) {
    *(undefined1 *)(_DAT_0800ea70 + 1) = 0;
  }
  *(byte *)((uint)*(byte *)(iVar2 + 1) + iVar2 + 2) = param_1 + 0x30;
  if (*(char *)(iVar2 + 1) == '\0') {
    FUN_0801d8c0(iVar2 + 3,0x800e614,3);
  }
  *(char *)(iVar2 + 1) = *(char *)(iVar2 + 1) + '\x01';
  uStack_1c = 3;
  uStack_20 = 6;
  FUN_0801cc0c(0x40,0x3f,iVar2 + 2,4);
LAB_0800e4bc:
  return CONCAT44(uStack_1c,uStack_20);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e4c0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  undefined4 in_r3;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  if (*(char *)(_DAT_0800ea70 + 2) == -1) {
    uStack_c = 3;
    uStack_10 = 4;
    FUN_0801cc0c(0x40,0x3f,_DAT_0800ea74,4);
  }
  else {
    uStack_10 = 3;
    FUN_0801cb60(0x40,0x3f,_DAT_0800ea70 + 2,4);
    uStack_c = in_r3;
  }
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e4f8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  undefined4 uVar2;
  undefined2 uVar3;
  uint uVar4;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  
  iVar1 = _DAT_0800ea70;
  uVar4 = (uint)param_1;
  if (uVar4 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar4 - 1) {
    uStack_20 = param_3;
    uStack_1c = param_4;
    if (uVar4 == 10) {
      if (*_DAT_0800e8e4 == -0x7a) {
        uVar3 = FUN_0801a426(_DAT_0800ea70 + 2,5);
        *(undefined2 *)(iVar1 + 0x40) = uVar3;
        uVar2 = _DAT_0800ecac;
        FUN_0801d8c0(_DAT_0800ecac,iVar1 + 0x40,2);
        FUN_0801a4dc(uVar2,2);
        FUN_0801a702();
      }
      FUN_0801cdfc();
      func_0x08014cd4();
      if (*(char *)(_DAT_0800ea70 + 1) != '\0') {
        recovery_fn((uint)*(byte *)(_DAT_0800ea70 + 1) + _DAT_0800ea70 + 2,
                    0x14 - (uint)*(byte *)(_DAT_0800ea70 + 1),0);
      }
    }
    else if (uVar4 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    goto LAB_0800e5d8;
  }
  if (4 < *(byte *)(_DAT_0800ea70 + 1)) {
    *(undefined1 *)(_DAT_0800ea70 + 1) = 0;
  }
  *(byte *)((uint)*(byte *)(iVar1 + 1) + iVar1 + 2) = param_1 + 0x30;
  if (*(char *)(iVar1 + 1) == '\0') {
    FUN_0801d8c0(iVar1 + 3,_DAT_0800ee1c,4);
  }
  *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
  uStack_1c = 3;
  uStack_20 = 5;
  FUN_0801cc0c(0x40,0x3f,iVar1 + 2,4);
LAB_0800e5d8:
  return CONCAT44(uStack_1c,uStack_20);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e5dc (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  undefined4 in_r3;
  undefined4 uStack_10;
  undefined4 uStack_c;
  
  if (*(char *)(_DAT_0800ea70 + 2) == -1) {
    uStack_c = 3;
    uStack_10 = 4;
    FUN_0801cc0c(0x40,0x3f,_DAT_0800ea74,4);
  }
  else {
    uStack_10 = 3;
    FUN_0801cb60(0x40,0x3f,_DAT_0800ea70 + 2,4);
    uStack_c = in_r3;
  }
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800e618 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  char cVar1;
  int iVar2;
  undefined4 uVar3;
  undefined4 uVar4;
  undefined4 uVar5;
  undefined4 uVar6;
  undefined4 uVar7;
  undefined4 uVar8;
  
  uVar8 = _DAT_0800efa8;
  uVar7 = _DAT_0800ee98;
  uVar6 = _DAT_0800ee94;
  uVar5 = _DAT_0800ee90;
  uVar4 = _DAT_0800ee8c;
  uVar3 = _DAT_0800ee88;
  iVar2 = _DAT_0800ea70;
  if (9 < param_1) {
    if (param_1 == 10) {
      if (*(char *)(_DAT_0800ea70 + 1) == '\x06') {
        *(undefined1 *)(_DAT_0800ea70 + 1) = 0;
      }
      *(undefined1 *)((uint)*(byte *)(iVar2 + 1) + iVar2 + 2) =
           *(undefined1 *)(_DAT_0800ede8 + (uint)*(byte *)(iVar2 + 0x3f));
      if (*(char *)(iVar2 + 1) == '\0') {
        recovery_fn(iVar2 + 3,5,0x2d);
      }
      *(char *)(iVar2 + 1) = *(char *)(iVar2 + 1) + '\x01';
      if (6 < *(byte *)(iVar2 + 1)) {
        *(undefined1 *)(iVar2 + 1) = 0;
      }
      _DAT_0800e8e4[2] = '\x01';
    }
    else if (param_1 == 0xb) {
      *(char *)(_DAT_0800ea70 + 0x3f) = *(char *)(_DAT_0800ea70 + 0x3f) + -1;
      if (0x23 < *(byte *)(iVar2 + 0x3f)) {
        *(undefined1 *)(iVar2 + 0x3f) = 0x23;
      }
      _DAT_0800e8e4[2] = '\x01';
    }
    else if (param_1 == 0xc) {
      *(char *)(_DAT_0800ea70 + 0x3f) = *(char *)(_DAT_0800ea70 + 0x3f) + '\x01';
      if (0x23 < *(byte *)(iVar2 + 0x3f)) {
        *(undefined1 *)(iVar2 + 0x3f) = 0;
      }
      _DAT_0800e8e4[2] = '\x01';
    }
    else if (param_1 == 0xd) {
      if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
        FUN_0801cdfc();
        func_0x08014cd4();
      }
      else {
        *(char *)(_DAT_0800ea70 + 1) = *(char *)(_DAT_0800ea70 + 1) + -1;
        *(undefined1 *)(iVar2 + (uint)*(byte *)(iVar2 + 1) + 2) = 0x2d;
      }
      _DAT_0800e8e4[2] = '\x01';
    }
    else if (param_1 == 0xf) {
      cVar1 = *_DAT_0800e8e4;
      if (cVar1 == 'G') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800ee88,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800ee88,6,0);
          FUN_0801d8c0(uVar3,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
        FUN_0801f970();
      }
      else if (cVar1 == 'Y') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800ee8c,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800ee8c,6,0);
          FUN_0801d8c0(uVar4,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
      }
      else if (cVar1 == '[') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800ee90,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800ee90,6,0);
          FUN_0801d8c0(uVar5,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
      }
      else if (cVar1 == ']') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800ee94,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800ee94,6,0);
          FUN_0801d8c0(uVar6,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
      }
      else if (cVar1 == '_') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800ee98,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800ee98,6,0);
          FUN_0801d8c0(uVar7,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
      }
      else if (cVar1 == 'z') {
        if (*(char *)(_DAT_0800ea70 + 1) == '\0') {
          FUN_0801d8c0(_DAT_0800efa8,_DAT_0800ea70 + 2,6);
        }
        else {
          recovery_fn(_DAT_0800efa8,6,0);
          FUN_0801d8c0(uVar8,iVar2 + 2,*(undefined1 *)(iVar2 + 1));
        }
        FUN_0801a702();
      }
      func_0x08014cd4();
      *(byte *)(_DAT_0800f124 + 8) = *(byte *)(_DAT_0800f124 + 8) & 0xfe;
      FUN_0801cdfc();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ea78 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  undefined4 uStack_10;
  
  iVar1 = _DAT_0800f39c;
  uStack_10 = param_4;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_0800f39c,0);
    if ((*(char *)(iVar1 + 2) == '\0') || (*(char *)(_DAT_0800f4fc + 0x2c) != '\0')) {
      func_0x0801a69c(_DAT_0800f424);
      FUN_08012f0c();
      *(byte *)(_DAT_0800f124 + 9) = *(byte *)(_DAT_0800f124 + 9) & 0x3f;
      FUN_0801da10();
    }
    else {
      func_0x0801a69c(_DAT_0800f4fc + 0x2c);
      FUN_08012f0c();
      FUN_08014128(0);
      if (*_DAT_0800f3e8 == '\x01') {
        FUN_0801cb60(0x30,0x2c,_DAT_0800f3ec,4);
      }
      else {
        FUN_0801cb60(0x30,0x2c,_DAT_0800f3f0,4);
      }
      uStack_10 = 3;
      FUN_0801baa4(2000);
      *(byte *)(_DAT_0800f124 + 9) = *(byte *)(_DAT_0800f124 + 9) & 0x3f;
      FUN_0801da10();
      FUN_0801cdfc();
    }
    if (*(char *)(_DAT_0800f4fc + 0x2c) != '\0') {
      *(undefined1 *)(_DAT_0800f4fc + 1) = 0;
      FUN_0801f6f4();
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
  return uStack_10;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800eb58 (GhidraScript)  
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
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_0800f460 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f5c8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800eb88 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_0800f638;
  if (param_1 == 10) {
    if (*_DAT_0800f12c == 'x') {
      func_0x0801a6ba(_DAT_0800f638 + 0x1ea);
      if (*(char *)(iVar1 + 0x1ea) == '\x03') {
        FUN_0801baa4(100);
        *(undefined1 *)(iVar1 + 0x4e) = 1;
        FUN_0801a702();
        func_0x0801fb26();
      }
    }
    else if (*_DAT_0800f12c == 'y') {
      func_0x0801a6ba(_DAT_0800f638 + 0x1eb);
      FUN_0801baa4(0x32);
      if (*(char *)(iVar1 + 0x1eb) == '\x03') {
        FUN_0801baa4(100);
        *(undefined1 *)(iVar1 + 0x4e) = 1;
        FUN_0801a702();
        func_0x0801fb26();
      }
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ec1e (GhidraScript)  
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
  if (*_DAT_0800f12c == 'x') {
    func_0x08014d06(4,3,*(undefined4 *)(_DAT_0800f500 + (uint)*_DAT_0800f3e8 * 8));
  }
  else if (*_DAT_0800f12c == 'y') {
    func_0x08014d06(4,3,*(undefined4 *)(_DAT_0800f500 + (uint)*_DAT_0800f3e8 * 8));
  }
  func_0x0801d8e0(&uStack_20,_DAT_0800f5c8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ec76 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800f790);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ecb0 (GhidraScript)  
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
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_0800f5cc + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f5c8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ece0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800f794);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ed16 (GhidraScript)  
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
  func_0x08014d06(4,9,*(int *)(_DAT_0800f63c + (uint)*_DAT_0800f3e8 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_0800f5c8,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ed48 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800f798);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ed7e (GhidraScript)  
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
  func_0x08014d06(4,9,*(int *)(_DAT_0800f63c + (uint)*_DAT_0800f3e8 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800edb0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800f8b8);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800edec (GhidraScript)  
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
  func_0x08014d06(4,7,*(undefined4 *)(_DAT_0800f79c + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ee20 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800f8bc);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ee56 (GhidraScript)  
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
  func_0x08014d06(4,7,*(undefined4 *)(_DAT_0800f79c + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ee9c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_0800f128;
  if (param_1 == 10) {
    if (*(char *)(_DAT_0800f128 + 0x168) == '\x01') {
      if (*(char *)(_DAT_0800f5c8 + 5) == '\0') {
        FUN_08014128(0xf);
        FUN_0801baa4(100);
      }
      *(byte *)(_DAT_0800f124 + 0xb) = *(byte *)(_DAT_0800f124 + 0xb) & 0xfd;
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      FUN_08014530(0x67);
      FUN_080100e4(0xff);
    }
    else if (*(char *)(_DAT_0800f128 + 0x168) == '\x02') {
      if (*(char *)(_DAT_0800f5c8 + 5) == '\x01') {
        FUN_08014128(0x10);
        FUN_0801baa4(100);
      }
      *(byte *)(_DAT_0800f124 + 0xb) = *(byte *)(_DAT_0800f124 + 0xb) & 0xfd;
      FUN_0801cd32(1,1,0x9e,0x10);
      FUN_0801cdfc();
      FUN_08014530(0x67);
      FUN_080100e4(0xff);
    }
    else {
      if (*(char *)(_DAT_0800f5c8 + 5) == '\0') {
        FUN_08014128(0xf);
        FUN_0801baa4(100);
      }
      func_0x08014cd4();
    }
    *(undefined1 *)(iVar1 + 0x168) = 0;
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      recovery_fn();
    }
    else if (param_1 < 0xc) {
      recovery_fn();
    }
    else if (param_1 == 0xd) {
      if (*(char *)(_DAT_0800f128 + 0x168) == '\0') {
        func_0x08014cd4();
      }
      else {
        FUN_0801cd32(1,1,0x9e,0x10);
        FUN_0801cdfc();
        FUN_08014530(0x67);
        FUN_080100e4(0xff);
      }
      *(undefined1 *)(iVar1 + 0x168) = 0;
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800efb0 (GhidraScript)  
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
  
  uStack_c = param_4;
  if (*(char *)(_DAT_0800f128 + 0x168) == '\x02') {
    uStack_10 = 3;
    FUN_0801cb60(1,2,_DAT_0800f8c0,4);
    func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800f8c4 + (uint)*_DAT_0800f3e8 * 8));
    func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
    func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  }
  else {
    uStack_10 = param_3;
    if (*(char *)(_DAT_0800f128 + 0x168) == '\x01') {
      uStack_10 = 3;
      FUN_0801cb60(1,2,*(int *)(*(int *)(_DAT_0800fa04 + (uint)*_DAT_0800f3e8 * 8) + 0x24) + 3,4);
    }
    func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800fa08 + (uint)*_DAT_0800f3e8 * 8));
    func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
    func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  }
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f050 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800fa60);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f086 (GhidraScript)  
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
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800fa64 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f0b6 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800fc40);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f0f0 (GhidraScript)  
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
  func_0x08014d06(4,9,*(int *)(_DAT_0800f63c + (uint)*_DAT_0800f3e8 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f130 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  byte bVar2;
  
  if (param_1 == 10) {
    bVar2 = *(char *)(_DAT_0800f5c8 + 5) + (*_DAT_0800fa68 - *(char *)(_DAT_0800f5c8 + 2));
    func_0x08014ca2();
    iVar1 = _DAT_0800f39c;
    if (bVar2 == 0) {
      *(undefined1 *)(_DAT_0800f39c + 2) = *(undefined1 *)(_DAT_0800f638 + 0x2f);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x88);
    }
    else if (bVar2 == 2) {
      *(undefined1 *)(_DAT_0800f39c + 2) = *(undefined1 *)(_DAT_0800f638 + 0x31);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x8a);
    }
    else if (bVar2 < 2) {
      *(undefined1 *)(_DAT_0800f39c + 2) = *(undefined1 *)(_DAT_0800f638 + 0x30);
      FUN_0801b15c(iVar1,4,1);
      FUN_08014530(0x89);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f1d8 (GhidraScript)  
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
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_0800fc44 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f20c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800fc60);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f242 (GhidraScript)  
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
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800fc64 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f270 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800fc68);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f2a6 (GhidraScript)  
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
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800fc6c + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f2d4 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800fe60);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f30a (GhidraScript)  
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
  func_0x08014d06(4,1,*(undefined4 *)(_DAT_0800fc70 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f338 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_0800fecc);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f36e (GhidraScript)  
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
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_0800fe44 + (uint)*_DAT_0800f3e8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f3a0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_0800ff38;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_0800ff38,0);
    *(undefined1 *)(_DAT_0800f638 + 0x1b) = *(undefined1 *)(iVar1 + 2);
    FUN_0801a702();
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f3f4 (GhidraScript)  
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
  func_0x08014d06(4,8,*(undefined4 *)(_DAT_0800ff74 + (uint)*_DAT_0800ffa8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f428 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800ffb0);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f464 (GhidraScript)  
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
  func_0x08014d06(4,0xf,*(int *)(_DAT_0800f63c + (uint)*_DAT_0800ffa8 * 8) + 4);
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f494 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_0800ffb4);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f4ca (GhidraScript)  
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
  func_0x08014d06(4,3,*(undefined4 *)(_DAT_0800ffac + (uint)*_DAT_0800ffa8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f504 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    *(undefined1 *)(_DAT_0800ffc0 + 0x27) = *(undefined1 *)(_DAT_0800ffb8 + 2);
    FUN_0801a702();
    func_0x08014cd4();
  }
  else if (9 < param_1) {
    if ((param_1 == 0xc) || (param_1 < 0xc)) {
      if (*(char *)(_DAT_0800ffb8 + 2) == '/') {
        *(undefined1 *)(_DAT_0800ffb8 + 2) = 0x5c;
      }
      else {
        *(undefined1 *)(_DAT_0800ffb8 + 2) = 0x2f;
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f55a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  FUN_0801cc0c(0x40,0x47,_DAT_0800ffc4,4);
  return 0x300000001;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f578 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  
  iVar1 = _DAT_0800ff38;
  if (param_1 == 10) {
    FUN_0801b074(_DAT_0800ff38,0);
    *(char *)(_DAT_0800ffc0 + 0x28) = *(char *)(iVar1 + 2) + '!';
    FUN_0801a702();
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f5d0 (GhidraScript)  
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
  if (*(char *)(_DAT_0800f638 + 0x27) == '/') {
    func_0x08014d06(4,0x5d,*(undefined4 *)(_DAT_0800ffc8 + (uint)*_DAT_0800ffa8 * 8));
    func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
    func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  }
  else {
    *(undefined1 *)(_DAT_0800f638 + 0x27) = 0x5c;
    func_0x08014d06(4,0x5d,*(undefined4 *)(_DAT_0800ffcc + (uint)*_DAT_0800ffa8 * 8));
    func_0x0801d8e0(&uStack_20,_DAT_0800f8b4,0x14);
    func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  }
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f640 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  
  uVar3 = _DAT_080100d4;
  iVar1 = _DAT_0800ffb8;
  if (9 < param_1) {
    if (param_1 == 10) {
      if (*(char *)(_DAT_0800ffb8 + 1) == '&') {
        *(undefined1 *)(_DAT_0800ffb8 + 1) = 0;
      }
      iVar2 = _DAT_080100cc;
      *(undefined1 *)(_DAT_080100cc + (uint)*(byte *)(iVar1 + 1)) =
           *(undefined1 *)(_DAT_080100d0 + (uint)*(byte *)(iVar1 + 0x3f));
      if (*(char *)(iVar1 + 1) == '\0') {
        recovery_fn(iVar2 + 1,0x4f,0x2d);
      }
      *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
      if (0x26 < *(byte *)(iVar1 + 1)) {
        *(undefined1 *)(iVar1 + 1) = 0;
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xb) {
      *(char *)(_DAT_0800ffb8 + 0x3f) = *(char *)(_DAT_0800ffb8 + 0x3f) + -1;
      if (0x23 < *(byte *)(iVar1 + 0x3f)) {
        *(undefined1 *)(iVar1 + 0x3f) = 0x23;
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xc) {
      *(char *)(_DAT_0800ffb8 + 0x3f) = *(char *)(_DAT_0800ffb8 + 0x3f) + '\x01';
      if (0x23 < *(byte *)(iVar1 + 0x3f)) {
        *(undefined1 *)(iVar1 + 0x3f) = 0;
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xd) {
      if (*(char *)(_DAT_0800ffb8 + 1) == '\0') {
        FUN_0801cdfc();
        func_0x08014cd4();
      }
      else {
        *(char *)(_DAT_0800ffb8 + 1) = *(char *)(_DAT_0800ffb8 + 1) + -1;
        *(undefined1 *)(_DAT_080100cc + (uint)*(byte *)(iVar1 + 1)) = 0x2d;
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xf) {
      if (*(char *)(_DAT_0800ffb8 + 1) == '\0') {
        FUN_0801d8c0(_DAT_080100d4,_DAT_080100cc,0x26);
      }
      else {
        recovery_fn(_DAT_080100d4,0x40,0);
        FUN_0801d8c0(uVar3,_DAT_080100cc,*(undefined1 *)(iVar1 + 1));
      }
      FUN_0801a702();
      FUN_0801f970();
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return param_4;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f7a0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  
  iVar1 = _DAT_080100d0;
  FUN_0801d8c0(_DAT_080100d0,_DAT_080100d8,0x24,param_4,param_3,param_4);
  FUN_0801cc0c(0x34,0x16,iVar1,4,10,3);
  FUN_0801cc0c(0x44,0x16,iVar1 + 10,4,0xd,3);
  FUN_0801cc0c(0x54,0x16,iVar1 + 0x17,4,0xd,3);
  if (*(byte *)(_DAT_0800ffb8 + 0x3f) < 10) {
    FUN_0801cebc(0x34,*(char *)(_DAT_0800ffb8 + 0x3f) * '\b' + '\x16',
                 iVar1 + (uint)*(byte *)(_DAT_0800ffb8 + 0x3f),1,1,3);
  }
  else if (*(byte *)(_DAT_0800ffb8 + 0x3f) < 0x17) {
    FUN_0801cebc(0x44,*(char *)(_DAT_0800ffb8 + 0x3f) * '\b' + -0x3a,
                 iVar1 + (uint)*(byte *)(_DAT_0800ffb8 + 0x3f),1,1,3);
  }
  else {
    FUN_0801cebc(0x54,*(char *)(_DAT_0800ffb8 + 0x3f) * '\b' + '^',
                 iVar1 + (uint)*(byte *)(_DAT_0800ffb8 + 0x3f),1,1,3);
  }
  iVar1 = _DAT_080100cc;
  FUN_0801cc0c(0x14,6,_DAT_080100cc,4,0x13,3);
  uVar3 = 3;
  uVar2 = 0x13;
  FUN_0801cc0c(0x24,6,iVar1 + 0x13,4);
  return CONCAT44(uVar3,uVar2);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800f8c8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  int iVar1;
  int iVar2;
  uint uVar3;
  
  iVar1 = _DAT_0800ffb8;
  uVar3 = (uint)param_1;
  if (uVar3 == 0) {
    param_1 = 0;
  }
  else if (8 < uVar3 - 1) {
    if (uVar3 == 10) {
      FUN_0801cdfc();
      func_0x08014cd4();
      if (*(char *)(_DAT_0800ffb8 + 1) != '\0') {
        recovery_fn(_DAT_080100d0 + (uint)*(byte *)(_DAT_0800ffb8 + 1),
                    0x50 - (uint)*(byte *)(_DAT_0800ffb8 + 1),0);
      }
      FUN_0801d8c0(_DAT_080100dc,_DAT_080100d0,0x20);
      FUN_0801a702();
      return;
    }
    if (uVar3 != 0xd) {
      if (uVar3 != 0xf) {
        return;
      }
      *(char *)(_DAT_0800ffb8 + 299) = *(char *)(_DAT_0800ffb8 + 299) + '\x01';
      if (2 < *(byte *)(iVar1 + 299)) {
        *(undefined1 *)(iVar1 + 299) = 0;
      }
      _DAT_0800ffbc[2] = 1;
      return;
    }
    if (*(char *)(_DAT_0800ffb8 + 1) == '\0') {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    else {
      *(undefined1 *)(_DAT_080100d0 + (uint)*(byte *)(_DAT_0800ffb8 + 1) + -1) = 0x2d;
      _DAT_0800ffbc[2] = 1;
      *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + -1;
    }
    _DAT_0800ffbc[2] = 1;
    return;
  }
  if (0x1f < *(byte *)(_DAT_0800ffb8 + 1)) {
    *(undefined1 *)(_DAT_0800ffb8 + 1) = 0;
  }
  iVar2 = _DAT_080100d0;
  *(byte *)(_DAT_080100d0 + (uint)*(byte *)(iVar1 + 1)) = param_1 + 0x30;
  if (*(char *)(iVar1 + 1) == '\0') {
    recovery_fn(iVar2 + 1,0x1f,0x2d);
  }
  if (*(char *)(iVar1 + 299) == '\0') {
    *(char *)(iVar1 + 1) = *(char *)(iVar1 + 1) + '\x01';
    _DAT_0800ffbc[2] = 1;
  }
  else {
    func_0x0800fa0c(0xff);
    func_0x08018788(param_1);
    *(undefined1 *)(iVar1 + 0x134) = *_DAT_0800ffbc;
    FUN_08014530(0x43);
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fa0c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  
  FUN_0801cb60(0x12,0x8e,*(undefined4 *)(_DAT_080100e0 + (uint)*(byte *)(_DAT_0800ffb8 + 299) * 4),4
               ,3,param_3,param_4);
  iVar1 = _DAT_080100d0;
  FUN_0801cc0c(0x18,6,_DAT_080100d0,4,0x10,3);
  uVar3 = 3;
  uVar2 = 0x10;
  FUN_0801cc0c(0x28,6,iVar1 + 0x10,4);
  return CONCAT44(uVar3,uVar2);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fa6c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  char cVar1;
  byte bVar2;
  int iVar3;
  
  if (param_1 == 10) {
    if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
      FUN_0801d8c0(_DAT_08010658,_DAT_0800ffb8 + 2,8);
      FUN_0801a702();
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    else {
      *(char *)(_DAT_0800ffb8 + 0x129) = *(char *)(_DAT_0800ffb8 + 0x129) + '\x01';
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      if (*(char *)(_DAT_0800ffb8 + 0x129) == '\0') {
        iVar3 = _DAT_0800ffb8 + 2;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 - 1;
        if (0x59 < bVar2) {
          bVar2 = 0x59;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x01') {
        iVar3 = _DAT_0800ffb8 + 4;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 - 1;
        if (0x3b < bVar2) {
          bVar2 = 0x3b;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x02') {
        iVar3 = _DAT_0800ffb8 + 7;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 - 1;
        if (99 < bVar2) {
          bVar2 = 99;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
        if (*(char *)(_DAT_0800ffb8 + 9) == 'N') {
          *(undefined1 *)(_DAT_0800ffb8 + 9) = 0x53;
        }
        else {
          *(undefined1 *)(_DAT_0800ffb8 + 9) = 0x4e;
        }
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 < 0xc) {
      if (*(char *)(_DAT_0800ffb8 + 0x129) == '\0') {
        iVar3 = _DAT_0800ffb8 + 2;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 + 1;
        if (0x59 < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x01') {
        iVar3 = _DAT_0800ffb8 + 4;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 + 1;
        if (0x3b < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x02') {
        iVar3 = _DAT_0800ffb8 + 7;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 + 1;
        if (99 < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
        if (*(char *)(_DAT_0800ffb8 + 9) == 'N') {
          *(undefined1 *)(_DAT_0800ffb8 + 9) = 0x53;
        }
        else {
          *(undefined1 *)(_DAT_0800ffb8 + 9) = 0x4e;
        }
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fc74 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  char cVar1;
  byte bVar2;
  int iVar3;
  
  iVar3 = _DAT_0800ffb8;
  if (param_1 == 10) {
    if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
      FUN_0801d8c0(_DAT_080109b0,_DAT_0800ffb8 + 2,9);
      FUN_0801a702();
      FUN_0801cdfc();
      func_0x08014cd4();
    }
    else {
      *(char *)(_DAT_0800ffb8 + 0x129) = *(char *)(_DAT_0800ffb8 + 0x129) + '\x01';
    }
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      if (*(char *)(_DAT_0800ffb8 + 0x129) == '\0') {
        iVar3 = _DAT_0800ffb8 + 2;
        cVar1 = FUN_0801a426(iVar3,3);
        bVar2 = cVar1 - 1;
        if (0xb3 < bVar2) {
          bVar2 = 0xb3;
        }
        FUN_0801a454(bVar2,iVar3,3);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x01') {
        cVar1 = FUN_0801a426(_DAT_0800ffb8 + 4,2);
        bVar2 = cVar1 - 1;
        if (0x3b < bVar2) {
          bVar2 = 0x3b;
        }
        FUN_0801a454(bVar2,iVar3 + 5,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x02') {
        iVar3 = _DAT_0800ffb8 + 8;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 - 1;
        if (99 < bVar2) {
          bVar2 = 99;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
        if (*(char *)(_DAT_0800ffb8 + 10) == 'E') {
          *(undefined1 *)(_DAT_0800ffb8 + 10) = 0x57;
        }
        else {
          *(undefined1 *)(_DAT_0800ffb8 + 10) = 0x45;
        }
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 < 0xc) {
      if (*(char *)(_DAT_0800ffb8 + 0x129) == '\0') {
        iVar3 = _DAT_0800ffb8 + 2;
        cVar1 = FUN_0801a426(iVar3,3);
        bVar2 = cVar1 + 1;
        if (0xb3 < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,3);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x01') {
        iVar3 = _DAT_0800ffb8 + 5;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 + 1;
        if (0x3b < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x02') {
        iVar3 = _DAT_0800ffb8 + 8;
        cVar1 = FUN_0801a426(iVar3,2);
        bVar2 = cVar1 + 1;
        if (99 < bVar2) {
          bVar2 = 0;
        }
        FUN_0801a454(bVar2,iVar3,2);
      }
      else if (*(char *)(_DAT_0800ffb8 + 0x129) == '\x03') {
        if (*(char *)(_DAT_0800ffb8 + 10) == 'E') {
          *(undefined1 *)(_DAT_0800ffb8 + 10) = 0x57;
        }
        else {
          *(undefined1 *)(_DAT_0800ffb8 + 10) = 0x45;
        }
      }
      *(undefined1 *)(_DAT_0800ffbc + 2) = 1;
    }
    else if (param_1 == 0xd) {
      FUN_0801cdfc();
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fe64 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_08010a64);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fe9c (GhidraScript)  
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
  func_0x08014d06(4,4,*(undefined4 *)(_DAT_080109b4 + (uint)*_DAT_0800ffa8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08010a68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800fed0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_08010a6c);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ff08 (GhidraScript)  
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
  func_0x08014d06(4,2,*(undefined4 *)(_DAT_080109b8 + (uint)*_DAT_0800ffa8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08010a68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ff3c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a6ba(_DAT_08010a70);
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
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ff78 (GhidraScript)  
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
  func_0x08014d06(4,0x1a,*(undefined4 *)(_DAT_08010a74 + (uint)*_DAT_0800ffa8 * 8));
  func_0x0801d8e0(&uStack_20,_DAT_08010a68,0x14);
  func_0x08014d5e(uStack_20,uStack_1c,uStack_18,uStack_14);
  return CONCAT44(uStack_c,uStack_10);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0800ffd0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(char param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  undefined4 uStack_20;
  
  iVar1 = DAT_08010a7c;
  uStack_20 = param_4;
  if (param_1 == '\n') {
    if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
      if (*(char *)(DAT_08010a80 + 0x2c) == '\0') {
        FUN_0801b198(0);
      }
      else {
        *(undefined1 *)(DAT_08010a84 + 0x129) = 0;
        *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) & 0xfe;
        recovery_fn(DAT_08010a88,200,0);
        recovery_fn(_DAT_08010a8c,0x49,0);
        FUN_0801cdfc();
        FUN_08014128(5);
        if (*DAT_08010a90 == '\x01') {
          FUN_0801cb60(0x30,0x2c,_DAT_08010a94,4);
        }
        else {
          FUN_0801cb60(0x30,0x2c,DAT_08010a98,4);
        }
        uStack_20 = 3;
        FUN_08014530(0x68);
        FUN_0801cd32(1,1,0x9e,0x10);
        *(undefined1 *)(_DAT_08010a9c + 2) = 0;
      }
    }
  }
  else if ((param_1 != '\v') && (param_1 != '\f')) {
    if (param_1 == '\r') {
      if ((*(byte *)(DAT_08010a7c + 0xc) & 3) >> 1 == 0) {
        FUN_0801b198(0);
      }
    }
    else if (param_1 == '\x0f') {
      recovery_fn(1);
    }
  }
  return uStack_20;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
