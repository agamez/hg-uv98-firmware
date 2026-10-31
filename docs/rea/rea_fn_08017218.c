 (GhidraScript)  

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
    func_0x08014660();
  }
  else if (param_1 == '\f') {
    func_0x0801471e();
  }
  else if (param_1 == '\r') {
    func_0x08014cd4();
  }
  else if (param_1 == '\x0f') {
    func_0x0801d8e0(&uStack_24,_DAT_08017388,0x14);
    func_0x0801cfd0(0,uStack_24,uStack_20,uStack_1c);
    func_0x0801d8e0(&uStack_24,uVar1,0x14);
    func_0x0801cfd0(1,uStack_24,uStack_20,uStack_1c);
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
