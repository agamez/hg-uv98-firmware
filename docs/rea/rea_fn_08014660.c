 (GhidraScript)  

/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  byte *pbVar2;
  undefined4 uStack_24;
  undefined4 uStack_20;
  undefined4 uStack_1c;
  undefined4 uStack_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  
  iVar1 = DAT_08014f48;
  uStack_18 = param_2;
  uStack_14 = param_3;
  uStack_10 = param_4;
  func_0x0801d8e0(&uStack_24,DAT_08014f48,0x14);
  func_0x0801cfd0(0,uStack_24,uStack_20,uStack_1c);
  *(char *)(iVar1 + 5) = *(char *)(iVar1 + 5) + -1;
  pbVar2 = _DAT_08014f54;
  if (*(char *)(iVar1 + 5) == -1) {
    *(byte *)(iVar1 + 4) = *_DAT_08014f54;
    if (*(byte *)(iVar1 + 2) < *(byte *)(iVar1 + 3)) {
      func_0x0801d8e0(&uStack_24,iVar1,0x14);
      func_0x0801cfd0(1,uStack_24,uStack_20,uStack_1c);
    }
    if (*pbVar2 == *(byte *)(iVar1 + 2)) {
      *(undefined1 *)(iVar1 + 0x10) = 1;
      *(byte *)(iVar1 + 5) = *(byte *)(iVar1 + 3) % 5;
      *pbVar2 = *(byte *)(iVar1 + 3);
      *(ushort *)(iVar1 + 0x12) = (ushort)*(byte *)(iVar1 + 3);
      *(undefined1 *)(DAT_08014f34 + 2) = 1;
      goto LAB_0801471c;
    }
    if (*(char *)(iVar1 + 0x10) == '\0') {
      *pbVar2 = *pbVar2 - 5;
    }
    else {
      *(undefined1 *)(iVar1 + 2) = 4;
      *(undefined1 *)(iVar1 + 0x10) = 0;
      *pbVar2 = ((*(byte *)(iVar1 + 3) / 5) * '\x05' + (*pbVar2 - *(byte *)(iVar1 + 3))) - 1;
    }
    *(undefined1 *)(iVar1 + 5) = 4;
  }
  *(ushort *)(iVar1 + 0x12) =
       (ushort)*(byte *)(iVar1 + 5) + ((ushort)*_DAT_08014f54 - (ushort)*(byte *)(iVar1 + 2));
  *(undefined1 *)(DAT_08014f34 + 2) = 1;
LAB_0801471c:
  return CONCAT44(uStack_14,uStack_18);
}

 (GhidraScript)  
