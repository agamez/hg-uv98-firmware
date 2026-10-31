 (GhidraScript)  

/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(byte param_1)

{
  undefined4 uVar1;
  
  if (*(char *)(_DAT_08021b98 + 0x1e) == '\x02') {
    uVar1 = 0xff;
  }
  else {
    if (*(char *)(_DAT_08021b98 + 0x1e) == '\0') {
      if ((-1 < *(char *)(_DAT_08021b94 + 6)) && (param_1 < 0x12)) {
        return 0xff;
      }
    }
    else if ((*(char *)(_DAT_08021b98 + 0x1e) == '\x01') && (param_1 < 0x12)) {
      return 0xff;
    }
    uVar1 = 0;
  }
  return uVar1;
}

 (GhidraScript)  
