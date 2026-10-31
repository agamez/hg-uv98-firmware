 (GhidraScript)  

/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  undefined4 unaff_r7;
  
  if (*(char *)(DAT_0802107c + 0x17) == '\0') {
    if ((*(char *)(DAT_0802107c + 0x16) == '\x01') &&
       ((*(byte *)(DAT_08020fa0 + 7) & 0xf) >> 3 == 0)) {
      *(byte *)(DAT_08020fa0 + 7) = *(byte *)(DAT_08020fa0 + 7) | 8;
      FUN_08020cee();
    }
  }
  else if ((*(byte *)(DAT_08020fa0 + 7) & 0xf) >> 3 == 0) {
    *(byte *)(DAT_08020fa0 + 7) = *(byte *)(DAT_08020fa0 + 7) | 8;
    func_0x08026318(*_DAT_08021664);
  }
  return unaff_r7;
}

 (GhidraScript)  
