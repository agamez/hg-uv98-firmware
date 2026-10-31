 (GhidraScript)  

/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (param_1 == 10) {
    func_0x0801a69c(_DAT_08018780 + (uint)*(byte *)(DAT_08018940 + 0xf0) + 10);
  }
  else if (9 < param_1) {
    if (param_1 == 0xc) {
      func_0x0801471e();
    }
    else if (param_1 < 0xc) {
      func_0x08014660();
    }
    else if (param_1 == 0xd) {
      func_0x08014cd4();
    }
  }
  return;
}

 (GhidraScript)  
