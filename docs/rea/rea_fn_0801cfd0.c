 (GhidraScript)  

/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(char param_1,undefined4 param_2,uint param_3,int param_4,undefined4 param_5)

{
  char cVar1;
  byte bVar2;
  code *UNRECOVERED_JUMPTABLE;
  char cStack_c;
  byte bStack_a;
  byte bStack_9;
  undefined4 uStack_8;
  
  bStack_a = (byte)((uint)param_2 >> 0x10);
  uStack_8 = param_3;
  if ((param_3 & 0xff) < (uint)bStack_a) {
    uStack_8 = CONCAT31((int3)(param_3 >> 8),bStack_a);
  }
  cStack_c = (char)param_2;
  if (param_1 == '\0') {
    FUN_0801cd32(uStack_8._1_1_ * '\x11' + cStack_c,1,0x9e,0x10);
  }
  else {
    bStack_9 = (byte)((uint)param_2 >> 0x18);
    if ((4 < bStack_9) || ((*_DAT_0801d838 != '\v' && (*_DAT_0801d838 != '\f')))) {
      cVar1 = '\0';
      for (bVar2 = 0; bVar2 <= bStack_a; bVar2 = bVar2 + 1) {
        FUN_0801cf5e(cVar1 + cStack_c,
                     *(undefined4 *)(param_4 + ((uStack_8 & 0xff) - (uint)bStack_a) * 4),4,
                     param_5._2_1_);
        uStack_8 = (uint)(byte)((char)uStack_8 + 1);
        cVar1 = cVar1 + '\x11';
      }
    }
  }
                    /* WARNING: Could not recover jumptable at 0x0801d06e. Too many branches */
                    /* WARNING: Treating indirect jump as call */
  (*UNRECOVERED_JUMPTABLE)();
  return;
}

 (GhidraScript)  
