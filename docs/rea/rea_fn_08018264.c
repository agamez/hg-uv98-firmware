 (GhidraScript)  

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
