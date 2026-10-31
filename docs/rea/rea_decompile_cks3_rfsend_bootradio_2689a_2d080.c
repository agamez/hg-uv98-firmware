INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802689a (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(undefined1 param_1,undefined1 *param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uVar1;
  short sVar2;
  bool bVar3;
  undefined1 uStack_18;
  undefined1 uStack_17;
  undefined1 uStack_16;
  undefined1 uStack_15;
  undefined4 uStack_14;
  
  uVar1 = _DAT_08027470;
  uStack_15 = (undefined1)((uint)param_3 >> 0x18);
  uStack_18 = *param_2;
  uStack_17 = (undefined1)((ushort)*(undefined2 *)(param_2 + 2) >> 8);
  uStack_16 = (undefined1)*(undefined2 *)(param_2 + 2);
  uStack_14 = param_4;
  FUN_08023788(_DAT_08027470,0x8000);
  sVar2 = 8;
  do {
    bVar3 = sVar2 != 0;
    sVar2 = sVar2 + -1;
  } while (bVar3);
  FUN_08026838(param_1);
  sVar2 = 10;
  do {
    bVar3 = sVar2 != 0;
    sVar2 = sVar2 + -1;
  } while (bVar3);
  recovery_fn(&uStack_18,3);
  sVar2 = 8;
  do {
    bVar3 = sVar2 != 0;
    sVar2 = sVar2 + -1;
  } while (bVar3);
  FUN_08026838(4);
  sVar2 = 10;
  do {
    bVar3 = sVar2 != 0;
    sVar2 = sVar2 + -1;
  } while (bVar3);
  FUN_08023788(uVar1,0x8000);
  FUN_08023788(_DAT_0802746c,0x200);
  sVar2 = 8;
  do {
    bVar3 = sVar2 != 0;
    sVar2 = sVar2 + -1;
  } while (bVar3);
  return CONCAT44(uStack_14,CONCAT13(uStack_15,CONCAT12(uStack_16,CONCAT11(uStack_17,uStack_18))));
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802d080 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
undefined4 recovery_fn(void)

{
  undefined4 unaff_r7;
  
  recovery_fn();
  func_0x080281a4();
  recovery_fn();
  func_0x080267aa();
  recovery_fn(1);
  recovery_fn(0);
  recovery_fn();
  func_0x0802bb88();
  recovery_fn();
  recovery_fn();
  recovery_fn();
  FUN_08029504();
  recovery_fn();
  recovery_fn();
  recovery_fn();
  func_0x0801bb22();
  func_0x0802bdf4();
  recovery_fn();
  return unaff_r7;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
