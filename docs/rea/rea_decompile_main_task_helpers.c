INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08023f4c (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(short param_1)

{
  byte bVar1;
  int *piVar2;
  int iVar3;
  undefined4 uVar4;
  
  if ((*_DAT_08024034 == '\0') && (param_1 != 0)) {
    uVar4 = FUN_0802b1e0();
    iVar3 = _DAT_0802403c;
    piVar2 = _DAT_08024038;
    bVar1 = *(byte *)(*_DAT_08024038 + 0x36);
    *(byte *)(_DAT_0802403c + (uint)bVar1) =
         *(byte *)(_DAT_0802403c + (uint)bVar1) & ~*(byte *)(*_DAT_08024038 + 0x37);
    if (*(char *)(iVar3 + (uint)bVar1) == '\0') {
      *_DAT_08024040 = *_DAT_08024040 & ~*(byte *)(*piVar2 + 0x38);
    }
    *(short *)(*piVar2 + 0x30) = param_1;
    FUN_0802b1e8(uVar4);
    FUN_0802b86c();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08013e58 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined4 recovery_fn(void)

{
  int iVar1;
  int iVar2;
  char *pcVar3;
  int iVar4;
  undefined4 in_r3;
  
  iVar2 = DAT_080144ac;
  iVar1 = DAT_080144a8;
  if ((*(byte *)(DAT_080144a8 + 9) & 1) != 0) {
    iVar4 = FUN_08023610(DAT_080144ac,PTR_s__A00__080144b0,5);
    if (iVar4 == 0) {
      FUN_0801da10();
    }
    else {
      iVar4 = FUN_08023610(iVar2,PTR_s__A01__080144b4,5);
      if (iVar4 == 0) {
        FUN_0801da10();
      }
      else {
        iVar4 = FUN_08023610(iVar2,PTR_s__A08__080144b8,5);
        if (iVar4 == 0) {
          func_0x08020e78();
          pcVar3 = DAT_080144c0;
          if (*(char *)(_DAT_080144bc + 0x38) == '\x01') {
            if ((((*DAT_080144c0 == '\0') || (*DAT_080144c0 == 'j')) || (*DAT_080144c0 == 'g')) ||
               (*DAT_080144c0 == 'h')) {
              *(byte *)(iVar1 + 0xb) = *(byte *)(iVar1 + 0xb) | 4;
              func_0x0801fb5e(_DAT_080144c4);
              iVar4 = DAT_080144c8;
              *(undefined1 *)(DAT_080144c8 + 0x129) = 0;
              *(undefined1 *)(iVar4 + 0x12a) = 0;
              *(undefined1 *)(iVar4 + 1) = 0;
              *(undefined1 *)(iVar4 + 0x124) = 0;
              FUN_0801cd32(1,1,0x9e,0x10);
              if (*pcVar3 == '\0') {
                *(byte *)(iVar1 + 8) = *(byte *)(iVar1 + 8) & 0xfe;
                FUN_0801cd32(1,1,0x9e,0x10);
                *(byte *)(iVar1 + 8) = *(byte *)(iVar1 + 8) & 0xfe;
                FUN_0801cdfc();
              }
              FUN_08014530(0x6a);
              FUN_08014128(6);
              FUN_0801219c(0xff);
              *(byte *)(iVar1 + 0xc) = *(byte *)(iVar1 + 0xc) | 0x20;
              *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) & 0xfe;
            }
          }
          else {
            *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) & 0xfe;
            if (*DAT_080144c0 == 'h') {
              FUN_08014128(5);
            }
            else if (*DAT_080144c0 == 'j') {
              *(undefined1 *)(DAT_080144c8 + 0x129) = 0;
              FUN_08014128(6);
            }
          }
        }
        else {
          iVar4 = FUN_08023610(iVar2,PTR_s__A09__080144cc,5);
          if (iVar4 == 0) {
            FUN_08023610(iVar2 + 5,0x8014118,2);
          }
          else {
            iVar4 = FUN_08023610(iVar2,PTR_s__A10__080144d0,5);
            if (iVar4 == 0) {
              FUN_08023610(iVar2 + 5,0x8014118,2);
            }
            else {
              iVar4 = FUN_08023610(iVar2,0x801411c,2);
              if (iVar4 == 0) {
                *(byte *)(iVar1 + 0xb) = *(byte *)(iVar1 + 0xb) & 0x7f;
                if (*DAT_080144c0 == 'i') {
                  func_0x0801145c(0xff);
                }
                else if (*DAT_080144c0 == 'j') {
                  FUN_0801219c(0xff);
                }
              }
              else {
                iVar4 = FUN_08023610(iVar2,0x8014120,2);
                if (iVar4 == 0) {
                  *(byte *)(iVar1 + 0xb) = *(byte *)(iVar1 + 0xb) & 0xbf;
                  if (*DAT_080144c0 == 'h') {
                    FUN_080109bc(0xff);
                  }
                }
                else {
                  iVar4 = FUN_08023610(iVar2,PTR_s__D00__080144d4,5);
                  if ((iVar4 == 0) ||
                     (iVar4 = FUN_08023610(iVar2,PTR_s__F00__080144d8,5), iVar4 == 0)) {
                    if ((*DAT_080144c0 == '{') || (*DAT_080144c0 == '|')) {
                      func_0x080134b4(0xff);
                    }
                  }
                  else {
                    iVar4 = FUN_08023610(iVar2,PTR_s__A50__080144dc,5);
                    if (iVar4 == 0) {
                      if (*DAT_080144c0 == '?') {
                        func_0x080197a6(0xff);
                      }
                    }
                    else {
                      iVar4 = FUN_08023610(iVar2,PTR_s__A12__080144e0,5);
                      if (iVar4 == 0) {
                        if (*DAT_080144c0 == '@') {
                          func_0x0801983e(0xff);
                        }
                      }
                      else {
                        iVar4 = FUN_08023610(iVar2,PTR_s__F01__080144e4,5);
                        if ((iVar4 == 0) && (*DAT_080144c0 == '@')) {
                          func_0x0801983e(0xff);
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    FUN_0801d9a0(iVar2,1000,0);
    *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) & 0xfe;
    iVar2 = DAT_080144e8;
    iVar4 = FUN_08023610(DAT_080144e8,_DAT_080144ec,5);
    if (iVar4 == 0) {
      func_0x0801fb5e(iVar2 + 5);
      if ((*DAT_080144c0 == 'g') && ((*(byte *)(iVar1 + 0xc) & 0x7f) >> 6 == 0)) {
        FUN_080100e4(0xff);
      }
      *(byte *)(iVar1 + 9) = *(byte *)(iVar1 + 9) & 0xfe;
    }
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08014302 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  ushort uVar1;
  int iVar2;
  int iVar3;
  undefined4 in_r3;
  undefined1 auStack_30 [20];
  undefined4 uStack_1c;
  
  iVar3 = _DAT_080144c8;
  iVar2 = _DAT_080144a8;
  if (((*_DAT_080144c0 == 'h') || (*_DAT_080144c0 == 'i')) || (*_DAT_080144c0 == 'j')) {
    uStack_1c = in_r3;
    if (*(char *)(DAT_0801452c + 0xca) == '\0') {
      if ((*(byte *)(_DAT_080144a8 + 0xb) & 0x7f) >> 6 == 0) {
        if (*(char *)(_DAT_080144a8 + 0xb) < '\0') {
          uVar1 = *(ushort *)(_DAT_080144c8 + 0x135);
          *(ushort *)(_DAT_080144c8 + 0x135) = uVar1 + 1;
          if (0x32 < uVar1) {
            FUN_0801d9a0(_DAT_080144f4,0x32,0);
            *_DAT_080144f0 = 0;
            *(byte *)(iVar2 + 9) = *(byte *)(iVar2 + 9) & 0xfe;
            *(undefined2 *)(iVar3 + 0x135) = 0;
            FUN_080235d0(auStack_30,DAT_08014504,*(undefined1 *)(iVar3 + 0x129));
            FUN_08023eaa(auStack_30,6);
          }
        }
        else {
          *(byte *)(_DAT_080144a8 + 0xb) = *(byte *)(_DAT_080144a8 + 0xb) & 0xbf;
          *(byte *)(iVar2 + 0xb) = *(byte *)(iVar2 + 0xb) & 0x7f;
        }
      }
      else {
        uVar1 = *(ushort *)(_DAT_080144c8 + 0x135);
        *(ushort *)(_DAT_080144c8 + 0x135) = uVar1 + 1;
        if (0x32 < uVar1) {
          FUN_0801d9a0(_DAT_080144f4,0x32,0);
          *_DAT_080144f0 = 0;
          *(byte *)(iVar2 + 9) = *(byte *)(iVar2 + 9) & 0xfe;
          *(undefined2 *)(iVar3 + 0x135) = 0;
          FUN_080235d0(auStack_30,_DAT_08014500,*(byte *)(iVar3 + 0x129) / 5);
          FUN_08023eaa(auStack_30,6);
        }
      }
    }
    else if (*_DAT_080144c0 == 'h') {
      uVar1 = *(ushort *)(_DAT_080144c8 + 0x135);
      *(ushort *)(_DAT_080144c8 + 0x135) = uVar1 + 1;
      if (0x28 < uVar1) {
        FUN_0801d9a0(_DAT_080144f4,0x32,0);
        *_DAT_080144f0 = 0;
        *(byte *)(_DAT_080144a8 + 9) = *(byte *)(_DAT_080144a8 + 9) & 0xfe;
        *(undefined2 *)(iVar3 + 0x135) = 0;
        FUN_080235d0(auStack_30,_DAT_08014500,*(byte *)(iVar3 + 0x129) / 5);
        FUN_08023eaa(auStack_30,6);
      }
    }
    else {
      uVar1 = *(ushort *)(_DAT_080144c8 + 0x135);
      *(ushort *)(_DAT_080144c8 + 0x135) = uVar1 + 1;
      if (0x28 < uVar1) {
        FUN_0801d9a0(_DAT_080144f4,0x32,0);
        *_DAT_080144f0 = 0;
        *(byte *)(_DAT_080144a8 + 9) = *(byte *)(_DAT_080144a8 + 9) & 0xfe;
        *(undefined2 *)(iVar3 + 0x135) = 0;
        FUN_080235d0(auStack_30,DAT_08014504,*(undefined1 *)(iVar3 + 0x129));
        FUN_08023eaa(auStack_30,6);
      }
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08012d24 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

undefined8 recovery_fn(void)

{
  undefined4 unaff_r5;
  undefined4 unaff_r6;
  
  if (*_DAT_0801358c == 'M') {
    if ((*(byte *)(DAT_08013590 + 8) & 0x7f) >> 6 == 0) {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) | 0x40;
      if (*(byte *)(_DAT_08013594 + 0x129) < 2) {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,*(char *)(_DAT_08013594 + 0x129) * '\x10' + '2',0x8013004,4);
      }
      else {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,*(char *)(_DAT_08013594 + 0x129) * '\x10' + ':',0x8013004,4);
      }
    }
    else {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) & 0xbf;
      func_0x0800fc48(0xff);
    }
  }
  else if (*_DAT_0801358c == 'N') {
    if ((*(byte *)(DAT_08013590 + 8) & 0x7f) >> 6 == 0) {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) | 0x40;
      if (*(char *)(_DAT_08013594 + 0x129) == '\0') {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,0x32,0x801307c,4);
      }
      else if (*(char *)(_DAT_08013594 + 0x129) == '\x01') {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,0x4a,0x8013004,4);
      }
      else if (*(char *)(_DAT_08013594 + 0x129) == '\x02') {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,0x62,0x8013004,4);
      }
      else {
        unaff_r5 = 3;
        FUN_0801cb60(0x40,0x72,0x80130b8,4);
      }
    }
    else {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) & 0xbf;
      func_0x0800fe48(0xff);
    }
  }
  else if ((((*_DAT_0801358c == 'G') || (*_DAT_0801358c == 'Y')) || (*_DAT_0801358c == '[')) ||
          ((*_DAT_0801358c == ']' || (*_DAT_0801358c == '_')))) {
    if ((*(byte *)(DAT_08013590 + 8) & 0x7f) >> 6 == 0) {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) | 0x40;
      unaff_r5 = 3;
      FUN_0801cb60(0x20,*(char *)(_DAT_08013594 + 1) * '\b' + '\x02',0x80130b8,4);
    }
    else {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) & 0xbf;
      func_0x0800e8e8(0xff);
    }
  }
  else if (*_DAT_0801358c == '\x11') {
    if ((*(byte *)(DAT_08013590 + 8) & 0x7f) >> 6 == 0) {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) | 0x40;
      unaff_r6 = 3;
      unaff_r5 = 8;
      FUN_0801cc0c(0x40,*(char *)(_DAT_08013594 + 1) * '\b' + '\x14',0x80130b8,4);
    }
    else {
      *(byte *)(DAT_08013590 + 8) = *(byte *)(DAT_08013590 + 8) & 0xbf;
      unaff_r6 = 3;
      unaff_r5 = 8;
      FUN_0801cc0c(0x40,0x14,_DAT_08013598,4);
    }
  }
  return CONCAT44(unaff_r6,unaff_r5);
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0801b5a8 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

char recovery_fn(void)

{
  char cVar1;
  undefined4 uVar2;
  
  cVar1 = FUN_080289a4(0x801b665,0,_DAT_0801b98c + 0x3fc,10,10,_DAT_0801b98c,0x100,0,3);
  if (((cVar1 == '\0') &&
      (cVar1 = FUN_080289a4(0x801b705,0,_DAT_0801b990 + 0x3fc,0xd,0xd,_DAT_0801b990,0x100,0,3),
      cVar1 == '\0')) &&
     (cVar1 = FUN_080289a4(_DAT_0801b998,0,_DAT_0801b994 + 0x3fc,0xb,0xb,_DAT_0801b994,0x100,0,3),
     cVar1 == '\0')) {
    uVar2 = FUN_08028c78(1);
    *_DAT_0801b99c = uVar2;
    cVar1 = '\0';
  }
  return cVar1;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 0802d0d0 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  func_0x0802d200();
  func_0x0802bd88();
  func_0x0802b3cc();
  FUN_0802aac2(0x1fd,1);
  FUN_08023798(0x8000,1);
  func_0x0802d5d4(1);
  func_0x0802a9e0(0);
  func_0x0802d5d4(1);
  func_0x0802a9ba(0);
  func_0x0802d5e4(0);
  FUN_08023798(_DAT_0802d1cc,1);
  func_0x0802d080();
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08020cee (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
undefined4 recovery_fn(void)

{
  char *pcVar1;
  char *pcVar2;
  undefined4 in_r3;
  
  pcVar2 = DAT_08021088;
  if ((*DAT_08021668 == '\0') && (*(char *)(DAT_0802107c + 0x16) != '\0')) {
    if ((*DAT_08021088 != '\0') && ((*(byte *)(DAT_08020fa0 + 5) & 3) >> 1 != 0)) {
      FUN_08025780(0,1,0);
    }
    pcVar1 = DAT_08021080;
    if ((*DAT_08021080 != '\0') && ((*(byte *)(DAT_08020fa0 + 5) & 3) >> 1 != 0)) {
      FUN_08025780(1,0,0);
    }
    FUN_08029468(600,0x4b0);
    FUN_0801baa4(0x46);
    FUN_08029454();
    if ((*pcVar2 != '\0') && ((*(byte *)(DAT_08020fa0 + 5) & 3) >> 1 != 0)) {
      *(byte *)(DAT_08020fa0 + 5) = *(byte *)(DAT_08020fa0 + 5) & 0xfd;
      *DAT_0802108c = 0;
    }
    if ((*pcVar1 != '\0') && ((*(byte *)(DAT_08020fa0 + 5) & 7) >> 2 != 0)) {
      *(byte *)(DAT_08020fa0 + 5) = *(byte *)(DAT_08020fa0 + 5) & 0xfb;
      *DAT_08021084 = 0;
    }
  }
  return in_r3;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: BEGIN 08026318 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(byte param_1)

{
  if (*_DAT_080266c8 == 0x36) {
    if (*_DAT_080266cc == '\n') {
      func_0x080264b4(*(char *)(_DAT_080266d0 + 0x131) + '\x01');
    }
    else {
      recovery_fn();
    }
  }
  else if ((*_DAT_080266c8 == 0) || (9 < param_1)) {
    if ((DAT_08026698[9] & 0x1f) >> 3 == 0) {
      if (param_1 == 0) {
        FUN_080260a4(*_DAT_080266bc,0);
      }
      else if (param_1 == 2) {
        FUN_080260a4(*_DAT_080266bc,2);
      }
      else if (param_1 < 2) {
        FUN_080260a4(*_DAT_080266bc,1);
      }
      else if (param_1 == 4) {
        FUN_080260a4(*_DAT_080266bc,4);
      }
      else if (param_1 < 4) {
        FUN_080260a4(*_DAT_080266bc,3);
      }
      else if (param_1 == 6) {
        FUN_080260a4(*_DAT_080266bc,6);
      }
      else if (param_1 < 6) {
        FUN_080260a4(*_DAT_080266bc,5);
      }
      else if (param_1 == 8) {
        FUN_080260a4(*_DAT_080266bc,8);
      }
      else if (param_1 < 8) {
        FUN_080260a4(*_DAT_080266bc,7);
      }
      else if (param_1 == 10) {
        if (((((*_DAT_080266c8 == 0) || (*_DAT_080266c8 == 0x35)) || (*_DAT_080266c8 == 0x67)) ||
            ((*_DAT_080266c8 == 0x68 || (*_DAT_080266c8 == 0x69)))) ||
           ((0x36 < *_DAT_080266c8 && (*_DAT_080266c8 < 0x3e)))) {
          recovery_fn();
        }
        else if (*_DAT_080266c8 == 0x26) {
          if (*(byte *)(_DAT_080266d0 + 0x3f) < 3) {
            FUN_08020da6();
          }
        }
        else {
          FUN_080260a4(*_DAT_080266bc,0x31);
        }
      }
      else if (param_1 < 10) {
        FUN_080260a4(*_DAT_080266bc,9);
      }
      else if ((param_1 == 0xc) || (param_1 < 0xc)) {
        recovery_fn();
      }
      else {
        if (param_1 != 0xe) {
          if (param_1 < 0xe) {
            if ((*DAT_08026698 & 0x7f) >> 6 != 0) {
              return;
            }
            recovery_fn();
            return;
          }
          if ((param_1 != 0x10) && (0xf < param_1)) {
            if (param_1 != 0x11) {
              return;
            }
            if (*_DAT_080266c8 == 0) {
              return;
            }
            recovery_fn();
            return;
          }
        }
        recovery_fn();
      }
    }
    else {
      recovery_fn();
    }
  }
  else {
    recovery_fn();
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecompMulti.java> REA-RESULT: END (GhidraScript)  
