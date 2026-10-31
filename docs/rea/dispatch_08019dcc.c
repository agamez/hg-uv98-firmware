INFO  ReaCreateDecomp.java> REA-RESULT: BEGIN 08019dcc (GhidraScript)  
INFO  ReaCreateDecomp.java> 
/* WARNING: Globals starting with '_' overlap smaller symbols at the same address */

void recovery_fn(void)

{
  byte *pbVar1;
  int iVar2;
  byte *pbVar3;
  char *pcVar4;
  char cVar5;
  int iVar6;
  int iVar7;
  uint uVar8;
  undefined4 uStack_34;
  undefined4 uStack_30;
  undefined4 uStack_2c;
  
  pcVar4 = _DAT_0801a398;
  cVar5 = func_0x080282fc();
  *pcVar4 = cVar5;
  iVar2 = DAT_0801a380;
  pbVar1 = DAT_0801a37c;
  if (*pcVar4 == -1) {
    if (((*(int *)(DAT_0801a380 + 0x21) != 0) && (*(uint *)(DAT_0801a380 + 0x21) < 0x28)) ||
       ((*DAT_0801a37c & 0xf) >> 3 != 0)) {
      *DAT_0801a37c = *DAT_0801a37c & 0xf7;
      iVar6 = _DAT_0801a3a4;
      pcVar4 = _DAT_0801a3a0;
      pbVar3 = DAT_0801a390;
      if (((*_DAT_0801a3a0 == '\v') || (*_DAT_0801a3a0 == '\f')) ||
         ((*_DAT_0801a3a0 == '\r' || (*_DAT_0801a3a0 == '\x0f')))) {
        (**(code **)((uint)*DAT_0801a390 * 0x10 + _DAT_0801a3a4 + 4))(*_DAT_0801a3a0);
        if (pbVar3[2] != 0) {
          (**(code **)((uint)*pbVar3 * 0x10 + iVar6 + 8))(*pcVar4);
          pbVar3[2] = 0;
        }
      }
      else if (*_DAT_0801a3a0 == '\x0e') {
        if (((*pbVar1 & 0x7f) >> 6 == 0) &&
           ((**(code **)((uint)*DAT_0801a390 * 0x10 + _DAT_0801a3a4 + 4))(*_DAT_0801a3a0),
           pbVar3[2] != 0)) {
          (**(code **)((uint)*pbVar3 * 0x10 + iVar6 + 8))(*pcVar4);
          pbVar3[2] = 0;
        }
      }
      else if (*_DAT_0801a3a0 == '\x12') {
        FUN_08021238(*(undefined1 *)(DAT_0801a388 + 0x21));
      }
      else if (*_DAT_0801a3a0 == '\x13') {
        FUN_080214e4(*(undefined1 *)(DAT_0801a388 + 0x23));
      }
      else if (*_DAT_0801a3a0 == '\x14') {
        func_0x0802166c(*(undefined1 *)(DAT_0801a388 + 0x25));
      }
    }
    func_0x08027ef4();
    pbVar1 = DAT_0801a37c;
    DAT_0801a37c[0xc] = DAT_0801a37c[0xc] & 0xfb;
    pbVar1[0xc] = pbVar1[0xc] & 0xbf;
    pbVar1[7] = pbVar1[7] & 0xf7;
    *(undefined4 *)(iVar2 + 0x21) = 0;
    pbVar1[3] = pbVar1[3] & 0xf7;
    *pbVar1 = *pbVar1 & 0xf7;
    pbVar1[7] = pbVar1[7] & 0x7f;
    pbVar1[9] = pbVar1[9] & 0xfb;
    *_DAT_0801a3a0 = '\0';
    pbVar1[7] = pbVar1[7] & 0xfb;
  }
  else {
    *(undefined2 *)(DAT_0801a380 + 0x103) = 0;
    *(undefined2 *)(iVar2 + 0x25) = 0;
    *(undefined1 *)(iVar2 + 0x124) = 0;
    *(undefined2 *)(iVar2 + 0xdf) = 10;
    *(undefined1 *)(iVar2 + 0x128) = 3;
    pbVar1 = DAT_0801a37c;
    if (((DAT_0801a37c[0xc] & 7) >> 2 == 0) && ((DAT_0801a37c[8] & 0xc) != 0)) {
      DAT_0801a37c[0xb] = DAT_0801a37c[0xb] & 0xf7;
      FUN_080242b8();
      pbVar1[8] = pbVar1[8] & 0xfb;
      pbVar1[8] = pbVar1[8] & 0xf7;
      FUN_08023788(DAT_0801a384,2);
    }
    else {
      if (((DAT_0801a37c[4] & 3) >> 1 != 0) && ((DAT_0801a37c[7] & 7) >> 2 == 0)) {
        DAT_0801a37c[4] = DAT_0801a37c[4] & 0xfd;
        pbVar1[7] = pbVar1[7] | 4;
        FUN_08023780(DAT_0801a384,0x2000);
        FUN_08024cf8((ushort)*(byte *)(DAT_0801a388 + 0x19) * 0x10 + 0x5f);
      }
      if (-1 < (char)pbVar1[2]) {
        if (*pcVar4 == '\x0f') {
          func_0x08020c9e();
          pbVar1[0xc] = pbVar1[0xc] | 0x40;
          iVar6 = *(int *)(iVar2 + 0x21);
          *(int *)(iVar2 + 0x21) = iVar6 + 1;
          if (iVar6 == 0x28) {
            if (*(char *)(DAT_0801a388 + 0x2f) == '\0') {
              *(undefined1 *)(DAT_0801a388 + 0x2f) = 1;
              FUN_080260a4(*_DAT_0801a39c,0xd);
            }
            else {
              *(undefined1 *)(DAT_0801a388 + 0x2f) = 0;
              FUN_080260a4(*_DAT_0801a39c,0xe);
            }
            FUN_0801fc50();
            FUN_0801a6ea();
          }
        }
        iVar6 = DAT_0801a388;
        if (((*(char *)(DAT_0801a388 + 0x2f) == '\0') || (*pcVar4 == '\x0f')) ||
           (iVar7 = func_0x08021a50(*pcVar4), iVar7 != 0xff)) {
          if ((pbVar1[2] & 7) >> 2 == 0) {
            pbVar1[6] = pbVar1[6] & 0x7f;
            *_DAT_0801a3a0 = *pcVar4;
            pbVar3 = DAT_0801a390;
            if ((*pcVar4 == '\v') || (*pcVar4 == '\f')) {
              func_0x08020c9e();
              uVar8 = *(uint *)(iVar2 + 0x21);
              *(uint *)(iVar2 + 0x21) = uVar8 + 1;
              if (0x27 < uVar8) {
                *pbVar1 = *pbVar1 | 8;
                iVar2 = _DAT_0801a3a4;
                pbVar1 = DAT_0801a390;
                (**(code **)((uint)*DAT_0801a390 * 0x10 + _DAT_0801a3a4 + 4))(*pcVar4);
                if (pbVar1[2] != 0) {
                  (**(code **)((uint)*pbVar1 * 0x10 + iVar2 + 8))(*pcVar4);
                  pbVar1[2] = 0;
                }
              }
            }
            else if (*pcVar4 == '\r') {
              func_0x08020c9e();
              iVar6 = *(int *)(iVar2 + 0x21);
              *(int *)(iVar2 + 0x21) = iVar6 + 1;
              if (iVar6 == 0x28) {
                FUN_08022cb4();
              }
            }
            else if (*pcVar4 == '\x0e') {
              func_0x08020c9e();
              iVar6 = *(int *)(iVar2 + 0x21);
              *(int *)(iVar2 + 0x21) = iVar6 + 1;
              if (iVar6 == 0x28) {
                FUN_08021238(1);
              }
            }
            else if (*pcVar4 == '\x12') {
              if ((*DAT_0801a390 == 0) || (*(char *)(iVar6 + 0x21) == '\x03')) {
                func_0x08020c9e();
                iVar7 = *(int *)(iVar2 + 0x21);
                *(int *)(iVar2 + 0x21) = iVar7 + 1;
                if (iVar7 == 0x28) {
                  FUN_08021238(*(undefined1 *)(iVar6 + 0x22));
                }
              }
              else {
                FUN_08020da6();
              }
            }
            else if (*pcVar4 == '\x13') {
              func_0x08020c9e();
              iVar7 = *(int *)(iVar2 + 0x21);
              *(int *)(iVar2 + 0x21) = iVar7 + 1;
              if (iVar7 == 0x28) {
                FUN_080214e4(*(undefined1 *)(iVar6 + 0x24));
              }
            }
            else if (*pcVar4 == '\x14') {
              if ((*DAT_0801a390 == 0) || (*(char *)(iVar6 + 0x25) == '\x02')) {
                func_0x08020c9e();
                iVar7 = *(int *)(iVar2 + 0x21);
                *(int *)(iVar2 + 0x21) = iVar7 + 1;
                if (iVar7 == 0x28) {
                  func_0x0802166c(*(undefined1 *)(iVar6 + 0x26));
                  pbVar1[0xc] = pbVar1[0xc] | 4;
                }
              }
              else {
                FUN_08020da6();
              }
            }
            else if (*DAT_0801a390 == 0x67) {
              if (*pcVar4 == '\x01') {
                func_0x08020c9e();
                iVar6 = *(int *)(iVar2 + 0x21);
                *(int *)(iVar2 + 0x21) = iVar6 + 1;
                if (iVar6 == 0x28) {
                  *(undefined1 *)(iVar2 + 0x168) = 1;
                  FUN_0801cd32(1,1,0x9e,0x10);
                  FUN_0801cdfc();
                  FUN_08014530(0x83);
                }
              }
              else if (*pcVar4 == '\x02') {
                func_0x08020c9e();
                iVar6 = *(int *)(iVar2 + 0x21);
                *(int *)(iVar2 + 0x21) = iVar6 + 1;
                if (iVar6 == 0x28) {
                  *(undefined1 *)(iVar2 + 0x168) = 2;
                  iVar2 = DAT_0801a394;
                  *(undefined1 *)(DAT_0801a394 + 2) = 1;
                  FUN_0801b15c(iVar2,4,1);
                  FUN_0801cd32(1,1,0x9e,0x10);
                  FUN_0801cdfc();
                  FUN_08014530(0x83);
                }
              }
            }
            else {
              if ((pbVar1[3] & 0xf) >> 3 == 0) {
                if (*pcVar4 == '\x0f') {
                  return;
                }
                pbVar1[3] = pbVar1[3] | 8;
                func_0x08020c9e();
                (**(code **)(_DAT_0801a3a4 + (uint)*pbVar3 * 0x10 + 4))(*pcVar4);
                func_0x08014c50(*pcVar4);
              }
              if (pbVar3[2] != 0) {
                (**(code **)(_DAT_0801a3a4 + (uint)*pbVar3 * 0x10 + 8))(*pcVar4);
                pbVar3[2] = 0;
              }
            }
          }
          else if ((*pcVar4 == '\x12') || (*pcVar4 == '\x13')) {
            func_0x08027e10();
          }
          else if ((*pcVar4 == '\x14') && (-1 < (char)pbVar1[7])) {
            pbVar1[7] = pbVar1[7] | 0x80;
            iVar2 = DAT_0801a38c;
            *(byte *)((uint)*(byte *)(iVar6 + 0x40) * 0x1e + DAT_0801a38c + 0xc) =
                 *(byte *)((uint)*(byte *)(iVar6 + 0x40) * 0x1e + DAT_0801a38c + 0xc) & 0xfe |
                 ~*(byte *)((uint)*(byte *)(iVar6 + 0x40) * 0x1e + DAT_0801a38c + 0xc) & 1;
            FUN_0801d8c0(&uStack_34,iVar2 + (uint)*(byte *)(iVar6 + 0x40) * 0x1e,0x1e);
            FUN_0801acfc(*(undefined1 *)(iVar6 + 0x40),uStack_34,uStack_30,uStack_2c);
            func_0x0801f104(*(undefined1 *)(iVar6 + 0x40));
            if (*(char *)(iVar6 + 0x40) == '\0') {
              FUN_0801f57c(0x32,0x16,*(byte *)(iVar2 + 0xc) & 1,1);
            }
            else {
              FUN_0801f57c(0x61,0x16,*(byte *)(iVar2 + 0x2a) & 1,1);
            }
            FUN_0801db54(*(undefined1 *)(iVar6 + 0x40));
            FUN_0801baa4(200);
          }
        }
        else {
          pbVar1[6] = pbVar1[6] & 0x7f;
          if (*pcVar4 != '\x0f') {
            FUN_08020da6();
          }
          *pcVar4 = -1;
        }
      }
    }
  }
  return;
}

 (GhidraScript)  
INFO  ReaCreateDecomp.java> REA-RESULT: END (GhidraScript)  
