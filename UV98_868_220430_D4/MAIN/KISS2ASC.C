
//#include "STC8A8K64D4.H"
//#include "BEACON.H"

#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "PUBLIC_BUF.H"

#include "KISS2ASC.H"



//18.5   2648   17384

//µçÌ¨½ÓÊÕµ½µÄ  KISSÊý¾Ý×ª»»ASCII UI¸ñÊ½,,format  0=Ô­Ê¼Êý¾Ý  1=²åÈë±¾»úºôºÅ

uchar ASC_IDX;

void ASCII_WRITE_SSID(unsigned char *nbuff, uchar SSID)
{
    if (SSID > 0)
    {
        nbuff[ASC_IDX++] = '-';

        if (SSID < 10)
        {
            nbuff[ASC_IDX++] = SSID + 0x30;       //×ª»»³ÉASCII
        }
        else
        {
            nbuff[ASC_IDX++] = SSID / 10 + 0x30;
            nbuff[ASC_IDX++] = SSID % 10 + 0x30;;
        }
    }
}


void ASCII_WRITE_CALL(unsigned char *nbuff, uchar idx)
{
    uchar temp;
    uchar i;

    for (i = idx; i < idx + 6; i++)
    {
        temp = ((KISS_DATA[i])  >> 1);

        if (temp != ' ')
        {
            nbuff[ASC_IDX++] = temp;
        }
    }
}

void KISS_TO_ASCII( unsigned char *nbuff, unsigned char format)
{
    unsigned char i, w, idx;

    /*  ---------------------      */		//×ª»»Ô´µØÖ·
    ASC_IDX = 0;
    ASCII_WRITE_CALL(nbuff, 7);	 //CALL
    ASCII_WRITE_SSID(nbuff, (KISS_DATA[13] & 0x1E) >> 1);	//SSID
    /*  ---------------------      */
    nbuff[ASC_IDX++] = '>';
    /*  ---------------------      */		   // ×ª»»Ä¿±êµØÖ·
    ASCII_WRITE_CALL(nbuff, 0);	 //CALL
    ASCII_WRITE_SSID(nbuff, (KISS_DATA[6] & 0x1E) >> 1); //SSID
    //----------------------------------------------------

    idx = 13;

    for (w = 0; w < 8; w++)		 //×î¶à8¸öÂ·¾¶
    {
        if ((KISS_DATA[idx] & 0x01) == 1)
        {
            break;   //Â·¾¶½áÊø·ûºÅ
        }

        nbuff[ASC_IDX++] = ',';

        idx++;
        ASCII_WRITE_CALL(nbuff, idx);	 //CALL

        idx = idx + 6;
        ASCII_WRITE_SSID(nbuff, (KISS_DATA[idx] & 0x1E) >> 1);	//SSID

        if ((KISS_DATA[idx] & 0x80) == 0x80)
        {
            nbuff[ASC_IDX++] = '*';	  	   //SSID.7=1£¬ÔòÎªÖÐ¼Ì×ª·¢¹ýµÄÊý¾Ý£¬²åÈë·ûºÅ*
        }
    }

    //----------------------------------------------------
    //  ·þÎñÆ÷Êý¾Ý²åÈëÍø¹ØÃû³Æ  format  0=Ô­Ê¼Êý¾Ý  1=²åÈë±¾»úºôºÅ
    if (format == 1)
    {
        if (EEPROM_Buffer[0X1A] == 1) 	//ÉÏ´«ÐÅ±êÊ±£¬ÊÇ·ñ²åÈë×Ô¼ºµÄºôºÅ£¬0=²»²åÈë 1=²åÈë
        {
            nbuff[ASC_IDX++] = ',';

            for (i = 0; i < 6; i++) 				//CALL
            {
                if (EEPROM_Buffer[0x08 + i] == 0x00)
                {
                    break;
                }

                nbuff[ASC_IDX++] = (EEPROM_Buffer[0x08 + i]);
            }


            ASCII_WRITE_SSID(nbuff, EEPROM_Buffer[0x0f]);	 //SSID
            nbuff[ASC_IDX++] = '*';
        }
    }

    /*  ---------------------      */
    nbuff[ASC_IDX++] = ':';	  // ²åÈë·ûºÅ ¡° £º¡±
    /*  ---------------------      */		   //·¢ËÍÊ£ÓàASCÐÅÏ¢
    idx = idx + 3;

    for (i = idx; i < KISS_LEN; i++)
    {
        nbuff[ASC_IDX++] = KISS_DATA[i];

        if (ASC_IDX > 190)
        {
            nbuff[ASC_IDX++] = '~';             //ÏÞ¶¨³¤¶È
            break;
        }
    }

    if (nbuff[ASC_IDX - 1] == 0x0d)
    {
        nbuff[ASC_IDX++] = '\n';
        nbuff[ASC_IDX++] = 0x00;
    }
    else
    {
        nbuff[ASC_IDX++] = '\r';
        nbuff[ASC_IDX++] = '\n';
        nbuff[ASC_IDX++] = 0x00;	   	//²¹ÉÏ»»ÐÐ·û

    }

    //ÒÔÉÏÎªÊý¾Ý¸ñÊ½×ª»»
}



////µçÌ¨½ÓÊÕµ½µÄ  KISSÊý¾Ý×ª»»ASCII UI¸ñÊ½,,format  0=Ô­Ê¼Êý¾Ý  1=²åÈë±¾»úºôºÅ
//void KISS_TO_ASCII( unsigned char *nbuff,unsigned char format)
//{
//	   unsigned char i,k ,w,idx,temp,ssid;
//
//
//		/*  ---------------------      */		//×ª»»Ô´µØÖ·
//		k=0;
//		for (i=7;i<13;i++)
//		{  	temp= ((KISS_DATA[i])  >> 1);	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
//
//		ssid=	 ((KISS_DATA[13] & 0x1E)>>1) ;
//		if (ssid>0)
//		{	   nbuff[k]= '-';	 k++;
//
//			if (ssid<10)
//			{ 	ssid |= 0x30; nbuff[k]=ssid;  	k++;  }	  //×ª»»³ÉASCII
//			else
//			{  nbuff[k]='1';  k++;	ssid=ssid-10;	nbuff[k]=(ssid |= 0x30);	 k++;	}
//		}
//
//		/*  ---------------------      */
//		 nbuff[k]= '>';  k++;
//		/*  ---------------------      */		   // ×ª»»Ä¿±êµØÖ·
//
//		for (i=0;i<6;i++)
//		{  	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
//
//		ssid=	 ((KISS_DATA[6] & 0x1E)>>1) ;
//		if (ssid>0)
//		{	   nbuff[k]= '-';	 k++;
//
//			if (ssid<10)
//			{ 	ssid |= 0x30; 	nbuff[k]=ssid;  	k++;  }	  //×ª»»³ÉASCII
//			else
//			{  nbuff[k]='1';  k++;	  	ssid=ssid-10; nbuff[k]=(ssid |= 0x30);	 k++;	}
//		}
//
//
//
//	  //----------------------------------------------------
//
//  	idx=13;
//	for (w=0;w<7;w++)		 //×î¶à7¸öÂ·¾¶
//	{
//		if ((KISS_DATA[idx] & 0x01) == 1){break;}	 //Â·¾¶½áÊø·ûºÅ
//
//
//		nbuff[k]= ',';	 k++;
//
//		idx++;
//		for (i=idx;i<(idx+6);i++)
//		{   	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
//
//
//
//		idx=idx+6;
//		ssid=	 ((KISS_DATA[idx] & 0x1E)>>1) ;
//		if (ssid>0)			  //Â·¾¶¼ÆÊý>0,ÏÔÊ¾£¬Â·¾¶¼ÆÊý=0£¬Ôò0ÏûÒþ
//		{	nbuff[k]= '-';	 k++;
//
//			if (ssid<10)
//			{ 	ssid |= 0x30; 		nbuff[k]=ssid;  	k++;  }  //×ª»»³ÉASCII
//			else
//			{  nbuff[k]='1';  k++;	ssid=ssid-10; 	 nbuff[k]=(ssid |= 0x30);	 k++;	}
//		}
//
//		if ((KISS_DATA[idx] & 0x80)==0x80)		{	nbuff[k]= '*';	 k++;	}	//SSID.7=1£¬ÔòÎªÖÐ¼Ì×ª·¢¹ýµÄÊý¾Ý£¬²åÈë·ûºÅ*
//	 }
//
//	  //----------------------------------------------------
//
//
//
////
////
////
////		   /*  ---------------------      */		   // ×ª»»ÆäÓàÂ·¾¶ÐÅÏ¢
////		l=7;	 //	path_count=0;
////		if ((KISS_DATA[13] & 0x01) != 1)
////		{
////			do{
////
////				nbuff[k]= ',';	 k++;
////
////				l=l+7;
////				for (i=l;i<(l+6);i++)
////				{   	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
////
////				ssid=	 ((KISS_DATA[l+6] & 0x1E)>>1) ;
////				if (ssid>0)			  //Â·¾¶¼ÆÊý>0,ÏÔÊ¾£¬Â·¾¶¼ÆÊý=0£¬Ôò0ÏûÒþ
////				{	nbuff[k]= '-';	 k++;
////
////					if (ssid<10)
////					{ 	ssid |= 0x30; 		nbuff[k]=ssid;  	k++;  }  //×ª»»³ÉtempII
////					else
////					{  nbuff[k]='1';  k++;	ssid=ssid-10; 	 nbuff[k]=(ssid |= 0x30);	 k++;	}
////
////				}
////
////				if ((KISS_DATA[l+6] & 0x80)==0x80)		{	nbuff[k]= '*';	 k++;	}	//SSID.7=1£¬ÔòÎªÖÐ¼Ì×ª·¢¹ýµÄÊý¾Ý£¬²åÈë·ûºÅ*
////
////
////			}while ((KISS_DATA[l+6] & 0x01) != 1);
////		}
//
//
//  	//  ·þÎñÆ÷Êý¾Ý²åÈëÍø¹ØÃû³Æ  format  0=Ô­Ê¼Êý¾Ý  1=²åÈë±¾»úºôºÅ
//
//	if (format==1)
//	{
//
//		if (EEPROM_Buffer[0X1A]==1) 	//ÉÏ´«ÐÅ±êÊ±£¬ÊÇ·ñ²åÈë×Ô¼ºµÄºôºÅ£¬0=²»²åÈë 1=²åÈë
//		{
//			nbuff[k]= ',';	 k++;
//
//			for (i=0;i<6;i++) 				//CALL
//			{
//			if (EEPROM_Buffer[0x08+i]==0x00){break;}
//			nbuff[k]=(EEPROM_Buffer[0x08+i]);   k++;
//			}
//
////			n=0x08;
////			while (EEPROM_Buffer[n]!=0x00)
////			{nbuff[k]=(EEPROM_Buffer[n]); n++; k++;}
//
//			nbuff[k]= '-';	 k++;
//
//			ssid=EEPROM_Buffer[0x0f];		 //SSID
//			if (ssid<10)		 //0-9
//			{	nbuff[k]=(ssid%10+0x30);   k++;}
// 			else
//			{	nbuff[k]=(ssid/10%10+0x30);  k++; 	nbuff[k]=(ssid%10+0x30);	  k++;
// 			}
//
//			nbuff[k]= '*';	 k++;
//		 }
//	}
//		//	²åÈëTCPIP*,qAS,BH4TDV
//		//  ²åÈëÍø¹ØÃû³Æ   ·þÎñÆ÷×Ô¶¯²åë
//	   /*  ---------------------      */
//	 	nbuff[k]= ':';	 k++;		  // ²åÈë·ûºÅ ¡° £º¡±
//
//	    /*  ---------------------      */		   //·¢ËÍÊ£ÓàASCÐÅÏ¢
//		idx=idx+3;
//
//		for (i=idx;i<KISS_LEN;i++)
//		{	nbuff[k]= KISS_DATA[i];	 k++;
//		if (k>120){ nbuff[k]= '~';   k++;       break;}  //ÏÞ¶¨³¤¶È
//		}
//
//		// while (l<KISS_LEN)	 {	 	nbuff[k]= KISS_DATA[l];	 k++; l++;	}
//
//		nbuff[k]= '\r';   k++;	nbuff[k]= '\n';	  k++;	//²¹ÉÏ»»ÐÐ·û
// 		nbuff[k]= 0x00;	  		//²¹ÉÏ½áÊø·û
//		//ÒÔÉÏÎªÊý¾Ý¸ñÊ½×ª»»
// 		//	return k; 	  //UIÊý¾Ý³¤È
//}
//
//






