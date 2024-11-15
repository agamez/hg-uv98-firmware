
// #include "STC8A8K64D4.H"
// #include "BEACON.H"

#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "PUBLIC_BUF.H"

#include "KISS2ASC.H"



// 18.5   2648   17384

// Convert the received KISS data to ASCII UI format, format 0 = original data 1 = insert the local call sign

uchar ASC_IDX;

void ASCII_WRITE_SSID(unsigned char *nbuff, uchar SSID)
{
    if (SSID > 0)
    {
        nbuff[ASC_IDX++] = '-';

        if (SSID < 10)
        {
            nbuff[ASC_IDX++] = SSID + 0x30;       // Convert to ASCII
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

    /*  ---------------------      */		// Convert source address
    ASC_IDX = 0;
    ASCII_WRITE_CALL(nbuff, 7);	 // CALL
    ASCII_WRITE_SSID(nbuff, (KISS_DATA[13] & 0x1E) >> 1);	// SSID
    /*  ---------------------      */
    nbuff[ASC_IDX++] = '>';
    /*  ---------------------      */		   // Conversion target address
    ASCII_WRITE_CALL(nbuff, 0);	 // CALL
    ASCII_WRITE_SSID(nbuff, (KISS_DATA[6] & 0x1E) >> 1); // SSID
    // ----------------------------------------------------

    idx = 13;

    for (w = 0; w < 8; w++)		 // Up to 8 paths
    {
        if ((KISS_DATA[idx] & 0x01) == 1)
        {
            break;   // End of path symbol
        }

        nbuff[ASC_IDX++] = ',';

        idx++;
        ASCII_WRITE_CALL(nbuff, idx);	 // CALL

        idx = idx + 6;
        ASCII_WRITE_SSID(nbuff, (KISS_DATA[idx] & 0x1E) >> 1);	// SSID

        if ((KISS_DATA[idx] & 0x80) == 0x80)
        {
            nbuff[ASC_IDX++] = '*';	  	   // If SSID.7=1, it is data that has been relayed, insert the symbol *
        }
    }

    // ----------------------------------------------------
    // Server data is inserted into the gateway name format 0 = original data 1 = insert the local call sign
    if (format == 1)
    {
        if (EEPROM_Buffer[0X1A] == 1) 	// When uploading a beacon, whether to insert your own call sign, 0 = do not insert 1 = insert
        {
            nbuff[ASC_IDX++] = ',';

            for (i = 0; i < 6; i++) 				// CALL
            {
                if (EEPROM_Buffer[0x08 + i] == 0x00)
                {
                    break;
                }

                nbuff[ASC_IDX++] = (EEPROM_Buffer[0x08 + i]);
            }


            ASCII_WRITE_SSID(nbuff, EEPROM_Buffer[0x0f]);	 // SSID
            nbuff[ASC_IDX++] = '*';
        }
    }

    /*  ---------------------      */
    nbuff[ASC_IDX++] = ':';	  // Caret &quot;:&quot;
    /*  ---------------------      */		   // Send remaining ASC information
    idx = idx + 3;

    for (i = idx; i < KISS_LEN; i++)
    {
        nbuff[ASC_IDX++] = KISS_DATA[i];

        if (ASC_IDX > 190)
        {
            nbuff[ASC_IDX++] = '~';             // Limited length
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
        nbuff[ASC_IDX++] = 0x00;	   	// Add line breaks

    }

    // The above is data format conversion
}



// Convert the received KISS data to ASCII UI format, format 0 = original data 1 = insert the local call sign
// void KISS_TO_ASCII( unsigned char *nbuff,unsigned char format)
// {
// unsigned char i,k ,w,idx,temp,ssid;
// 
// 
// /* --------------------- */ //Convert source address
// k=0;
// for (i=7;i<13;i++)
// {  	temp= ((KISS_DATA[i])  >> 1);	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
// 
// ssid=	 ((KISS_DATA[13] & 0x1E)>>1) ;
// if (ssid>0)
// { nbuff[k]= &#39;-&#39;; k++;
// 
// if (ssid<10)
// { ssid |= 0x30; nbuff[k]=ssid; k++; } //Convert to ASCII
// else
// {  nbuff[k]='1';  k++;	ssid=ssid-10;	nbuff[k]=(ssid |= 0x30);	 k++;	}
// }
// 
// /*  ---------------------      */
// nbuff[k]= &#39;&gt;&#39;; k++;
// /* --------------------- */ // Convert target address
// 
// for (i=0;i<6;i++)
// {  	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
// 
// ssid=	 ((KISS_DATA[6] & 0x1E)>>1) ;
// if (ssid>0)
// { nbuff[k]= &#39;-&#39;; k++;
// 
// if (ssid<10)
// { ssid |= 0x30; nbuff[k]=ssid; k++; } //Convert to ASCII
// else
// {  nbuff[k]='1';  k++;	  	ssid=ssid-10; nbuff[k]=(ssid |= 0x30);	 k++;	}
// }
// 
// 
// 
// //----------------------------------------------------
// 
// idx=13;
// for (w=0;w<7;w++)		 //最多7个路径
// {
// if ((KISS_DATA[idx] &amp; 0x01) == 1){break;} //path end symbol
// 
// 
// nbuff[k]= &#39;,&#39;; k++;
// 
// idx++;
// for (i=idx;i<(idx+6);i++)
// {   	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
// 
// 
// 
// idx=idx+6;
// ssid=	 ((KISS_DATA[idx] & 0x1E)>>1) ;
// if (ssid&gt;0) //path count&gt;0, display, path count=0, then 0 is blanked
// { nbuff[k]= &#39;-&#39;; k++;
// 
// if (ssid<10)
// { ssid |= 0x30; nbuff[k]=ssid; k++; } //Convert to ASCII
// else
// {  nbuff[k]='1';  k++;	ssid=ssid-10; 	 nbuff[k]=(ssid |= 0x30);	 k++;	}
// }
// 
// if ((KISS_DATA[idx] &amp; 0x80)==0x80) { nbuff[k]= &#39;*&#39;; k++; } //SSID.7=1, then it is the data forwarded by the relay, insert the symbol *
// }
// 
// //----------------------------------------------------
// 
// 
// 
// 
// 
// 
// /* --------------------- */ // Convert the rest of the path information
// l=7;	 //	path_count=0;
// if ((KISS_DATA[13] & 0x01) != 1)
// {
// do{
// 
// nbuff[k]= &#39;,&#39;; k++;
// 
// l=l+7;
// for (i=l;i<(l+6);i++)
// {   	temp= ((KISS_DATA[i])  >> 1); 	if (temp !=' ')	  {	 nbuff[k]= temp;  k++;	}  	 }
// 
// ssid=	 ((KISS_DATA[l+6] & 0x1E)>>1) ;
// if (ssid&gt;0) //path count&gt;0, display, path count=0, then 0 is blanked
// { nbuff[k]= &#39;-&#39;; k++;
// 
// if (ssid<10)
// { ssid |= 0x30; nbuff[k]=ssid; k++; } //Convert to tempII
// else
// {  nbuff[k]='1';  k++;	ssid=ssid-10; 	 nbuff[k]=(ssid |= 0x30);	 k++;	}
// 
// }
// 
// if ((KISS_DATA[l+6] &amp; 0x80)==0x80) { nbuff[k]= &#39;*&#39;; k++; } // SSID.7=1, it is the data forwarded by the relay, insert the symbol *
// 
// 
// }while ((KISS_DATA[l+6] & 0x01) != 1);
// }
// 
// 
// // Server data is inserted into the gateway name format 0 = original data 1 = insert the local call sign
// 
// if (format==1)
// {
// 
// if (EEPROM_Buffer[0X1A]==1) //Whether to insert your own call sign when uploading the beacon, 0=not insert 1=insert
// {
// nbuff[k]= &#39;,&#39;; k++;
// 
// for (i=0;i<6;i++) 				//CALL
// {
// if (EEPROM_Buffer[0x08+i]==0x00){break;}
// nbuff[k]=(EEPROM_Buffer[0x08+i]);   k++;
// }
// 
// n=0x08;
// while (EEPROM_Buffer[n]!=0x00)
// {nbuff[k]=(EEPROM_Buffer[n]); n++; k++;}
// 
// nbuff[k]= &#39;-&#39;; k++;
// 
// ssid=EEPROM_Buffer[0x0f];		 //SSID
// if (ssid<10)		 //0-9
// {	nbuff[k]=(ssid%10+0x30);   k++;}
// else
// {	nbuff[k]=(ssid/10%10+0x30);  k++; 	nbuff[k]=(ssid%10+0x30);	  k++;
// }
// 
// nbuff[k]= &#39;*&#39;; k++;
// }
// }
// // Insert TCPIP*,qAS,BH4TDV
// // Insert gateway name server automatically inserted
// /*  ---------------------      */
// nbuff[k]= &#39;:&#39;; k++; // insert symbol &quot; ：&quot;
// 
// /* --------------------- */ //Send the remaining ASC information
// idx=idx+3;
// 
// for (i=idx;i<KISS_LEN;i++)
// {	nbuff[k]= KISS_DATA[i];	 k++;
// if (k&gt;120){ nbuff[k]= &#39;~&#39;; k++; break;} //Limit length
// }
// 
// // while (l<KISS_LEN)	 {	 	nbuff[k]= KISS_DATA[l];	 k++; l++;	}
// 
// nbuff[k]= &#39;\r&#39;; k++; nbuff[k]= &#39;\n&#39;; k++; //Add newline character
// nbuff[k]= 0x00; //Add the end character
// //The above is the data format conversion
// // return k; //UI data length
// }
// 
// 






