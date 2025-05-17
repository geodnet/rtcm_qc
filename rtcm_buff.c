#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "rtcm_buff.h"

#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */

#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

#define ROUND_U(x)  ((uint32_t)floor((x)+0.5))

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

extern unsigned int crc24q_(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

extern void setbitu_(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}
extern void setbits_(uint8_t *buff, int pos, int len, int32_t data)
{
    if (data<0) data|=1<<(len-1); else data&=~(1<<(len-1)); /* set sign bit */
    setbitu_(buff,pos,len,(uint32_t)data);
}
/* set signed 38 bit field ---------------------------------------------------*/
extern void set38bits_(uint8_t *buff, int pos, double value)
{
    int word_h=(int)floor(value/64.0);
    uint32_t word_l=(uint32_t)(value-word_h*64.0);
    setbits_(buff,pos  ,32,word_h);
    setbitu_(buff,pos+32,6,word_l);
}
extern unsigned int getbitu_(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits_(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu_(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}
/* get signed 38bit field ----------------------------------------------------*/
extern double getbits_38_(const unsigned char *buff, int pos)
{
    return (double)getbits_(buff,pos,32)*64.0+getbitu_(buff,pos+32,6);
}
/* decode type 1005: stationary rtk reference station arp --------------------*/
extern int decode_type1005_(unsigned char *buff, int len, int *staid, double *pos)
{
    double rr[3];
    int i=24+12,j/*,staid,itrf*/;
    
    if (i+140<=len*8) {
        *staid=getbitu_(buff,i,12); i+=12;
        /*itrf =getbitu_(buff,i, 6);*/ i+= 6+4;
        rr[0]=getbits_38_(buff,i); i+=38+2;
        rr[1]=getbits_38_(buff,i); i+=38+2;
        rr[2]=getbits_38_(buff,i);
    }
    else {
        return -1;
    }
    
    for (j=0;j<3;j++) {
        pos[j]=rr[j]*0.0001;
    }
    return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
extern int decode_type1006_(unsigned char *buff, int len, int *staid, double *pos)
{
    double rr[3]/*,anth*/;
    int i=24+12,j/*,staid,itrf*/;
    
    if (i+156<=len*8) {
        *staid=getbitu_(buff,i,12); i+=12;
        /*itrf =getbitu_(buff,i, 6);*/ i+= 6+4;
        rr[0]=getbits_38_(buff,i); i+=38+2;
        rr[1]=getbits_38_(buff,i); i+=38+2;
        rr[2]=getbits_38_(buff,i); i+=38;
        /*anth =getbitu_(buff,i,16);*/
    }
    else {
        return -1;
    }
    
    for (j=0;j<3;j++) {
        pos[j]=rr[j]*0.0001;
    }
    return 5;
}
static int add_rtcm_to_buff(rtcm_buff_t* rtcm, unsigned char data)
{
    if (rtcm->sync == 0) rtcm->slen = 0;
    rtcm->type=0;
    rtcm->crc=0;
    rtcm->staid=0;

    if (rtcm->nbyte>=MAX_RTCM_BUF_LEN) rtcm->nbyte = 0;
    if (rtcm->nbyte==0) {
        rtcm->len = 0;
        if (data!=RTCM3PREAMB) return 0;
        rtcm->buff[rtcm->nbyte++]=data;
        return 0;
    }
    rtcm->buff[rtcm->nbyte++] = data;
    return 1;
}

extern int input_rtcm3_type(rtcm_buff_t *rtcm, unsigned char data, int fix_sync)
{
    int ret = 0, i = 24, j = 0, mask = 0, is_obs = 0, nbyte = rtcm->nbyte, is_msm4 = 0, is_msm5 = 0, is_msm6 = 0, is_msm7 = 0;
    int rng=0,rng_m=0,prv=0,cpv=0,rate=0,rrv=0;
    rtcm->tow_pre=rtcm->tow;
    rtcm->numofbyte++;
    if (rtcm->sync == 0) rtcm->slen = 0;
    if (add_rtcm_to_buff(rtcm, data) == 0) return 0;
    
    if (rtcm->nbyte < 3) return 0;
    rtcm->len=getbitu_(rtcm->buff,14,10)+3; /* length without parity */
    if (rtcm->nbyte<rtcm->len+3) return 0;
    nbyte = rtcm->nbyte;
    rtcm->nbyte=0;
    /* check parity */
    rtcm->numofmsg++;
    if (crc24q_(rtcm->buff, rtcm->len) != getbitu_(rtcm->buff, rtcm->len * 8, 24)) {
        rtcm->crc = 1;
        rtcm->numofcrc++;
        return 0;
    }
    i = 24;
    rtcm->type = getbitu_(rtcm->buff, i, 12); i += 12;
    /* decode rtcm3 message */
    if ((rtcm->type == 1074 || rtcm->type == 1075 || rtcm->type == 1076 || rtcm->type == 1077)|| /* GPS */
        (rtcm->type == 1094 || rtcm->type == 1095 || rtcm->type == 1096 || rtcm->type == 1097)|| /* GAL */
        (rtcm->type == 1104 || rtcm->type == 1105 || rtcm->type == 1106 || rtcm->type == 1107)|| /* SBS */
        (rtcm->type == 1114 || rtcm->type == 1115 || rtcm->type == 1116 || rtcm->type == 1117)|| /* QZS */
        (rtcm->type == 1134 || rtcm->type == 1135 || rtcm->type == 1136 || rtcm->type == 1137))  /* IRN */
    {
        /* GPS, GAL, SBS, QZS, IRN */
        rtcm->staid = getbitu_(rtcm->buff, i, 12);           i += 12;
        rtcm->tow   = getbitu_(rtcm->buff, i, 30) * 0.001;   i += 30;
        rtcm->sync  = getbitu_(rtcm->buff, i,  1);           i +=  1;
        ret = rtcm->sync?0:1;
        is_obs = 1;
    }
    else if (rtcm->type == 1084 || rtcm->type == 1085 || rtcm->type == 1086 || rtcm->type == 1087)  /* GLO */
    {
        /* GLO */
        rtcm->staid = getbitu_(rtcm->buff, i, 12);           i += 12;
        double dow  = getbitu_(rtcm->buff, i,  3);           i +=  3;
        double tod  = getbitu_(rtcm->buff, i, 27) * 0.001;   i += 27;
        rtcm->sync  = getbitu_(rtcm->buff, i,  1);           i +=  1;
        double tow  = dow * 24 * 3600 + tod - 3 * 3600 + 18;
        if (rtcm->tow>0.0&&fabs(tow- rtcm->tow)>(24*1800))
        {
            tow -= floor((tow- rtcm->tow)/(24*1800))*(24*1800);
        }
        tow -= floor(tow / 604800) * 604800;
        rtcm->tow   = tow;
        ret = rtcm->sync?0:1;
        is_obs = 1;
    }
    else if (rtcm->type == 1124 || rtcm->type == 1125 || rtcm->type == 1126 || rtcm->type == 1127)  /* BDS */
    {
        /* BDS */
        rtcm->staid = getbitu_(rtcm->buff, i, 12);           i += 12;
        double tow  = getbitu_(rtcm->buff, i, 30) * 0.001;   i += 30;
        rtcm->sync  = getbitu_(rtcm->buff,i, 1);             i +=  1;
        tow += 14.0; /* BDT -> GPST */
        tow -= floor(tow / 604800) * 604800;
        rtcm->tow = tow;
        ret = rtcm->sync?0:1;
        is_obs = 1;
    }
    else if (rtcm->type == 1019)
    {
        int prn   =getbitu_(rtcm->buff,i, 6);              i+= 6;
        int week  =getbitu_(rtcm->buff,i,10);              i+=10;
        rtcm->wk  =week+2048;
        rtcm->sys = 'G';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }
    else if (rtcm->type == 1020)
    {
        int prn   =getbitu_(rtcm->buff,i, 6);              i+= 6;
        int frq   =getbitu_(rtcm->buff,i, 5)-7;            i+= 5+2+2;
        rtcm->sys = 'R';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }
    else if (rtcm->type == 1041)
    {
        int prn   =getbitu_(rtcm->buff,i, 4);              i+= 6;
        int week  =getbitu_(rtcm->buff,i,10);              i+=10;
        rtcm->wk  =week+2048;
        rtcm->sys = 'I';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }		
    else if (rtcm->type == 1042)
    {
        int prn   =getbitu_(rtcm->buff,i, 6);              i+= 6;
        int week  =getbitu_(rtcm->buff,i,13);              i+=13;
        rtcm->wk  =week+1356; /* BDT week to GPS week */
        rtcm->sys = 'C';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }
   else  if (rtcm->type == 1044)
    {
        int prn   =getbitu_(rtcm->buff,i, 4);              i+= 4+430;
        int week  =getbitu_(rtcm->buff,i,10);              i+=10;
        rtcm->wk  =week+2048;
        rtcm->sys = 'J';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }		
    else if (rtcm->type == 1045|| rtcm->type == 1046)
    {
        int prn   =getbitu_(rtcm->buff,i, 6);              i+= 6;
        int week  =getbitu_(rtcm->buff,i,12);              i+=12; /* gst-week */
        rtcm->wk  =week+1024 ; /* gal-week = gst-week + 1024 */
        rtcm->sys = 'E';
        rtcm->prn = prn;
        rtcm->numofmsg_eph++;
    }
    else if (rtcm->type == 1005)
    {
        ret = decode_type1005_(rtcm->buff, rtcm->len, &rtcm->staid, rtcm->pos);
    }
    else if (rtcm->type == 1006)
    {
        ret = decode_type1006_(rtcm->buff, rtcm->len, &rtcm->staid, rtcm->pos);
    }
    else if (rtcm->type == 1033)
    {
        ret = decode_type1033_(rtcm->buff, rtcm->len, &rtcm->staid, rtcm->antdes, rtcm->antsno, rtcm->rectype, rtcm->recver, rtcm->recsno);
    }
    else if (rtcm->type == 1230)
    {
        ret = decode_type1230_(rtcm->buff, rtcm->len, &rtcm->staid, &rtcm->glo_cp_align, rtcm->glo_cp_bias);
    }
    else if (rtcm->type == 4054)
    {
        int vers = getbitu_(rtcm->buff, 24 + 12, 3);
        int stype = getbitu_(rtcm->buff, 24 + 12 + 3, 9);
        if (stype == 300)
        {
            rtcm->v1 = getbitu_(rtcm->buff, 85, 25);
            rtcm->v2 = getbitu_(rtcm->buff, 124, 7);
            rtcm->v3 = getbitu_(rtcm->buff, 131, 2);
            rtcm->v4 = getbitu_(rtcm->buff, 135, 3);
        }
    }
    if (is_obs)
    {
        rtcm->numofmsg_obs++;
        rtcm->cur_obscount++;
        char *pstr = rtcm->msg;
        int nstr = 0;
        nstr += sprintf(pstr + nstr, "%10.3f,%4i,%4i,%i,%i,%2i,%2i", rtcm->tow, rtcm->len+3, rtcm->type, rtcm->sync, ret, rtcm->cur_obscount, rtcm->pre_obscount);
        if (rtcm->misorder == 3 || rtcm->misorder == 4) rtcm->misorder = 0;
        if (rtcm->numofmsg_obs > 1) /* more message */
        {
            rtcm->etime = rtcm->tow;
            rtcm->dt = rtcm->tow - rtcm->tow_pre;
            if (fabs(rtcm->dt) < 0.001) /* same epoch */
            {
                if (ret == 1)   /* sync flag = 0 => epoch completed ? */
                {
                    rtcm->numofsync++; /* sync message count (flag=0) */
                    if (rtcm->pre_obscount == 0) /* first epoch */
                    {
                        rtcm->pre_obscount = rtcm->cur_obscount;
                        rtcm->cur_obscount = 0;
                        rtcm->misorder = 0;
                    }
                    else /* more epochs */
                    {
                        if (rtcm->cur_obscount >= rtcm->pre_obscount) /* normal data, no sync issue */
                        {
                            rtcm->pre_obscount = rtcm->cur_obscount;
                            rtcm->cur_obscount = 0;
                            rtcm->misorder = 0;
                        }
                        else /* sync issue, need to fix the sync flag */
                        {
                            rtcm->numofmissync++;
                            rtcm->misorder = 1;
                            if (fix_sync)
                            {
                                update_msm_sync_(rtcm->buff, rtcm->len + 3, 1); /* more message */
                                rtcm->misorder = 2;/* fixed */
                                rtcm->sync = getbitu_(rtcm->buff, 24 + 12 + 12 + 30, 1);
                                ret = !rtcm->sync;
                                nstr += sprintf(pstr + nstr, ",%i,%i,%i,fix sync", rtcm->sync, ret, rtcm->misorder);
                            }
                        }
                    }
                }
                /* same epoch sync flag = 1 */
                else if (rtcm->pre_obscount > 0 && rtcm->cur_obscount >= rtcm->pre_obscount) /* check the epoch complete by obs msg counter */
                {
                    if (rtcm->misorder == 2)
                    {
                        update_msm_sync_(rtcm->buff, rtcm->len + 3, 0); /* no more message */
                        rtcm->sync = getbitu_(rtcm->buff, 24 + 12 + 12 + 30, 1);
                        ret = !rtcm->sync;
                        nstr += sprintf(pstr + nstr, ",%i,%i,%i,fix sync", rtcm->sync, ret, rtcm->misorder);
                    }
                    rtcm->pre_obscount = rtcm->cur_obscount;
                    rtcm->cur_obscount = 0;
                    rtcm->misorder = 0;
                }
            }
            else /* new epoch */
            {
                if (rtcm->dt < -7 * 24 * 1800) rtcm->dt += 7 * 24 * 3600;
                else if (rtcm->dt > 7 * 24 * 1800) rtcm->dt -= 7 * 24 * 3600;
                if (rtcm->dt < 0.0001)
                {
                    rtcm->misorder = 4; /* do not output */
                    nstr += sprintf(pstr + nstr, ",%i,%i,%i,last message arrived later", rtcm->sync, ret, rtcm->misorder);
                    if (rtcm->cur_obscount > 0) --rtcm->cur_obscount;
                    rtcm->numofmistime++;
                    /* switch tow with pre_tow */
                    double cur_tow = rtcm->tow;
                    rtcm->tow = rtcm->tow_pre;
                    rtcm->tow_pre = cur_tow;
                }
                else
                {
                    if (rtcm->cur_obscount > 1)   /* missed the last sync message (in the previous epoch) */
                    {
                        rtcm->misorder = 3;
                        nstr += sprintf(pstr + nstr, ",%i,%i,%i,missed the last sync message", rtcm->sync, ret, rtcm->misorder);
                        rtcm->cur_obscount = 1;
                    }
                    else if (rtcm->cur_obscount == 1)
                    {
                    }
                    else
                    {
                        rtcm->cur_obscount = rtcm->cur_obscount;
                    }
                    rtcm->numofepo++;
                }
            }
        }
        else /* first message */
        {
            rtcm->stime = rtcm->tow;
            if (ret == 1) /* first message with sync flag = 0 (ret=1) */
            {
                rtcm->numofsync++;
                rtcm->cur_obscount = 0;
            }
            rtcm->numofepo++;
        }
        nstr += sprintf(pstr + nstr, "\n");
        //printf("%s", rtcm->msg);
#if 0        
		is_msm4 = (rtcm->type==1074||rtcm->type==1084||rtcm->type==1094||rtcm->type==1104||rtcm->type==1114||rtcm->type==1124||rtcm->type==1134);
		is_msm5 = (rtcm->type==1075||rtcm->type==1085||rtcm->type==1095||rtcm->type==1105||rtcm->type==1115||rtcm->type==1125||rtcm->type==1135);
		is_msm6 = (rtcm->type==1076||rtcm->type==1086||rtcm->type==1096||rtcm->type==1106||rtcm->type==1116||rtcm->type==1126||rtcm->type==1136);
		is_msm7 = (rtcm->type==1077||rtcm->type==1087||rtcm->type==1097||rtcm->type==1107||rtcm->type==1117||rtcm->type==1127||rtcm->type==1137);

        memset(rtcm->sats, 0, sizeof(rtcm->sats));
        memset(rtcm->sigs, 0, sizeof(rtcm->sigs));
        memset(rtcm->cels, 0, sizeof(rtcm->cels));
        rtcm->nsat = 0;
        rtcm->nsig = 0;
        rtcm->ncel = 0;
        /* iod */      i += 3;
        /* time_s*/    i += 7;
        /* clk_str */  i += 2;
        /* clk_ext */  i += 2;
        /* smooth */   i += 1;
        /* tint_s */   i += 3;
        for (j = 1; j <= 64; j++) {
            mask = getbitu_(rtcm->buff, i, 1); i += 1;
            if (mask) rtcm->sats[rtcm->nsat++] = j;
        }
        for (j = 1; j <= 32; j++) {
            mask = getbitu_(rtcm->buff, i, 1); i += 1;
            if (mask) rtcm->sigs[rtcm->nsig++] = j;
        }
		if (rtcm->nsat * rtcm->nsig>64)
		{
            /* error */
            rtcm->nsat = rtcm->nsig = rtcm->ncel = 0;
		}
		else if (i + rtcm->nsat * rtcm->nsig > rtcm->len * 8) {
            /* error */
            rtcm->nsat = rtcm->nsig = rtcm->ncel = 0;
        }
        else
        {
            for (j = 0; j < rtcm->nsat * rtcm->nsig; j++) {
                rtcm->cels[j] = getbitu_(rtcm->buff, i, 1); i += 1;
                if (rtcm->cels[j]) rtcm->ncel++;
            }

			if ((is_msm4 && (i+rtcm->nsat*18+rtcm->ncel*48)<=rtcm->len*8)||
 				(is_msm5 && (i+rtcm->nsat*36+rtcm->ncel*63)<=rtcm->len*8)||
				(is_msm6 && (i+rtcm->nsat*18+rtcm->ncel*65)<=rtcm->len*8)||
				(is_msm7 && (i+rtcm->nsat*36+rtcm->ncel*80)<=rtcm->len*8))
			{

				for (j=0;j<rtcm->nsat;j++) {
					rtcm->r[j]=rtcm->rr[j]=0.0; rtcm->ex[j]=15;
				}
				for (j=0;j<rtcm->ncel;j++) rtcm->pr[j]=rtcm->cp[j]=rtcm->rrf[j]=-1E16;

				/* MSM4 => nsat * (8  +10   ) + ncel * (15+22+ 4+1+ 6   ) = nsat *18 + ncel *48 */
				/* MSM5 => nsat * (8+4+10+14) + ncel * (15+22+ 4+1+ 6+15) = nsat *36 + ncel *63 */
				/* MSM6 => nsat * (8  +10   ) + ncel * (20+24+10+1+10   ) = nsat *18 + ncel *65 */
				/* MSM7 => nsat * (8+4+10+14) + ncel * (20+24+10+1+10+15) = nsat *36 + ncel *80 */

				/* decode satellite data */
				for (j=0;j<rtcm->nsat;j++) { /* range */
					rng  =getbitu_(rtcm->buff,i, 8); i+= 8;
					if (rng!=255) rtcm->r[j]=rng*RANGE_MS;
				}
				if (is_msm5||is_msm7)
				{
					for (j=0;j<rtcm->nsat;j++) { /* extended info */
						rtcm->ex[j]=getbitu_(rtcm->buff,i, 4); i+= 4;
					}
				}
				for (j=0;j<rtcm->nsat;j++) {
					rng_m=getbitu_(rtcm->buff,i,10); i+=10;
					if (rtcm->r[j]!=0.0) rtcm->r[j]+=rng_m*P2_10*RANGE_MS;
				}
				if (is_msm5||is_msm7)
				{
					for (j=0;j<rtcm->nsat;j++) { /* phaserangerate */
						rate =getbits_(rtcm->buff,i,14); i+=14;
						if (rate!=-8192) rtcm->rr[j]=rate*1.0;
					}
				}
				if (is_msm4||is_msm5)
				{
					/* decode signal data */
					for (j=0;j<rtcm->ncel;j++) { /* pseudorange */
						prv=getbits_(rtcm->buff,i,15); i+=15;
						if (prv!=-16384) rtcm->pr[j]=prv*P2_24*RANGE_MS;
					}
					for (j=0;j<rtcm->ncel;j++) { /* phaserange */
						cpv=getbits_(rtcm->buff,i,22); i+=22;
						if (cpv!=-2097152) rtcm->cp[j]=cpv*P2_29*RANGE_MS;
					}
					for (j=0;j<rtcm->ncel;j++) { /* lock time */
                        rtcm->lock[j]=getbitu_(rtcm->buff,i,4); i+=4;
					}
					for (j=0;j<rtcm->ncel;j++) { /* half-cycle amiguity */
						rtcm->half[j]=getbitu_(rtcm->buff,i,1); i+=1;
					}
					for (j=0;j<rtcm->ncel;j++) { /* cnr */
                        rtcm->cnr[j]=getbitu_(rtcm->buff,i,6)*1.0; i+=6;
					}
				} 
				else if (is_msm6||is_msm7)
				{
					/* decode signal data */
					for (j=0;j<rtcm->ncel;j++) { /* pseudorange */
						prv=getbits_(rtcm->buff,i,20); i+=20;
						if (prv!=-524288) rtcm->pr[j]=prv*P2_29*RANGE_MS;
					}
					for (j=0;j<rtcm->ncel;j++) { /* phaserange */
						cpv=getbits_(rtcm->buff,i,24); i+=24;
						if (cpv!=-8388608) rtcm->cp[j]=cpv*P2_31*RANGE_MS;
					}
					for (j=0;j<rtcm->ncel;j++) { /* lock time */
						rtcm->lock[j]=getbitu_(rtcm->buff,i,10); i+=10;
					}
					for (j=0;j<rtcm->ncel;j++) { /* half-cycle amiguity */
						rtcm->half[j]=getbitu_(rtcm->buff,i,1); i+=1;
					}
					for (j=0;j<rtcm->ncel;j++) { /* cnr */
						rtcm->cnr[j]=getbitu_(rtcm->buff,i,10)*0.0625; i+=10;
					}
				}
				if (is_msm5||is_msm7)
				{
					for (j=0;j<rtcm->ncel;j++) { /* phaserangerate */
						rrv=getbits_(rtcm->buff,i,15); i+=15;
						if (rrv!=-16384) rtcm->rrf[j]=rrv*0.0001;
					}
				}
			}
		}
#endif            
    }    
    rtcm->slen += rtcm->len + 3;
    return ret;
}

extern int rtcm_obs_type(int type)
{
    if ((type == 1071 || type == 1072 || type == 1073 || type == 1074 || type == 1075 || type == 1076 || type == 1077) || /* GPS */
        (type == 1081 || type == 1082 || type == 1083 || type == 1084 || type == 1085 || type == 1086 || type == 1087) || /* GLO */
        (type == 1091 || type == 1092 || type == 1093 || type == 1094 || type == 1095 || type == 1096 || type == 1097) || /* GAL */
        (type == 1101 || type == 1102 || type == 1103 || type == 1104 || type == 1105 || type == 1106 || type == 1107) || /* SBS */
        (type == 1111 || type == 1112 || type == 1113 || type == 1114 || type == 1115 || type == 1116 || type == 1117) || /* QZS */
        (type == 1121 || type == 1122 || type == 1123 || type == 1124 || type == 1125 || type == 1126 || type == 1127) || /* BDS */
        (type == 1131 || type == 1132 || type == 1133 || type == 1134 || type == 1135 || type == 1136 || type == 1137))   /* IRN */
        return 1;
    else
        return 0;
}
extern int rtcm_eph_type(int type)
{
    return type == 1019 || type == 1020 || type == 1041 || type == 1042 || type == 1044 || type == 1045 || type == 1046;
}

extern int update_type_1005_1006_pos(uint8_t* buff, int nbyte, double* p)
{
    int len = 0, i = 24, type = 0, staid = 0;
    int crc = 0;
    int ret = 0;
    if (buff[0] != RTCM3PREAMB || nbyte < 6) return ret;
    len = getbitu_(buff, 14, 10) + 3; /* length without parity */
    if (nbyte < (len + 3)) return ret;

    i = 24; /* type */
    type = getbitu_(buff, i, 12); i += 12;

    if (type == 1005 || type == 1006)
    {
        /* 24 + 12, staid */
        i += 12; /* ref station id */
        i += 6; /* itrf realization year */
        i += 1; /* gps indicator */
        i += 1; /* glonass indicator */
        i += 1; /* galileo indicator */
        i += 1; /* ref station indicator */
        set38bits_(buff, i, p[0] / 0.0001); i += 38; /* antenna ref point ecef-x */
        i += 1; /* oscillator indicator */
        i += 1; /* reserved */
        set38bits_(buff, i, p[1] / 0.0001); i += 38; /* antenna ref point ecef-y */
        i += 2; /* quarter cycle indicator */
        set38bits_(buff, i, p[2] / 0.0001); i += 38; /* antenna ref point ecef-z */
        if (type==1006)
        {
        setbitu_  (buff, i, 16,         0); i += 16; /* antenna height */
        }
        /* crc-24q */
        crc = crc24q_(buff, len);
        setbitu_(buff, len * 8, 24, crc);
        ret = 1;
    }
    return ret;
}
extern int update_msm_sync_(uint8_t* buff, int nbyte, int sync)
{
    int len = 0, i = 24, type = 0;
    int crc = 0;
    int ret = 0;
    if (buff[0] != RTCM3PREAMB || nbyte < 6) return ret;
    len = getbitu_(buff, 14, 10) + 3; /* length without parity */
    if (nbyte < (len + 3)) return ret;

    i = 24; /* type */
    type = getbitu_(buff, i, 12); i += 12;

    if (rtcm_obs_type(type))
    {
        i += 12;    /* staid */
        i += 30;    /* tow */
        setbitu_(buff, i, 1, sync); i += 1; /* sync */
        /* crc-24q */
        crc = crc24q_(buff, len);
        setbitu_(buff, len * 8, 24, crc);
        ret = 1;
    }
    return ret;
}
extern int encode_msm4_sync(uint8_t* buff, double tow, int type, int staid, int sync)
{
    uint8_t sat_ind[64]={0},sig_ind[32]={0},cell_ind[32*64]={0};
    uint32_t dow=0,epoch=0,crc=0;
    int i=0,j,nsat=0,nsig=0,seqno=0,len=0;
    
	if (type==1074||type==1084||type==1094||type==1104||type==1114||type==1124||type==1134)
	{
	}
	else
	{
		return 0;
	}
    
    if (type==1084) {
        /* GLONASS time (dow + tod-ms) */
        dow=(uint32_t)(tow/86400.0);
        epoch=(dow<<27)+ROUND_U(fmod(tow,86400.0)*1E3);
    }
    else if (type==1124) {
        /* BDS time (tow-ms) */
		tow-=14.0;
        epoch=ROUND_U(tow*1E3);
    }
    else {
        /* GPS, QZSS, Galileo and IRNSS time (tow-ms) */
        epoch=ROUND_U(tow*1E3);
    }

    /* set preamble and reserved */
    setbitu_(buff,i, 8,RTCM3PREAMB); i+= 8;
    setbitu_(buff,i, 6,0          ); i+= 6;
    setbitu_(buff,i,10,0          ); i+=10;

    /* encode msm header (ref [15] table 3.5-78) */
    setbitu_(buff,i,12,type       ); i+=12; /* message number */
    setbitu_(buff,i,12,staid      ); i+=12; /* reference station id */
    setbitu_(buff,i,30,epoch      ); i+=30; /* epoch time */
    setbitu_(buff,i, 1,sync       ); i+= 1; /* multiple message bit */
    setbitu_(buff,i, 3,seqno      ); i+= 3; /* issue of data station */
    setbitu_(buff,i, 7,0          ); i+= 7; /* reserved */
    setbitu_(buff,i, 2,0          ); i+= 2; /* clock streering indicator */
    setbitu_(buff,i, 2,0          ); i+= 2; /* external clock indicator */
    setbitu_(buff,i, 1,0          ); i+= 1; /* smoothing indicator */
    setbitu_(buff,i, 3,0          ); i+= 3; /* smoothing interval */
    
    /* satellite mask */
    for (j=0;j<64;j++) {
        setbitu_(buff,i,1,sat_ind[j]?1:0); i+=1;
    }
    /* signal mask */
    for (j=0;j<32;j++) {
        setbitu_(buff,i,1,sig_ind[j]?1:0); i+=1;
    }
    /* cell mask */
    for (j=0;j<nsat*nsig&&j<64;j++) {
        setbitu_(buff,i,1,cell_ind[j]?1:0); i+=1;
    }

    /* padding to align 8 bit boundary */
    for (;i%8;i++) {
        setbitu_(buff,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((len=i/8)>=3+1024) {
        return 0;
    }
    /* message length without header and parity */
    setbitu_(buff,14,10,len-3);
    
    /* crc-24q */
    crc=crc24q_(buff,len);
    setbitu_(buff,i,24,crc);
    
    return len+3; /* length total (bytes) */
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
extern int decode_type1033_(uint8_t* buff, int len, int* staid, char* antdes, char* antsno, char* rectype, char* recver, char* recsno)
{
    char des[32]="",sno[32]="",rec[32]="",ver[32]="",rsn[32]="";
    int i=24+12,j,n,m,n1,n2,n3,setup;
    
    n =getbitu_(buff,i+12,8);
    m =getbitu_(buff,i+28+8*n,8);
    n1=getbitu_(buff,i+36+8*(n+m),8);
    n2=getbitu_(buff,i+44+8*(n+m+n1),8);
    n3=getbitu_(buff,i+52+8*(n+m+n1+n2),8);
    
    if (i+60+8*(n+m+n1+n2+n3)<=len*8) {
        *staid=getbitu_(buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) { 
            des[j]=(char)getbitu_(buff,i,8); i+=8;
        }
        setup=getbitu_(buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu_(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n1&&j<31;j++) {
            rec[j]=(char)getbitu_(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n2&&j<31;j++) {
            ver[j]=(char)getbitu_(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n3&&j<31;j++) {
            rsn[j]=(char)getbitu_(buff,i,8); i+=8;
        }
    }
    else {
        return -1;
    }
    
    strncpy(antdes, des,n ); antdes [n] ='\0';
    strncpy(antsno, sno,m ); antsno [m] ='\0';
    strncpy(rectype,rec,n1); rectype[n1]='\0';
    strncpy(recver, ver,n2); recver [n2]='\0';
    strncpy(recsno, rsn,n3); recsno [n3]='\0';
    
    //printf("rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n",des,sno,rec,ver,rsn);
    return 5;
}

/* decode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
extern int decode_type1230_(uint8_t* buff, int len, int* staid, int* glo_cp_align, double* glo_cp_bias)
{
    /* GLONASS code-phase alignment (0:no,1:yes) */
    /* GLONASS code-phase biases {1C,1P,2C,2P} (m) */

    int i=24+12,j,align,mask,bias;
    
    if (i+20>=len*8) {
        return -1;
    }
    *staid=getbitu_(buff,i,12); i+=12;
    align =getbitu_(buff,i, 1); i+= 1+3;
    mask  =getbitu_(buff,i, 4); i+= 4;
    
    *glo_cp_align=align;
    for (j=0;j<4;j++) {
        glo_cp_bias[j]=0.0;
    }
    for (j=0;j<4&&i+16<=len*8;j++) {
        if (!(mask&(1<<(3-j)))) continue;
        bias=getbits_(buff,i,16); i+=16;
        if (bias!=-32768) {
            glo_cp_bias[j]=bias*0.02;
        }
    }
    return 5;
}
/* decode type 1029: UNICODE text string -------------------------------------*/
extern int decode_type1029_(uint8_t* buff, int len, int* staid, char* msg)
{
    int i=24+12,j,mjd,tod,nchar,cunit;
    
    if (i+60<=len*8) {
        *staid=getbitu_(buff,i,12); i+=12;
        mjd   =getbitu_(buff,i,16); i+=16;
        tod   =getbitu_(buff,i,17); i+=17;
        nchar =getbitu_(buff,i, 7); i+= 7;
        cunit =getbitu_(buff,i, 8); i+= 8;
    }
    else {
        //trace(2,"rtcm3 1029 length error: len=%d\n",len);
        return -1;
    }
    if (i+nchar*8>len*8) {
        //trace(2,"rtcm3 1029 length error: len=%d nchar=%d\n",len,nchar);
        return -1;
    } 
    for (j=0;j<nchar&&j<126;j++) {
        msg[j]=getbitu_(buff,i,8); i+=8;
    }
    msg[j]='\0';
    
    return nchar;
}
   int week_number(double sec)
{
    return (int)(sec/(24*7*3600));
}
double week_second(double sec)
{
    return sec-week_number(sec)*(24*7*3600);
}
