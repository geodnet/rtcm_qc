#ifndef _RTCM_BUFF_H
#define _RTCM_BUFF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MAX_RTCM_BUF_LEN 1200

typedef struct {        /* RTCM control struct type */
    int staid;          /* station id */
    int type;
    int nbyte;          /* number of bytes in message buffer */ 
    int nbit;           /* number of bits in word buffer */ 
    int len;            /* message length (bytes) */
    unsigned char nsat; /* number of satellites */
    unsigned char nsig; /* number of signals */
    unsigned char ncel; /* number of cell */
    unsigned char sats[64];	/* satellites */
    unsigned char sigs[32];	/* signals */
    unsigned char cels[64];	/* cell mask */
	double r[64];
	double rr[64];
	double pr[64];
	double cp[64];
	double rrf[64];
	double cnr[64];
	int lock[64];
	int ex[64];
	int half[64];
    unsigned char buff[MAX_RTCM_BUF_LEN]; /* message buffer */
    unsigned short wk;
    char staname[5];
    char antdes[32];
    char antsno[32];
    char rectype[32];
    char recver[32];
    char recsno[32];
    double tow;
    double pos[3];
	int sync;
    int crc;
    int slen;
    unsigned char sys;         /* update satellite of ephemeris */
    unsigned char prn;
} rtcm_buff_t;

int input_rtcm3_type(rtcm_buff_t* rtcm, unsigned char data);

int decode_type1005_(unsigned char* buff, int len, int* staid, double* pos);
int decode_type1006_(unsigned char* buff, int len, int* staid, double* pos);
int update_type_1005_1006_pos(uint8_t* buff, int nbyte, double* p);
int decode_type1033_(uint8_t* buff, int len, char *staname, char* antdes, char* antsno, char* rectype, char* recver, char* recsno);

unsigned int crc24q_(const unsigned char *buff, int len);
void setbitu_(unsigned char *buff, int pos, int len, unsigned int data);
void setbits_(uint8_t *buff, int pos, int len, int32_t data);
void set38bits_(uint8_t *buff, int pos, double value);
unsigned int getbitu_(const unsigned char *buff, int pos, int len);
int getbits_(const unsigned char *buff, int pos, int len);
double getbits_38_(const unsigned char *buff, int pos);

int rtcm_obs_type(int type);
int rtcm_eph_type(int type);

#ifdef __cplusplus
}
#endif

#endif
