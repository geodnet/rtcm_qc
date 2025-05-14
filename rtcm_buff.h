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
    char antdes[32];
    char antsno[32];
    char rectype[32];
    char recver[32];
    char recsno[32];
    int glo_cp_align;
    double glo_cp_bias[4];
    double tow;
    double pos[3];
	int sync;
    int crc;
    int misorder;
    int slen;
    unsigned char sys;         /* update satellite of ephemeris */
    unsigned char prn;
    unsigned long long numofbyte;   /* total number of bytes*/
    unsigned long long numofmsg;    /* total number of msg*/
    unsigned long long numofcrc;    /* total number of crc failure */
    unsigned long long numofepo;    /* total number of epoch */
    unsigned long long numofsync;   /* total number of sync flag = 0 (no more message in the same epoch) */
    unsigned long long numofmissync;/* misorder with sync flag */
    unsigned long long numofmistime;/* misorder with time tag  */
    unsigned long long numofmsg_obs;/* total number of observation message */
    unsigned long long numofmsg_eph;/* total number of eph message */
    double stime;   /* start time */
    double etime;   /* start time */
    int pre_obscount;   /* obs message count in one epoch */
    int cur_obscount;
    double rcv_gps_sec; /* received time in gps sec */
    double dt;
} rtcm_buff_t;

int input_rtcm3_type(rtcm_buff_t* rtcm, unsigned char data, int fix_sync);

int decode_type1005_(unsigned char* buff, int len, int* staid, double* pos);
int decode_type1006_(unsigned char* buff, int len, int* staid, double* pos);
int update_type_1005_1006_pos(uint8_t* buff, int nbyte, double* p);
int decode_type1033_(uint8_t* buff, int len, int* staid, char* antdes, char* antsno, char* rectype, char* recver, char* recsno);
int decode_type1230_(uint8_t* buff, int len, int* staid, int* glo_cp_align, double* glo_cp_bias);
int update_msm_sync_(uint8_t* buff, int len, int sync);
int decode_type1029_(uint8_t* buff, int len, int* staid, char* msg);

unsigned int crc24q_(const unsigned char *buff, int len);
void setbitu_(unsigned char *buff, int pos, int len, unsigned int data);
void setbits_(uint8_t *buff, int pos, int len, int32_t data);
void set38bits_(uint8_t *buff, int pos, double value);
unsigned int getbitu_(const unsigned char *buff, int pos, int len);
int getbits_(const unsigned char *buff, int pos, int len);
double getbits_38_(const unsigned char *buff, int pos);

int rtcm_obs_type(int type);
int rtcm_eph_type(int type);

   int week_number(double sec);
double week_second(double sec);

#ifdef __cplusplus
}
#endif

#endif
