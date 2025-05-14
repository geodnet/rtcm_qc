// test_rtcm.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <string.h>
#include "rtcm_buff.h"

typedef struct
{
    int type;
    double time;
    unsigned long numofepoch;
}rtcm_obs_t;

typedef struct
{
    double x;
    double y;
    double z;
}xyz_t;

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef D2R
#define D2R (PI/180.0)
#endif

#ifndef R2D
#define R2D (180.0/PI)
#endif


double lat2local(double lat, double* lat2north)
{
    double f_WGS84 = (1.0 / finv_WGS84);
    double e2WGS84 = (2.0 * f_WGS84 - f_WGS84 * f_WGS84);
    double slat = sin(lat);
    double clat = cos(lat);
    double one_e2_slat2 = 1.0 - e2WGS84 * slat * slat;
    double Rn = ae_WGS84 / sqrt(one_e2_slat2);
    double Rm = Rn * (1.0 - e2WGS84) / (one_e2_slat2);
    *lat2north = Rm;
    return Rn * clat;
}


extern void xyz2blh_(const double* xyz, double* blh)
{
    // ecef xyz => blh
    double a = ae_WGS84, finv = finv_WGS84;
    double f = 1.0 / finv, e2 = 2 * f - f * f;
    double x = xyz[0], y = xyz[1], z = xyz[2], lat, lon, ht;
    double R = sqrt(x * x + y * y + z * z);
    double ang = 0.0;
    double lat1 = 0.0;
    double Rw = 0.0;
    double Rn = 0.0;
    if (fabs(z) < 1.0e-5)
    {
        lat = 0.0;
    }
    else
    {
        ang = atan(fabs(z / sqrt(x * x + y * y))) * ((z < 0.0) ? -1.0 : 1.0);
        //if (z<0.0) ang = -ang;
        lat1 = ang;
        Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
        Rn = 0.0;
        lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
        if (z < 0.0) lat = -lat;
        while (fabs(lat - lat1) > 1e-12)
        {
            lat1 = lat;
            Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
            lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
            if (z < 0.0) lat = -lat;
        }
        if (lat > PI) lat = lat - 2.0 * PI;
    }
    if (fabs(x) < 1e-5)
    {
        if (y >= 0.0)
            lon = PI / 2.0;
        else
            lon = 3.0 * PI / 2.0;
    }
    else
    {
        lon = atan(fabs(y / x));
        if (x > 0.0)
        {
            if (y >= 0.0)
                lon = lon;
            else
                lon = 2.0 * PI - lon;
        }
        else
        {
            if (y >= 0.0)
                lon = PI - lon;
            else
                lon = PI + lon;
        }
    }
    Rw = sqrt(1 - e2 * sin(lat) * sin(lat));
    Rn = a / Rw;
    ht = R * cos(ang) / cos(lat) - Rn;
    if (lon > PI) lon = lon - 2.0 * PI;
    blh[0] = lat;
    blh[1] = lon;
    blh[2] = ht;
    return;
}

void blh2xyz_(const double* blh, double* xyz)
{
    // lat, lon, ht => ecef xyz
    double a = ae_WGS84, finv = finv_WGS84;
    double f = 1.0 / finv, e2 = 2 * f - f * f;
    double lat = blh[0], lon = blh[1], ht = blh[2];
    double Rw = sqrt(1 - e2 * sin(lat) * sin(lat));
    double Rn = a / Rw;
    xyz[0] = (Rn + ht) * cos(lat) * cos(lon);
    xyz[1] = (Rn + ht) * cos(lat) * sin(lon);
    xyz[2] = (Rn * (1 - e2) + ht) * sin(lat);
    return;
}

void deg2dms(double deg, double* dms)
{
    double sign = deg < 0.0 ? -1.0 : 1.0;
    double a = fabs(deg);

    dms[0] = floor(a); a = (a - dms[0]) * 60.0;
    dms[1] = floor(a); a = (a - dms[1]) * 60.0;
    dms[2] = a;

    dms[0] *= sign;
}

/* output solution in the form of nmea GGA sentence */
int outnmea_gga(unsigned char* buff, double time, int type, double lat_deg, double lon_deg, double alt, int ns, double dop, double age)
{
    double h, ep[6], dms1[3], dms2[3];
    char* p = (char*)buff, * q, sum;

    if (type != 1 && type != 4 && type != 5) {
        p += sprintf(p, "$GPGGA,,,,,,,,,,,,,,");
        for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
        p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
        return((int)(p - (char*)buff));
    }
    time -= 18.0;
    ep[2] = floor(time / (24 * 3600));
    time -= ep[2] * 24 * 3600.0;
    ep[3] = floor(time / 3600);
    time -= ep[3] * 3600;
    ep[4] = floor(time / 60);
    time -= ep[4] * 60;
    ep[5] = time;

    h = 0.0;

    //
    deg2dms(fabs(lat_deg), dms1);
    deg2dms(fabs(lon_deg), dms2);

    //
    p += sprintf(p, "$GPGGA,%02.0f%02.0f%08.5f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f",
        ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, lat_deg >= 0 ? "N" : "S",
        dms2[0], dms2[1] + dms2[2] / 60.0, lon_deg >= 0 ? "E" : "W", type,
        ns, dop, alt - h, h, age);

    // Compute checksum
    for (q = (char*)buff + 1, sum = 0; *q; q++) {
        sum ^= *q; /* check-sum */
    }

    //
    p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);

    return((int)(p - (char*)buff));
}

FILE* set_output_file(const char* fname, const char* key, int is_binary)
{
    char filename[255] = { 0 }, outfilename[255] = { 0 };
    strcpy(filename, fname);
    char* temp = strrchr(filename, '.');
    if (temp) temp[0] = '\0';
    sprintf(outfilename, "%s-%s", filename, key);
    return is_binary ? fopen(outfilename, "wb") : fopen(outfilename, "w");
}

typedef struct
{
    int type;
    int nsig;
    unsigned long count;
}sig_t;

static double xyz_ref[3] = { 0 };

static void test_rtcm(const char* fname, int opt)
{
    FILE* fRTCM = fopen(fname, "rb");

    if (fRTCM==NULL) return;

    FILE* fGGA = (opt & 1 << 0) ? set_output_file(fname, "-rtcm.nmea", 0) : NULL;
    FILE* fOUT = (opt & 1 << 1) ? set_output_file(fname, "-out.rtcm3", 1) : NULL;
    FILE* fLOG = (opt & 1 << 2) ? set_output_file(fname, "-msg.csv", 0) : NULL;
    FILE* fDIF = (opt & 1 << 3) ? set_output_file(fname, "-coord-diff.csv", 0) : NULL;

    int data = 0;
    rtcm_buff_t gRTCM_Buf = { 0 };
    rtcm_buff_t* rtcm = &gRTCM_Buf;
    unsigned long numofsync = 0;
    unsigned long numofmisorder = 0;
    unsigned long numofcrc = 0;
    unsigned long numofmsg = 0;
    unsigned long numofepoch = 0;
    unsigned long numofepoch_bad = 0;
    double lastTime = 0.0;
    std::vector< rtcm_obs_t> vObsType;
    std::vector<xyz_t> vxyz;
    std::vector<xyz_t> vxyz_final;

    std::vector<std::string> vDataGapOutput;
    std::vector<std::string> vTypeGapOutput;
    double start_time =-1;
    double end_time = -1;
    double tow = 0;
    double dt = 0;
    int sync_count = 0;
    int ms_time_cur =-1;
    int ms_time_pre =-1;
    std::map<int, int> mObsType;
    int nloc[10] = { 0 };
    int nseg = 0;
    int nlen = 0;
    uint8_t key_buff[100] = { 0 };
    char msg1029[129] = { 0 };

    while (!feof(fRTCM))
    {
        if ((data = fgetc(fRTCM)) == EOF) break;
        int ret = input_rtcm3_type(rtcm, (unsigned char)data, opt & 1 << 4);
        /* check GEOD log file header for the */
        if (nlen >= 100) nlen = 0;
        if (nlen == 0)
        {
            if (data == '$')
            {
                memset(key_buff, 0, sizeof(key_buff));
                memset(nloc, 0, sizeof(nloc));
                nseg = 0;
                key_buff[nlen] = data;
                nlen++;
            }
        }
        else
        {
            key_buff[nlen] = data;
            nlen++;
            if (data == ',')
            {
                nloc[nseg] = nlen;
                nseg++;
                if (nseg == 2)
                {
                    //printf("%s\n", key_buff);
                    if (strstr((char*)key_buff, "$GEOD"))
                    {
                        rtcm->rcv_gps_sec = atof((char*)(key_buff + nloc[0])) / 1000.0 + 18.0 - 315964800; /* 315964800 => gps start time */
                    }
                    nlen = 0;
                    nseg = 0;
                }
            }
        }
        /* process regular rtcm data */
        if (rtcm->crc) continue;
        if (rtcm->len>0 && rtcm->nbyte==0) /* rtcm data */
        {
            if (rtcm->rcv_gps_sec > 0)
            {
                dt = week_second(rtcm->rcv_gps_sec) - rtcm->tow;
                if (dt < -7 * 24 * 1800) dt += 7 * 24 * 3600;
                if (dt > 7 * 24 * 1800) dt -= 7 * 24 * 3600;
                if (fLOG) fprintf(fLOG, "%10.3f,%4i,%i,%i,%2i,%2i%c,%7.3f%s\n", rtcm->tow, rtcm->type, rtcm->sync, ret, rtcm->cur_obscount, rtcm->pre_obscount, rtcm->misorder ? '*' : ' ', dt, dt < 0.001 ? "++" : "  ");
            }
            else
            {
                if (fLOG) fprintf(fLOG, "%10.3f,%4i,%i,%i,%2i,%2i%c\n", rtcm->tow, rtcm->type, rtcm->sync, ret, rtcm->cur_obscount, rtcm->pre_obscount, rtcm->misorder ? '*' : ' ');
            }
            if (rtcm->type == 1029 && decode_type1029_(rtcm->buff, rtcm->len + 3, &rtcm->staid, msg1029))
            {
                if (fLOG) fprintf(fLOG, "%s\n", msg1029);
            }
            if (ret == 1)
            {
                if (numofepoch > 0)
                {
                    if (rtcm->dt > 1.5 && rtcm_obs_type(rtcm->type))
                    {
                        char temp[255] = { 0 };
                        sprintf(temp, "%10.3f,%10.3f,%6lu\r\n", rtcm->tow, rtcm->dt, numofepoch);
                        vDataGapOutput.push_back(temp);
                    }
                    if (rtcm->dt < 0.8)
                    {
                        ++numofepoch_bad;
                    }
                    else
                    {
                        ++numofepoch;
                    }
                }
                else
                {
                    ++numofepoch;
                }
                lastTime = rtcm->tow;
            }
            if (ret == 5)
            {
                xyz_t xyz = { 0 };
                if ((rtcm->pos[0] * rtcm->pos[0] + rtcm->pos[1] * rtcm->pos[1] + rtcm->pos[2] * rtcm->pos[2]) < 0.1)
                {
                }
                else
                {
                    xyz.x = rtcm->pos[0];
                    xyz.y = rtcm->pos[1];
                    xyz.z = rtcm->pos[2];
                    vxyz.push_back(xyz);
                    if (fGGA)
                    {
                        char gga[255] = { 0 };
                        double blh[3] = { 0 };
                        xyz2blh_(rtcm->pos, blh);
                        int blen = sprintf(gga, "%04i,%14.4f,%14.4f,%14.4f,", rtcm->staid, rtcm->pos[0], rtcm->pos[1], rtcm->pos[2]);
                        outnmea_gga((unsigned char*)gga+blen, rtcm->tow, 1, blh[0] * R2D, blh[1] * R2D, blh[2], 10, 1.0, 0.0);
                        fprintf(fGGA, "%s", gga);
                    }
                    int is_added = 0;
                    if (vxyz_final.size() == 0)
                    {
                        is_added = 1;
                    }
                    else
                    {
                        double dxyz[3] = { xyz.x - vxyz_final.rbegin()->x, xyz.y - vxyz_final.rbegin()->y, xyz.z - vxyz_final.rbegin()->z };
                        if (fabs(dxyz[0]) > 0.001 || fabs(dxyz[1]) > 0.001 || fabs(dxyz[2]) > 0.001)
                        {
                            is_added = 1;
                            if (fOUT)
                            {
                                //fclose(fOUT);
                                char buffer[255] = { 0 };
                                sprintf(buffer, "-out%03i.rtcm3", (int)vxyz_final.size());
                                //fOUT = set_output_file(fname, buffer, 1);
                            }
                        }
                    }
                    if (is_added)
                    {
                        //printf("%14.4f%14.4f%14.4f\n", xyz.x, xyz.y, xyz.z);
                        vxyz_final.push_back(xyz);
                    }
                }
            }
            /* output */
            if (fOUT) fwrite(rtcm->buff, rtcm->len + 3, sizeof(char), fOUT);

            int i = 0;
            for (; i < vObsType.size(); ++i)
            {
                if (vObsType[i].type == rtcm->type)
                {
                    if (vObsType[i].numofepoch > 0)
                    {
                        dt = tow - vObsType[i].time;
                        if (dt > 1.5 && rtcm_obs_type(rtcm->type))
                        {
                            char temp[255] = { 0 };
                            sprintf(temp, "%10.3f,%10.3f,%6lu,%4i\r\n", tow, dt, vObsType[i].numofepoch, rtcm->type);
                            vTypeGapOutput.push_back(temp);
                        }
                    }
                    vObsType[i].numofepoch++;
                    vObsType[i].time = tow;
                    break;
                }
            }
            if (i == vObsType.size())
            {
                rtcm_obs_t temp = { 0 };
                temp.type = rtcm->type;
                temp.time = tow;
                temp.numofepoch = 1;
                vObsType.push_back(temp);
            }
        }
    }
    if (end_time < start_time) end_time += 7 * 24 * 3600;
    int total_epoch = (int)(end_time - start_time + 1);
    if (vObsType.size() > 0)
    {
        if (fLOG) fprintf(fLOG, "RTCM TYPE Count\n");
        for (int i = 0; i < vObsType.size(); ++i)
        {
            if (fLOG) fprintf(fLOG, "%4i,%6lu,%6lu\r\n", vObsType[i].type, vObsType[i].numofepoch, numofepoch);
        }
    }
    if (fLOG) fprintf(fLOG, "%6llu, failed in CRC check\r\n", rtcm->numofcrc);
    if (fLOG) fprintf(fLOG, "%6llu, sync count difference from previous epoch\r\n", rtcm->numofmissync);
    if (fLOG) fprintf(fLOG, "%6llu, misorder messages\r\n", rtcm->numofmistime);
    char* temp = strchr(rtcm->rectype, '\n'); if (temp) temp[0] = '\0';
    temp = strchr(rtcm->rectype, '\r'); if (temp) temp[0] = '\0';
    printf("%6llu, %6llu, %6llu, %6llu, %6llu, %7.2f, %7.2f, %7.2f, %i, %32s, %32s, %s\n", rtcm->numofepo, rtcm->numofmsg, rtcm->numofcrc, rtcm->numofmissync, rtcm->numofmistime, rtcm->numofmsg > 0 ? (rtcm->numofcrc * 100.0) / rtcm->numofmsg : 0, rtcm->numofepo > 0 ? (rtcm->numofmissync * 100.0) / rtcm->numofepo : 0, rtcm->numofepo > 0 ? (rtcm->numofmistime * 100.0) / rtcm->numofepo : 0, strstr(rtcm->rectype, "-U") ? 1 : 0, rtcm->recver, rtcm->rectype, fname);
    if (vxyz.size() > 0)
    {
        double midXYZ[3] = { 0 };
        for (int i = 0; i < vxyz.size(); ++i)
        {
            midXYZ[0] += vxyz[i].x;
            midXYZ[1] += vxyz[i].y;
            midXYZ[2] += vxyz[i].z;
        }
        midXYZ[0] /= vxyz.size();
        midXYZ[1] /= vxyz.size();
        midXYZ[2] /= vxyz.size();
        double stdXYZ[3] = { 0 };
        double midBLH[3] = { 0 };
        xyz2blh_(midXYZ, midBLH);
        if (vxyz.size() > 1)
        {
            for (int i = 0; i < vxyz.size(); ++i)
            {
                stdXYZ[0] += (vxyz[i].x - midXYZ[0]) * (vxyz[i].x - midXYZ[0]);
                stdXYZ[1] += (vxyz[i].y - midXYZ[1]) * (vxyz[i].y - midXYZ[1]);
                stdXYZ[2] += (vxyz[i].z - midXYZ[2]) * (vxyz[i].z - midXYZ[2]);
            }
            stdXYZ[0] = sqrt(stdXYZ[0] / (vxyz.size() - 1));
            stdXYZ[1] = sqrt(stdXYZ[1] / (vxyz.size() - 1));
            stdXYZ[2] = sqrt(stdXYZ[2] / (vxyz.size() - 1));
            if (fLOG) fprintf(fLOG, "%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%i\n", midBLH[0] * 180 / PI, midBLH[1] * 180 / PI, midBLH[2], midXYZ[0], midXYZ[1], midXYZ[2], stdXYZ[0], stdXYZ[1], stdXYZ[2], (int)vxyz.size());
        }
        else
        {
            if (fLOG) fprintf(fLOG, "%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%i\n", midBLH[0]*180/PI, midBLH[1] * 180 / PI, midBLH[2], midXYZ[0], midXYZ[1], midXYZ[2], (int)vxyz.size());
        }
        if (fabs(xyz_ref[0]) > 0.0 || fabs(xyz_ref[1]) > 0.0 || fabs(xyz_ref[2]) > 0.0)
        {
            for (int i = 0; i < vxyz.size(); ++i)
            {
                double dxyz[3] = { vxyz[i].x - xyz_ref[0] , vxyz[i].y - xyz_ref[1] , vxyz[i].z - xyz_ref[2] };
                double dist = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
                if (fDIF) fprintf(fDIF, "%.4f,%.4f,%.4f,%.4f,%i\n", dxyz[0], dxyz[1], dxyz[2], dist, dist > 5.0 ? 1 : 0);
            }
        }
        int ii = 0;
    }
    if (vDataGapOutput.size() > 0)
    {
        if (fLOG) fprintf(fLOG,"Epoch Data GAP\n");
        for (int i = 0; i < vDataGapOutput.size(); ++i)
        {
            if (fLOG) fprintf(fLOG, "%s", vDataGapOutput[i].c_str());
        }
    }
    if (vTypeGapOutput.size() > 0)
    {
        if (fLOG) fprintf(fLOG, "RTCM TYPE GAP\n");
        for (int i = 0; i < vTypeGapOutput.size(); ++i)
        {
            if (fLOG) fprintf(fLOG, "%s", vTypeGapOutput[i].c_str());
        }
    }
    if (fRTCM) fclose(fRTCM);
    if (fGGA) fclose(fGGA);
    if (fOUT) fclose(fOUT);
    if (fLOG) fclose(fLOG);
    if (fDIF) fclose(fDIF);
    return;
}

int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        int opt = 0;
        for (int i = 2; i < argc; ++i)
        {
            char* temp = strstr(argv[i], "=");
            if (temp)
            {
                temp[0] = '\0';
                if (strstr(argv[i], "nmea"))   /* output nmea */
                {
                    int is_nmea = atoi(temp + 1);
                    if (is_nmea) opt |= 1 << 0;
                }
                else if (strstr(argv[i], "rtcm"))   /* output rtcm */
                {
                    int is_rtcm = atoi(temp + 1);
                    if (is_rtcm) opt |= 1 << 1;
                }
                else if (strstr(argv[i], "msg"))   /* output msg log */
                {
                    int is_msg = atoi(temp + 1);
                    if (is_msg) opt |= 1 << 2;
                }
                else if (strstr(argv[i], "xyz")) /* output coordinate difference */
                {
                    int num = sscanf(temp + 1, "%lf,%lf,%lf", xyz_ref + 0, xyz_ref + 1, xyz_ref + 2);
                    if (num == 3)
                    {
                        opt |= 1 << 3;
                    }
                }
                else if (strstr(argv[i], "sync")) /* fix sync */
                {
                    int is_sync = atoi(temp + 1);
                    if (is_sync) opt |= 1 << 4;
                }
            }
        }
        test_rtcm(argv[1], opt);
    }
    return 0;
}

