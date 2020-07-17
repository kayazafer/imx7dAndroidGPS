/*
 * Copyright © 2015 Denys Petrovnin <dipcore@gmail.com>
   zaferkaya1960@hotmail.com
 */

#include "gps.h"


static void gps_dev_send(int fd, char *msg)
{
    int i, n, ret;

    i = strlen(msg);

    n = 0;

    do {

//        ret = write(fd, msg + n, i - n);
        ret = write(fd, msg + n, 1);

        if (ret < 0 && errno == EINTR) {
            continue;
        }
        usleep(100);

        n += ret;

    } while (n < i);

    usleep(250000);

    DFR("GPS sent to device: %s", msg);
}

void gps_dev_power(int fd, int mode)
{
//    DFR("%s %d",__FUNCTION__,state);
  
    char buff[50];
    int i;

    if (mode == 0)
	sprintf(buff, "$PMTK161,0*28\r\n"); // soft shutdown
    else 
    if (mode == 1)
	sprintf(buff, "\r\n");		    // soft powerup
    else 
    if (mode == 2)
	sprintf(buff, "$PMTK225,0*2B\r\n"); // normal mode
    else 
    if (mode == 3)
	sprintf(buff, "$PMTK225,8*23\r\n"); // standby
    else
    if (mode == 4)
	sprintf(buff, "$PMTK104*37\r\n"); // reset
    else
    if (mode == 5)
	sprintf(buff, "$PMTK161,0*28\r\n"); // stop mode
    else
	return;

    gps_dev_send(fd, buff);
    
  
    return;
}

static unsigned char gps_dev_calc_nmea_csum(char *msg)
{
    unsigned char csum = 0;
    int i;

    for (i = 1; msg[i] != '*'; ++i) {
        csum ^= msg[i];
    }

    return csum;
}


void gps_dev_set_baud_rate(int fd, int baud)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK251,%d*", baud);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

void gps_dev_set_nav_thres(int fd)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK386,0.5*");

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

void gps_dev_set_DT_UTC(int fd, int year,int month,int day,int hour,int min,int sec)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK740,%04d,%02d,%02d,%02d,%02d,%02d*", year,month,day,hour,min,sec);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

void gps_dev_set_reference_loc(int fd, double Lat, double Long, double Alt, int year,int month,int day,int hour,int min,int sec)
{
    char buff[200];
    int i;

    sprintf(buff, "$PMTK741,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%d*",Lat,Long,Alt,year,month,day,hour,min,sec);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}



void gps_dev_set_easy(int fd, int mode)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK869,1,%d*", mode);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

void gps_dev_set_aic(int fd, int mode)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK286,%d*", mode);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

void gps_dev_set_mode(int fd, int modeGPS, int modeGNS, int modeGAL, int modeBEI)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK353,%d,%d,%d,0,%d*", modeGPS,modeGNS,modeGAL,modeBEI);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

//$PTWSVER,TELIT,V13-2.3.0-STD-5.1.5-N96-000200*78
void gps_dev_get_ver(int fd)
{
    char buff[50];
    int i;
/*
    sprintf(buff, "$PMTK605*31\r\n");
    gps_dev_send(fd, buff);
*/
    sprintf(buff, "$PTWSVER,GET,TELIT*57\r\n");
    gps_dev_send(fd, buff);
/*
    sprintf(buff, "$PTWS,VERSION,GET*0C\r\n");
    gps_dev_send(fd, buff);
*/
}
 

/*
static void gps_dev_set_message_rate(int fd, int rate)
{

    unsigned int i;

    char *msg[] = {
                     "GGA", "GLL", "VTG",
                     "GSA", "GSV", "RMC"
                  };

    for (i = 0; i < sizeof(msg)/sizeof(msg[0]); ++i) {
        gps_dev_set_nmea_message_rate(fd, msg[i], rate);
    }

    return;
}
*/

static void gps_dev_set_message_rate(int fd, int rate)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK220,%d*", rate);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

static void gps_dev_set_solution_pri(int fd, int pri)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK257,%d*", pri);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);
}

static void gps_dev_set_fix_ctl(int fd, int rate)
{
    char buff[50];
    int i;
 
    sprintf(buff, "$PMTK500,%d,0,0,0,0*", rate);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

}

//0.GLL 1.RMC 2.VTG 3.GGA 4.GSA 5.GSV 17.ZDA
static void gps_dev_start_out(int fd)
{
    char buff[100];
    int i;

    sprintf(buff, "$PMTK314,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); ///< turn on ALL THE DATA

    gps_dev_send(fd, buff);

}

static void gps_dev_stop_out(int fd)
{
    char buff[100];

    sprintf(buff, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); ///< turn off output

    gps_dev_send(fd, buff);

}

static void gps_dev_hot_restart(int fd)
{
    char buff[100];

    sprintf(buff, "$PMTK101*32\r\n");

    gps_dev_send(fd, buff);

}

static void gps_dev_warm_restart(int fd)
{
    char buff[100];

    sprintf(buff, "$PMTK102*31\r\n");

    gps_dev_send(fd, buff);

}

static void gps_dev_cold_restart(int fd)
{
    char buff[100];

    sprintf(buff, "$PMTK103*30\r\n");

    gps_dev_send(fd, buff);

}

static void gps_dev_reset(int fd)
{
    char buff[100];

    sprintf(buff, "$PMTK104*37\r\n");

    gps_dev_send(fd, buff);

}

static void gps_dev_enb_epe(int fd)
{
    char buff[100];
    int i;
/*
    sprintf(buff,"$PTWSANT,OUTPUT,SET,NONE*");
    i = strlen(buff);
    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));
    gps_dev_send(fd, buff);

    sprintf(buff,"$PTWS,ANT,OUTPUT,SET,NONE*");
    i = strlen(buff);
    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));
    gps_dev_send(fd, buff);

    sprintf(buff,"$PTWS,LNA,GAIN,SET,1*");
    i = strlen(buff);
    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));
    gps_dev_send(fd, buff);

    sprintf(buff,"$PTWSLNA,GAIN,SET,1*");
    i = strlen(buff);
    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));
    gps_dev_send(fd, buff);

    sprintf(buff,"$PTWSEPE,SET,ON*");
    i = strlen(buff);
    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));
    gps_dev_send(fd, buff);
*/
    sprintf(buff,"$PTWSEPE,ENABLE*7D\r\n");
    gps_dev_send(fd, buff);

}

static void gps_dev_dis_epe(int fd)
{
    char buff[100];

//    sprintf(buff,"$PTWSEPE,SET,OFF\r\n");
//    gps_dev_send(fd, buff);

    sprintf(buff,"$PTWSEPE,DISABLE*28\r\n");
    gps_dev_send(fd, buff);

}

static void gps_dev_set_dgps_mode(int fd, int mode)
{
    char buff[50];
    int i;

    sprintf(buff, "$PMTK301,%d*", mode);

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);
}

static void gps_dev_enb_sbas(int fd)
{
    char buff[50];

    sprintf(buff, "$PMTK313,1*2E\r\n");
    gps_dev_send(fd, buff);
}

static void gps_dev_set_lna(int fd)
{
    char buff[50];
    int i;

    sprintf(buff, "$PTWSLNA,GAIN,SET,HIGH*");

    i = strlen(buff);

    sprintf((buff + i), "%02X\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);
}

void gps_dev_init(int fd){
    DFR("GPS dev start init");
    gps_dev_power(fd, 1);
    gps_dev_power(fd, 2);

//    gps_dev_hot_restart(fd);
//    gps_dev_warm_restart(fd);
    gps_dev_cold_restart(fd);

    gps_dev_set_mode(fd,1,1,1,0);

    gps_dev_set_solution_pri(fd,0);

    gps_dev_set_dgps_mode(fd,2);

    gps_dev_enb_sbas(fd);

    gps_dev_set_easy(fd, 1);

    gps_dev_set_aic(fd, 1);

    gps_dev_set_message_rate(fd, 1000);

    gps_dev_set_lna(fd);

    gps_dev_get_ver(fd);

}

void gps_dev_deinit(int fd){
    DFR("GPS dev start deinit");
//    gps_dev_power(fd, 1);
//    gps_dev_power(fd, 0);
//    gps_dev_power(fd, 2);
    gps_dev_power(fd, 1);
    gps_dev_power(fd, 2);
    gps_dev_power(fd, 5);

}

void gps_dev_start(int fd){

//    gps_dev_power(fd, 1);
//    gps_dev_power(fd, 2);

    gps_dev_start_out(fd);

    gps_dev_enb_epe(fd);

    DFR("GPS dev start initiated");
}

void gps_dev_stop(int fd){

    gps_dev_dis_epe(fd);

    gps_dev_stop_out(fd);
//    gps_dev_set_message_rate(fd, 10000);
//    gps_dev_power(fd, 1);
//    gps_dev_power(fd, 2);
//    gps_dev_power(fd, 3);

    DFR("GPS dev stop initiated");
}

