/*
 * Copyright © 2015 Denys Petrovnin <dipcore@gmail.com>

   zaferkaya1960@hotmail.com

 */

void gps_dev_init(int fd);
void gps_dev_deinit(int fd);
void gps_dev_start(int fd);
void gps_dev_stop(int fd);
void gps_dev_set_baud_rate(int fd, int baud);
void gps_dev_set_mode(int fd, int modeGPS, int modeGNS, int modeGAL, int modeBEI);
void gps_dev_set_nav_thres(int fd);
void gps_dev_set_DT_UTC(int fd, int year,int month,int day,int hour,int min,int sec);
void gps_dev_set_reference_loc(int fd, double Lat, double Long, double Alt, int year,int month,int day,int hour,int min,int sec);
void gps_dev_power(int fd, int state);
