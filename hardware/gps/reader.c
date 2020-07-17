/*
 * Copyright © 2015 Denys Petrovnin <dipcore@gmail.com>

   zaferkaya1960@hotmail.com

 */

#include"gps.h"

void notifier_svs_append(char talker[3], int prn, float elevation, float azimuth, float snr);
void notifier_svs_update_status();
void notifier_svs_used_ids(talker[3], int ids[12]);

void notifier_set_speed(float speed_knots);
void notifier_set_bearing(float bearing);
void notifier_set_latlong(double lat, double lon);
void notifier_set_altitude(double altitude, char units);
void notifier_set_accuracy(float accuracy);
void notifier_set_date_time(struct minmea_date date, struct minmea_time time_);
void notifier_push_location();
void notifier_set_version(char *brand, char *vers);
void update_gps_nmea(const char* data, int len);

static char line[ NMEA_MAX_SIZE + 1 ];
static int pos;
static bool epeflag = false;
static bool gsvflag = false;
static float HDOP = 10;

void nmea_reader_append (char *buff, int size)
{
	if (!size)
	{
		pos = 0;
		return;
	}


	for (int n = 0; n < size; n++)
	{

		if ((buff[n] == '$') || (pos >= NMEA_MAX_SIZE ))
		{
			pos = 0;
		}

		if (pos == 0 && buff[n] != '$')
		{
			continue;
		}


		if (buff[n] == '\r' || buff[n] == '\n')
		{
			line[pos] = '\0';
                     //   DFR("%s",line);
			if (pos > 2) update_gps_nmea(line, pos);
			nmea_reader_parse(line);
			pos = 0;
			continue;
		}

		line[pos++] = (char) buff[n];

	}
}

void nmea_reader_parse(char *line) {
	switch (minmea_sentence_id(line, false)) {
/*
		case MINMEA_SENTENCE_GLL: {
			struct minmea_sentence_gll frame;
			if (minmea_parse_gll(&frame, line)) {

				// update prev status
				notifier_svs_update_status();

				notifier_set_latlong(minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));

				notifier_push_location();
				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}
			}
			else {
				D("$xxGLL sentence is not parsed\n");
			}
		} break;
*/

		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, line)) {

/*				
				DFR("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
						frame.latitude.value, frame.latitude.scale,
						frame.longitude.value, frame.longitude.scale,
						frame.speed.value, frame.speed.scale);
				DFR("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
						minmea_rescale(&frame.latitude, 1000),
						minmea_rescale(&frame.longitude, 1000),
						minmea_rescale(&frame.speed, 1000));
				DFR("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
						minmea_tocoord(&frame.latitude),
						minmea_tocoord(&frame.longitude),
						minmea_tofloat(&frame.speed));
*/
				notifier_set_date_time(frame.date, frame.time);
				notifier_set_latlong(minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));
				notifier_set_speed(minmea_tofloat(&frame.speed));
				notifier_set_bearing(minmea_tofloat(&frame.course));

				notifier_push_location();
				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}
			}
			else {
				D("$xxRMC sentence is not parsed\n");
			}
		} break;
		
		case MINMEA_SENTENCE_GGA: {
			struct minmea_sentence_gga frame;
			char talker[3];
			if (minmea_parse_gga(&frame, line) && minmea_talker_id(talker, line)) {
/*
				DFR("$%sGGA: latitude: %f\n", talker, minmea_tocoord(&frame.latitude));
				DFR("$%sGGA: longitude: %f\n", talker, minmea_tocoord(&frame.longitude));

				DFR("$%sGGA: fix quality: %d\n", talker, frame.fix_quality);
				DFR("$%sGGA: satellites tracked: %d\n", talker, frame.satellites_tracked);
				DFR("$%sGGA: hdop: %f\n", talker, minmea_tofloat(&frame.hdop));
				DFR("$%sGGA: altitude: %f %c\n", talker, minmea_tofloat(&frame.altitude), frame.altitude_units);
				DFR("$%sGGA: height: %f %c\n", talker, minmea_tofloat(&frame.height), frame.height_units);
*/
				if (frame.fix_quality) {
					notifier_set_latlong(minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));
					notifier_set_altitude(minmea_tofloat(&frame.altitude), frame.altitude_units);
					HDOP = minmea_tofloat(&frame.hdop);
					// Use hdop value for now
	 				if (!epeflag) notifier_set_accuracy(HDOP * 10);
				}
				notifier_push_location();
				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}
			}
			else {
				D("$xxGGA sentence is not parsed\n");
			}
		} break;
		
		case MINMEA_SENTENCE_GSA: {
			struct minmea_sentence_gsa frame;
			char talker[3];
			if (minmea_parse_gsa(&frame, line) && minmea_talker_id(talker, line)) {
/*
				DFR("$%sGSA: mode: %c\n", talker, frame.mode);
				DFR("$%sGSA: fix type: %d\n", talker, frame.fix_type);
				DFR("$%sGSA: pdop: %f\n", talker, minmea_tofloat(&frame.pdop));
				DFR("$%sGSA: hdop: %f\n", talker, minmea_tofloat(&frame.hdop));
				DFR("$%sGSA: vdop: %f\n", talker, minmea_tofloat(&frame.vdop));
*/
				notifier_svs_used_ids(talker, frame.sats);
 				if (!epeflag && frame.fix_type > 1) {
					HDOP = minmea_tofloat(&frame.hdop);
					notifier_set_accuracy(HDOP * 10);
				}
				notifier_push_location();
				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}
			}
			else {
				D("$xxGSA sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GSV: {
			struct minmea_sentence_gsv frame;
			char talker[3];
			if ( minmea_parse_gsv(&frame, line) && minmea_talker_id(talker, line) ) {
/*
				D("$%sGSV: message %d of %d\n", talker, frame.msg_nr, frame.total_msgs);
				D("$%sGSV: sattelites in view: %d\n", talker, frame.total_sats);
*/
				for (int i = 0; i < 4; i++) {

					notifier_svs_append(talker, frame.sats[i].nr, frame.sats[i].elevation, frame.sats[i].azimuth, frame.sats[i].snr);
/*
					DFR("$%sGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
						talker,
						frame.sats[i].nr,
						frame.sats[i].elevation,
						frame.sats[i].azimuth,
						frame.sats[i].snr);
*/
				}

				gsvflag = true;
			}
			else {
				D("$xxGSV sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_VTG: {
		   struct minmea_sentence_vtg frame;
		   char talker[3];
		   if (minmea_parse_vtg(&frame, line) && minmea_talker_id(talker, line)) {
/*
				DFR("$%sVTG: true track degrees = %f\n",
					   talker, minmea_tofloat(&frame.true_track_degrees));
				DFR("        magnetic track degrees = %f\n",
					   minmea_tofloat(&frame.magnetic_track_degrees));
				DFR("        speed knots = %f\n",
						minmea_tofloat(&frame.speed_knots));
				DFR("        speed kph = %f\n",
						minmea_tofloat(&frame.speed_kph));
*/
				notifier_set_speed(minmea_tofloat(&frame.speed_knots));
				notifier_set_bearing(minmea_tofloat(&frame.true_track_degrees));

				notifier_push_location();

				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}

		   }
		   else {
				D("$xxVTG sentence is not parsed\n");
		   }


		} break;
	

		case MINMEA_SENTENCE_EPE: {
			struct minmea_sentence_epe frame;
			char talker[3];

			if (minmea_parse_epe(&frame, line) && minmea_talker_id(talker, line)) {

				epeflag = true;

//				DFR("$%sEPE: ehpe: %f\n", talker, minmea_tofloat(&frame.ehpe));
//				DFR("$%sEPE: evpe: %f\n", talker, minmea_tofloat(&frame.evpe));
				
				if (HDOP > 1)
					notifier_set_accuracy(minmea_tofloat(&frame.ehpe) * HDOP);
				else
					notifier_set_accuracy(minmea_tofloat(&frame.ehpe));

				notifier_push_location();
				if (gsvflag) {
					notifier_svs_update_status();
					gsvflag = false;
				}

			}
			else {
				D("$xxEPE sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_VER: {
			struct minmea_sentence_ver frame;
			char talker[3];

			if (minmea_parse_ver(&frame, line) && minmea_talker_id(talker, line)) {

				DFR("$%sVER: version: %s %s\n", talker, &frame.brand, &frame.vers);
				notifier_set_version(&frame.brand, &frame.vers);

			}
			else {
				D("$xxVER sentence is not parsed\n");
			}
		} break;

		case MINMEA_INVALID: {
			DFR("rcved not valid %s",line);
		} break;

		default: {
			DFR("rcved %s",line);
		} break;
	}
}
