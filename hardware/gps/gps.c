/*
 * Copyright © 2015 Denys Petrovnin <dipcore@gmail.com>

   zaferkaya1960@hotmail.com

 */

#include "gps.h"

typedef struct {
    bool                init;
    bool                active;
    bool                version;
    GpsCallbacks        *callbacks;
    pthread_t		main_thread;
    uint32_t            min_interval;
    GpsPositionMode     position_mode;
    GpsPositionRecurrence recurrence;
    int fd;
} GpsState;

GpsState  _gps_state[1] =
{
    {
        .init           = false,
        .active         = false,    /* gps_start will change it to true, gps_stop to false */
        .version        = false,
        .position_mode  = GPS_POSITION_MODE_STANDALONE,
        .min_interval   = 0,
	.fd             = -1,
    },
};

static void* gps_state_thread( void*  arg );

static void gps_state_init( GpsState*  state, GpsCallbacks* callbacks );
static void gps_state_stop( GpsState*  s );
static void gps_state_start( GpsState*  s );
static void gps_state_done( GpsState*  s );

static void update_gps_status(GpsStatusValue val);
static void update_gps_svstatus(GnssSvStatus *val);
static void update_gps_location(GpsLocation *fix);

static int serial_gps_init(GpsCallbacks* callbacks);
static void serial_gps_cleanup(void);
static int serial_gps_start();
static int serial_gps_stop();
static int serial_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty);
static int serial_gps_inject_location(double latitude, double longitude, double altitude, float accuracy);
static void serial_gps_delete_aiding_data(GpsAidingData flags);
static int serial_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence, 
		uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);
static void* serial_gps_get_extension(const char* name);

int max_refresh_rate;

static GpsLocation location = {.size = sizeof(GpsLocation),};
static GnssSvStatus sv_status = {.size = sizeof(GnssSvStatus),};

static int gps_svs_used_ids[32];
static int glonass_svs_used_ids[32];
static int galileo_svs_used_ids[32];
static int beidou_svs_used_ids[32];
static int gps_svs_used_ids_X = 0;
static int glonass_svs_used_ids_X = 0;
static int galileo_svs_used_ids_X = 0;
static int beidou_svs_used_ids_X = 0;

static int sv_counter;

void update_gps_nmea(const char* data, int len);

void notifier_svs_append(char talker[3], int prn, float elevation, float azimuth, float snr);
void notifier_svs_inview(char talker[3], int num_svs);
void notifier_svs_update_status();
void notifier_svs_used_ids(char talker[3], int ids[12]);

void notifier_set_speed(float speed_knots);
void notifier_set_bearing(float bearing);
void notifier_set_latlong(double lat, double lon);
void notifier_set_altitude(double altitude, char units);
void notifier_set_accuracy(float accuracy);
void notifier_set_date_time(struct minmea_date date, struct minmea_time time_);
void notifier_push_location();

/******/
const GpsInterface serialGpsInterface = {
    sizeof(GpsInterface),
    serial_gps_init,
    serial_gps_start,
    serial_gps_stop,
    serial_gps_cleanup,
    serial_gps_inject_time,
    serial_gps_inject_location,
    serial_gps_delete_aiding_data,
    serial_gps_set_position_mode,
    serial_gps_get_extension,
};
/*****/
/*****/
const GpsInterface* gps_get_hardware_interface()
{
    D("GPS dev get_hardware_interface");
    return &serialGpsInterface;
}
/*****/
/*****/
static int open_gps(const struct hw_module_t* module, const char* name, struct hw_device_t** device)
{
    D("GPS dev open_gps");
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps_get_hardware_interface;

    *device = &dev->common;
    return 0;
}
/*****/

/****/
static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};
/****/
/****/
struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 0,
    .version_minor = 1,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "Serial GPS/GLONASS Module",
    .author = "Zafer KAYA <ima.com.tr>",
    .methods = &gps_module_methods,
};
/****/

static void* gps_state_thread( void*  arg )
{
    GpsState* s = _gps_state;
    char buff[4096];

    tcflush(s->fd, TCIFLUSH );
    nmea_reader_append( buff, 0 );

    //DFR("state active");

    DFR("GPS thread running");

    for (;;)
    {
        if (s->active == false) break;

	int ret = read( s->fd, buff, sizeof(buff) );

        if (ret > 0)
        {
           nmea_reader_append( buff, ret );
        }
	else if (ret < 0)
	{
           DFR("Error while reading from GPS socket: %s:", strerror(errno));
           break;
        }

    }
exit:
    DFR("GPS thread EXIT");

    return NULL;
}

static int serial_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;
    char buff[1024];
    struct termios  ios;

    if (s->init)
    {
        DFR("Gps already initialized");
        return 0;
    }

    s->fd = open("/dev/ttymxc0", O_RDWR);

    if (s->fd < 0)
    {
        DFR("could not open gps serial device: %s", strerror(errno) );
        return -1;
    }

    //DFR("gps serial device opened ");

	tcgetattr(s->fd, &ios);	/* Get the current attributes of the Serial port */

	ios.c_iflag = 0;
	ios.c_oflag = 0;
	ios.c_cflag = CS8 | CLOCAL | CREAD;
	ios.c_lflag = 0;

	/* Setting Time outs */
	ios.c_cc[VMIN] = 0; 
	ios.c_cc[VTIME] = 50; 

	cfsetispeed(&ios,B9600); 
	cfsetospeed(&ios,B9600); 
        tcsetattr(s->fd, TCSANOW, &ios );
//        gps_dev_power(s->fd,4);
        usleep(100000);
	gps_dev_set_baud_rate(s->fd, 115200);
        usleep(100000);

	cfsetispeed(&ios,B19200); 
	cfsetospeed(&ios,B19200); 
        tcsetattr(s->fd, TCSANOW, &ios );
//        gps_dev_power(s->fd,4);
        usleep(100000);
	gps_dev_set_baud_rate(s->fd, 115200);
        usleep(100000);

	cfsetispeed(&ios,B921600); 
	cfsetospeed(&ios,B921600); 
        tcsetattr(s->fd, TCSANOW, &ios );
//        gps_dev_power(s->fd,4);
        usleep(100000);
	gps_dev_set_baud_rate(s->fd, 115200);
        usleep(100000);

	cfsetispeed(&ios,B115200); 
	cfsetospeed(&ios,B115200); 
        tcsetattr(s->fd, TCSANOW, &ios );
        usleep(100000);

        tcflush(s->fd, TCIFLUSH );
        gps_dev_init(s->fd);

	nmea_reader_append( buff, 0 );

	s->version = false;
	for (;;)
	{
		if (s->version) break;

		int ret = read( s->fd, buff, sizeof(buff) );

	        if (ret > 0)
	        {
			nmea_reader_append( buff, ret );
        	}
		else
		{
			DFR("Timeout while reading from GPS socket");
			break;
		}
	}

        gps_dev_stop(s->fd);

    s->callbacks = callbacks;
    s->init = true;

    s->callbacks->set_capabilities_cb(GPS_CAPABILITY_ON_DEMAND_TIME || GPS_CAPABILITY_SCHEDULING);
/*
| GPS_CAPABILITY_SINGLE_SHOT
    private static final int GPS_CAPABILITY_SCHEDULING = 0x0000001;
    private static final int GPS_CAPABILITY_MSB = 0x0000002;
    private static final int GPS_CAPABILITY_MSA = 0x0000004;
    private static final int GPS_CAPABILITY_SINGLE_SHOT = 0x0000008;
    private static final int GPS_CAPABILITY_ON_DEMAND_TIME = 0x0000010;
    private static final int GPS_CAPABILITY_GEOFENCING = 0x0000020;
    private static final int GPS_CAPABILITY_MEASUREMENTS = 0x0000040;
    private static final int GPS_CAPABILITY_NAV_MESSAGES = 0x0000080;
*/
    update_gps_status(GPS_STATUS_ENGINE_ON);

    return 0;
}


static int serial_gps_start()
{
    GpsState*  s = _gps_state;

    DFR("Start");
    if (s->init == false)
    {
        DFR("%s: called with uninitialized state !", __FUNCTION__);
        return -1;
    }

    if (s->active)
    {
        DFR("gps_state_thread is already running!");
        return 0;
    }

    s->active = true;
    if (pthread_create(&(s->main_thread), NULL, gps_state_thread, NULL))
    {
        DFR("Could not create gps thread: %d %s", errno, strerror(errno));
        return -1;
    }

    gps_dev_start(s->fd);
    update_gps_status(GPS_STATUS_SESSION_BEGIN);

    return 0;
}

static int serial_gps_stop() 
{
    GpsState* s = _gps_state;

    DFR("Stop");

    if (s->init && s->active)
    {
//////////////////////////////////////////////////////////////////////////////////////////////
	s->active = false;
//////////////////////////////////////////////////////////////////////////////////////////////
	if (pthread_join(s->main_thread, NULL))
	{
		DFR("pthread_join returned error. [Error %d] %s", errno, strerror (errno));
	}
    }

    gps_dev_stop(s->fd);
    update_gps_status(GPS_STATUS_SESSION_END);

    return 0;
}

static void serial_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    DFR("Cleanup");

    if (s->init) 
    {
	if (s->active)
	{
//////////////////////////////////////////////////////////////////////////////////////////////
		s->active = false;
//////////////////////////////////////////////////////////////////////////////////////////////
		if (pthread_join(s->main_thread, NULL))
		{
		        DFR("pthread_join returned error. [Error %d] %s", errno, strerror (errno));
		}
	}

	gps_dev_deinit(s->fd);

        update_gps_status(GPS_STATUS_ENGINE_OFF);

        s->init = false;

    }

    if (s->fd >= 0) close(s->fd);

}


static int __isleap(long year)
{
	return (year) % 4 == 0 && ((year) % 100 != 0 || (year) % 400 == 0);
}

static long math_div(long a, long b)
{
	return a / b - (a % b < 0);
}

static long leaps_between(long y1, long y2)
{
	long leaps1 = math_div(y1 - 1, 4) - math_div(y1 - 1, 100)
		+ math_div(y1 - 1, 400);
	long leaps2 = math_div(y2 - 1, 4) - math_div(y2 - 1, 100)
		+ math_div(y2 - 1, 400);
	return leaps2 - leaps1;
}

static const unsigned short __mon_yday[2][13] = {
	/* Normal years. */
	{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
	/* Leap years. */
	{0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366}
};

#define SECS_PER_HOUR	(60 * 60)
#define SECS_PER_DAY	(SECS_PER_HOUR * 24)

void time64_to_tm(int64_t time, struct tm *result)
{
	long days, y;
	int rem;
	const unsigned short *ip;
 
	time /= 1000;
	days = time / SECS_PER_DAY;
	rem = time % SECS_PER_DAY;

	while (rem < 0) {
		rem += SECS_PER_DAY;
		--days;
	}
	while (rem >= SECS_PER_DAY) {
		rem -= SECS_PER_DAY;
		++days;
	}

	result->tm_hour = rem / SECS_PER_HOUR;
	rem %= SECS_PER_HOUR;
	result->tm_min = rem / 60;
	result->tm_sec = rem % 60;

	/* January 1, 1970 was a Thursday. */
	result->tm_wday = (4 + days) % 7;
	if (result->tm_wday < 0)
		result->tm_wday += 7;

	y = 1970;

	while (days < 0 || days >= (__isleap(y) ? 366 : 365)) {
		/* Guess a corrected year, assuming 365 days per year. */
		long yg = y + math_div(days, 365);

		/* Adjust DAYS and Y to match the guessed year. */
		days -= (yg - y) * 365 + leaps_between(y, yg);
		y = yg;
	}

	result->tm_year = y;

	result->tm_yday = days;

	ip = __mon_yday[__isleap(y)];
	for (y = 11; days < ip[y]; y--)
		continue;
	days -= ip[y];

	result->tm_mon = y + 1;
	result->tm_mday = days + 1;
}


static struct tm tim;

static int serial_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    GpsState*  s = _gps_state;
    time64_to_tm(time, &tim);

    if (s->init && s->active) {
	gps_dev_set_DT_UTC(s->fd, tim.tm_year, tim.tm_mon, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);
//	DFR("inject time:%lld %lld %d %04d/%02d/%02d %02d:%02d:%02d",time,timeReference,uncertainty, tim.tm_year, tim.tm_mon, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);
    }
    return 0;
}

static int serial_gps_inject_location(double latitude, double longitude, double altitude, float accuracy)
{

    GpsState*  s = _gps_state;

    if (s->init && s->active) {
	gps_dev_set_reference_loc(s->fd, latitude, longitude, altitude, tim.tm_year, tim.tm_mon, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);
//	DFR("inject loc: lat %f lon %f alt %f acc %f ",latitude, longitude, altitude, accuracy);
//	DFR("inject time:%d %d %d %d %d %d",tim.tm_year, tim.tm_mon, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);
    }
    return 0;
}

static void serial_gps_delete_aiding_data(GpsAidingData flags)
{
}

static int serial_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
        uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time) {

    GpsState* gps_state = _gps_state;

    gps_state->position_mode = mode;
    gps_state->min_interval  = min_interval;
    gps_state->recurrence    = recurrence;

    return 0;
}


static void* serial_gps_get_extension(const char* name)
{
    return NULL;
}

static void notify_gps_status(void* arg)
{
    GpsState* s = _gps_state;

    if (s->callbacks->status_cb) s->callbacks->status_cb(arg);

    free(arg);
}

static void update_gps_status(GpsStatusValue gpsStatusValue)
{
    GpsState* s = _gps_state;

    D("state update: status %d", gpsStatusValue);

    if ((s->active) && (s->callbacks) && (s->callbacks->create_thread_cb))
    {
	GpsStatus* sta = (GpsStatus*) malloc (sizeof(GpsStatus));
        sta->size = sizeof(GpsStatus);
    	sta->status = gpsStatusValue;
        s->callbacks->create_thread_cb("gps_state_emit_thread", notify_gps_status, (void*)sta);
    }
}


static void notify_gps_location(void* arg /* GpsLocation pointer */)
{
    GpsState* s = _gps_state;

    D("Notify location");

    if (s->callbacks->location_cb) s->callbacks->location_cb(arg);

    free(arg);
  
}

static void update_gps_location(GpsLocation* location)
{
    GpsState* s = _gps_state;

    D("Update Location");

    if ((s->active) && (s->callbacks) && (s->callbacks->create_thread_cb))
    {
	GpsLocation* loc = (GpsLocation*) malloc (sizeof(GpsLocation));
	memcpy(loc, location, sizeof(GpsLocation));

        s->callbacks->create_thread_cb("gps_loc_emit_thread", notify_gps_location, (void*)loc);
    }
}


static void notify_gps_sv_status(void* arg /* GnssSvStatus pointer */)
{
    GpsState* s = _gps_state;

    D("Notify Satellites status");

    if (s->callbacks->gnss_sv_status_cb) s->callbacks->gnss_sv_status_cb(arg);

    free(arg);
}

static void update_gps_svstatus(GnssSvStatus* svStat)
{
    GpsState* s = _gps_state;

    D("Update Satellite status");

    if ((s->active) && (s->callbacks) && (s->callbacks->create_thread_cb))
    {
	GnssSvStatus* sv = (GnssSvStatus*) malloc (sizeof(GnssSvStatus));
	memcpy(sv, svStat, sizeof(GnssSvStatus));

        s->callbacks->create_thread_cb("gps_sv_state_emit_thread", notify_gps_sv_status, (void*)sv);
    }
}

static void notify_gps_nmea(char* nmea)
{
    GpsState* s = _gps_state;

    D("Notify NMEA status");

    int len;
    memcpy(&len, nmea, 4);
	
    char* mNMEA = nmea + 4;

    struct timeval tv;
    gettimeofday(&tv, (struct timezone *) NULL);
    int64_t now = tv.tv_sec * 1000LL + tv.tv_usec / 1000;

    if (s->callbacks->nmea_cb) s->callbacks->nmea_cb(now, mNMEA, len);

    free(nmea);
}

void update_gps_nmea(const char* data, int len)
{
    GpsState* s = _gps_state;
//    D("Update nmea status");

    if ((s->active) && (s->callbacks) && (s->callbacks->create_thread_cb))
    {
	char* nmea = (char*) malloc(len + 4);
	memcpy(nmea, &len, 4);
	memcpy(nmea + 4, data, len);

        s->callbacks->create_thread_cb("gps_nmea_emit_thread", notify_gps_nmea, nmea);
    }
}


/*
typedef uint8_t                         GnssConstellationType;
#define GNSS_CONSTELLATION_UNKNOWN      0
#define GNSS_CONSTELLATION_GPS          1
#define GNSS_CONSTELLATION_SBAS         2
#define GNSS_CONSTELLATION_GLONASS      3
#define GNSS_CONSTELLATION_QZSS         4
#define GNSS_CONSTELLATION_BEIDOU       5
#define GNSS_CONSTELLATION_GALILEO      6

typedef uint8_t                                 GnssSvFlags;
#define GNSS_SV_FLAGS_NONE                      0
#define GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA        (1 << 0)
#define GNSS_SV_FLAGS_HAS_ALMANAC_DATA          (1 << 1)
#define GNSS_SV_FLAGS_USED_IN_FIX               (1 << 2)

typedef struct {
    size_t size;
    int16_t svid;
    GnssConstellationType constellation;
    float c_n0_dbhz;
    float elevation;
    float azimuth;
    GnssSvFlags flags;

} GnssSvInfo;

typedef struct {
    size_t size;
    int num_svs;
    GnssSvInfo gnss_sv_list[GNSS_MAX_SVS];

} GnssSvStatus;
*/

void notifier_svs_append(char talker[3], int prn, float elevation, float azimuth, float snr)
{
	// Skip empty
	if ((prn != 0) && (snr != 0.0) && (sv_counter < GNSS_MAX_SVS))
	{

		sv_status.gnss_sv_list[sv_counter].size = sizeof(GnssSvInfo);

                if (talker[1] == 'P')
		{
			sv_status.gnss_sv_list[sv_counter].svid = prn;
                	sv_status.gnss_sv_list[sv_counter].constellation = GNSS_CONSTELLATION_GPS;
		}
		else       
                if (talker[1] == 'L')
		{
			sv_status.gnss_sv_list[sv_counter].svid = prn-64;
                	sv_status.gnss_sv_list[sv_counter].constellation = GNSS_CONSTELLATION_GLONASS;
		}
		else       
                if (talker[1] == 'A')
		{
			sv_status.gnss_sv_list[sv_counter].svid = prn;
                	sv_status.gnss_sv_list[sv_counter].constellation = GNSS_CONSTELLATION_GALILEO;
		}
		else       
                if (talker[1] == 'D')
		{
			sv_status.gnss_sv_list[sv_counter].svid = prn;
                	sv_status.gnss_sv_list[sv_counter].constellation = GNSS_CONSTELLATION_BEIDOU;
		}

		sv_status.gnss_sv_list[sv_counter].c_n0_dbhz = snr;
		sv_status.gnss_sv_list[sv_counter].elevation = elevation;
		sv_status.gnss_sv_list[sv_counter].azimuth = azimuth;

		sv_status.gnss_sv_list[sv_counter].flags = GNSS_SV_FLAGS_NONE;

                if (talker[1] == 'P')
		{
			for (int i = 0; i < gps_svs_used_ids_X; i++)
				if (gps_svs_used_ids[i] == prn) 
				{
					sv_status.gnss_sv_list[sv_counter].flags = GNSS_SV_FLAGS_USED_IN_FIX;
					break;
				}

		}
		else       
                if (talker[1] == 'L')
		{
			for (int i = 0; i < glonass_svs_used_ids_X; i++)
				if (glonass_svs_used_ids[i] == prn) 
				{
					sv_status.gnss_sv_list[sv_counter].flags = GNSS_SV_FLAGS_USED_IN_FIX;
					break;
				}

		}
		else       
                if (talker[1] == 'A')
		{
			for (int i = 0; i < galileo_svs_used_ids_X; i++)
				if (galileo_svs_used_ids[i] == prn) 
				{
					sv_status.gnss_sv_list[sv_counter].flags = GNSS_SV_FLAGS_USED_IN_FIX;
					break;
				}

		}
		else       
                if (talker[1] == 'D')
		{
			for (int i = 0; i < beidou_svs_used_ids_X; i++)
				if (beidou_svs_used_ids[i] == prn) 
				{
					sv_status.gnss_sv_list[sv_counter].flags = GNSS_SV_FLAGS_USED_IN_FIX;
					break;
				}

		}

		sv_status.num_svs = ++sv_counter;
	}
}

void notifier_svs_inview(char talker[3], int num_svs)
{
}

void notifier_svs_used_ids(char talker[3], int ids[12])
{
        int j = 0;

        if (talker[1] == 'P')
	{
		while (gps_svs_used_ids_X < 32 && j < 12) 
		{
			if (ids[j]) gps_svs_used_ids[gps_svs_used_ids_X++] = ids[j];
			j++;
		}
	}
	else
        if (talker[1] == 'L')
	{
		while (glonass_svs_used_ids_X < 32 && j<12) 
		{
			if (ids[j]) glonass_svs_used_ids[glonass_svs_used_ids_X++] = ids[j];
			j++;
		}
	}
	else
        if (talker[1] == 'A')
	{
		while (galileo_svs_used_ids_X < 32 && j<12) 
		{
			if (ids[j]) galileo_svs_used_ids[galileo_svs_used_ids_X++] = ids[j];
			j++;
		}
	}
	else
        if (talker[1] == 'D')
	{
		while (beidou_svs_used_ids_X < 32 && j<12) 
		{
			if (ids[j]) beidou_svs_used_ids[beidou_svs_used_ids_X++] = ids[j];
			j++;
		}
	}
}

void notifier_svs_update_status()
{
        GpsState* s = _gps_state;

        if (s->active) {
		update_gps_svstatus(&sv_status);
	}

	// start new cycle to collect svs
	sv_counter = 0;
	gps_svs_used_ids_X = 0;
	glonass_svs_used_ids_X = 0;
	galileo_svs_used_ids_X = 0;
	beidou_svs_used_ids_X = 0;
}

void notifier_set_speed(float speed_knots)
{
	location.flags   |= GPS_LOCATION_HAS_SPEED;
	location.speed    = speed_knots * 1.852 / 3.6; // knots to m/s
}

void notifier_set_bearing(float bearing)
{
	location.flags   |= GPS_LOCATION_HAS_BEARING;
    	location.bearing  = bearing;
}

void notifier_set_latlong(double lat, double lon)
{
	location.flags    |= GPS_LOCATION_HAS_LAT_LONG;
	location.latitude  = lat;
	location.longitude = lon;
}

void notifier_set_altitude(double altitude, char units)
{
	location.flags   |= GPS_LOCATION_HAS_ALTITUDE;
	location.altitude = altitude;
}

void notifier_set_accuracy(float accuracy)
{
	location.flags   |= GPS_LOCATION_HAS_ACCURACY;
	location.accuracy = accuracy;
}

void notifier_set_date_time(struct minmea_date date, struct minmea_time time_)
{
	struct timespec ts;
	minmea_gettime(&ts, &date, &time_);
	location.timestamp = (long long) ts.tv_sec * 1000 + (long long) (ts.tv_nsec / 1e6);
}

void notifier_push_location()
{
        GpsState* s = _gps_state;
        if (!s->active) {
		location.flags = 0;
		return;
	}

	uint16_t t = GPS_LOCATION_HAS_LAT_LONG |
	             GPS_LOCATION_HAS_ALTITUDE |
	       	     GPS_LOCATION_HAS_SPEED    |
		     GPS_LOCATION_HAS_BEARING  |
		     GPS_LOCATION_HAS_ACCURACY;

	if ((location.flags & t) == t)
	{
		update_gps_location(&location);
		location.flags = 0;
	}

}

void notifier_set_version(char *brand, char *vers)
{
        GpsState* s = _gps_state;

        if (s->version) return;
 
	DFR("notifier_set_version");

//	property_set("sys.gps.brand", brand);
	property_set("sys.gps.version", vers);

        s->version = true; 
}

