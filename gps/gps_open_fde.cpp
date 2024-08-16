/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define  LOG_TAG  "gps_fde"

#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <cutils/properties.h>
#include <hardware/gps.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <string>
#include <log/log.h>
#include "android-base/file.h"

using std::string;

#define  GPS_DEBUG  0

#undef D
#if GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  64

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

/* this is the state of our connection to the qemu_gpsd daemon */
typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
} GpsState;

static GpsState  _gps_state[1];
static const string unix_str_path = "/tmp/unix.str";

static int nmea_tokenizer_init(NmeaTokenizer* t, const char* p, const char* end)
{
    D("FUNC: %s, FILE: %s, LINE: %d, p: %s\n", __FUNCTION__, __FILE__, __LINE__, p);
    int    count = 0;

    // the initial '$' is optional
    if (p < end && p[0] == '$') {
        p += 1;
    }

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r') {
            end -= 1;
        }
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = (const char*)memchr(p, ',', end-p);
        if (q == NULL) {
            q = end;
        }

        if (count < MAX_NMEA_TOKENS) {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }
        if (q < end) {
            q += 1;
        }
        p = q;
    }

    t->count = count;
    return count;
}

static Token nmea_tokenizer_get(NmeaTokenizer* t, int index)
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else {
        tok = t->tokens[index];
    }
    return tok;
}

static double str2float(const char* p, const char* end)
{
    int   len    = end - p;
    char  temp[64];

    if (len >= (int)sizeof(temp)) {
        D("%s %d input is too long: '%.*s'", __func__, __LINE__, (int)(end-p), p);
        return 0.;
    }

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    GpsLocation  fix;
    gps_location_callback  callback;
    char    in[1024];
} NmeaReader;

static void nmea_reader_init(NmeaReader* r)
{
    memset(r, 0, sizeof(*r));

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->callback = NULL;
    r->fix.size = sizeof(r->fix);
}

static int nmea_reader_update_time(NmeaReader* r)
{
    struct tm  tm;
    time_t     fix_time;
    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }	
	time_t utc_time = time(NULL);
	struct tm *local_tm = localtime(&utc_time);

    tm.tm_hour  = local_tm->tm_hour;
    tm.tm_min   = local_tm->tm_min;
    tm.tm_sec   = local_tm->tm_sec;
    tm.tm_year  = r->utc_year - 1900;
    tm.tm_mon   = r->utc_mon - 1;
    tm.tm_mday  = r->utc_day;
    tm.tm_isdst = -1;

    fix_time = timegm( &tm );
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int nmea_reader_update_latlong(NmeaReader* r, Token         latitude, char latitudeHemi, Token longitude, char longitudeHemi)
{
    double   lat, lon;
    Token    tok;

    r->fix.flags &= ~GPS_LOCATION_HAS_LAT_LONG;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        D("latitude is too short: '%.*s'", (int)(tok.end-tok.p), tok.p);
        return -1;
    }
    lat = str2float(tok.p, tok.end);
    if (latitudeHemi == 'S') {
        lat = -lat;
    }

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        D("longitude is too short: '%.*s'", (int)(tok.end-tok.p), tok.p);
        return -1;
    }
    lon = str2float(tok.p, tok.end);
    if (longitudeHemi == 'W') {
        lon = -lon;
	}
    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}

static int nmea_reader_update_altitude(NmeaReader* r, Token altitude, Token __unused units)
{
    Token   tok = altitude;

    r->fix.flags &= ~GPS_LOCATION_HAS_ALTITUDE;

    if (tok.p >= tok.end) {
        return -1;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int nmea_reader_update_accuracy(NmeaReader* r)
{
    // Always return 20m accuracy.
    // Possibly parse it from the NMEA sentence in the future.
    r->fix.flags    |= GPS_LOCATION_HAS_ACCURACY;
    r->fix.accuracy = 20;
    return 0;
}

static bool has_all_required_flags(GpsLocationFlags flags) {
    return ( flags & GPS_LOCATION_HAS_LAT_LONG
            && flags & GPS_LOCATION_HAS_ALTITUDE
           );
}

static bool is_ready_to_send(NmeaReader* r) {
    return has_all_required_flags(r->fix.flags);
}

static void nmea_reader_set_callback(NmeaReader* r, gps_location_callback cb)
{
    r->callback = cb;
	D("%s: r->fix.flags:%d", __FUNCTION__, r->fix.flags);
    if (cb != NULL && is_ready_to_send(r)) {
        D("%s: sending latest fix to new callback", __FUNCTION__);
        r->callback( &r->fix );
    }
}

static void nmea_reader_parse(NmeaReader* r)
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
        D("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, (int)(tok.end-tok.p), tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        D("sentence id '%.*s' too short, ignored.", (int)(tok.end-tok.p), tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if (!memcmp(tok.p, "GGA", 3)) {
        // GPS fix
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else {
        tok.p -= 2;
        D("unknown sentence '%.*s", (int)(tok.end-tok.p), tok.p);
    }

    // Always update accuracy
    nmea_reader_update_accuracy( r );

    if (is_ready_to_send(r)) {
#if GPS_DEBUG
        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        //The unit of r->fix.timestamp is millisecond.
        time_t timestamp = r->fix.timestamp / 1000;
        gmtime_r( (time_t*) &timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
		D("fix: %s", temp);
#endif
        if (r->callback) {
            r->callback( &r->fix );
            /* we have sent a complete fix, now prepare for next complete fix */
            //r->fix.flags = 0;
        } else {
            D("no callback, keeping data until needed !");
        }
    }
}

static void nmea_reader_addc(NmeaReader* r, int c)
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

	D("FUNC: %s, FILE: %s, LINE: %d, cc: %c, cd: %d, pos: %d\n", __FUNCTION__, __FILE__, __LINE__, (char)c,  c, r->pos);

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == 'M') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};

static void gps_state_done(GpsState* s)
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;
    s->init = 0;
}

static void gps_state_start(GpsState* s)
{
    char  cmd = CMD_START;
    int   ret;

    do { 
		ret = write(s->control[0], &cmd, 1); 
	} while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        D("%s: could not send CMD_START command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
    }
}

static void gps_state_stop(GpsState* s)
{
    char  cmd = CMD_STOP;
    int   ret;

    do {
		ret = write(s->control[0], &cmd, 1);
	} while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        D("%s: could not send CMD_STOP command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
    }
}

static int epoll_register(int epoll_fd, int fd)
{
    struct epoll_event  ev;
    int ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static int Socket(int family, int type, int protocol)
{
	int	n;
	if ((n = socket(family, type, protocol)) < 0) {
		D("socket error: %s", strerror(errno));
	}
	return(n);
}
static void Bind(int fd, const struct sockaddr *sa, socklen_t salen)
{
	if (bind(fd, sa, salen) < 0) {
		D("bind error: %s", strerror(errno));
	}
}
static void Listen(int fd, int backlog)
{
	if (listen(fd, backlog) < 0) {
		D("listen error: %s", strerror(errno));
	}
}

static void Close(int fd)
{
	if (close(fd) == -1) {
		D("close error: %s", strerror(errno));
	}
}

static ssize_t writen(int fd, const void *vptr, size_t n)
{
	size_t		nleft;
	ssize_t		nwritten;
	const char	*ptr;

	ptr = (const char *)vptr;
	nleft = n;
	while (nleft > 0) {
		if ((nwritten = write(fd, ptr, nleft)) <= 0) {
			if (nwritten < 0 && errno == EINTR) {
				nwritten = 0;
			} else {
				return(-1);
			}
		}

		nleft -= nwritten;
		ptr   += nwritten;
	}
	return(n);
}

static bool Connect(int fd, const struct sockaddr *sa, socklen_t salen)
{
	if (connect(fd, sa, salen) < 0) {
        D("connect error: %s", strerror(errno));
		return false;
	}
	return true;	
}

static void Writen(int fd, void *ptr, size_t nbytes)
{
	if (writen(fd, ptr, nbytes) != nbytes) {
		D("writen error: %s", strerror(errno));
	}
}

static bool WriteFully(int fd, const void* data, size_t byte_count) {
    const uint8_t* p = (const uint8_t*)(data);
    size_t remaining = byte_count;
    while (remaining > 0) {
      ssize_t n = TEMP_FAILURE_RETRY(write(fd, p, remaining));
      if (n == -1) {
	      return false;
      }
      p += n;
      remaining -= n;
    }
    return true;
}

static bool client(int *fd)
{
	struct sockaddr_un	servaddr;
	*fd = Socket(AF_LOCAL, SOCK_STREAM, 0);
	
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sun_family = AF_LOCAL;
	strcpy(servaddr.sun_path, unix_str_path.c_str());
	
	return Connect(*fd, (struct sockaddr *) &servaddr, sizeof(servaddr));
}

static void gps_state_thread(void*  arg)
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];
    GpsStatus gps_status;
    gps_status.size = sizeof(gps_status);
    GnssSvStatus gnss_sv_status;
    memset(&gnss_sv_status, 0, sizeof(gnss_sv_status));
    gnss_sv_status.size = sizeof(gnss_sv_status);
    gnss_sv_status.num_svs = 1;
    gnss_sv_status.gnss_sv_list[0].size = sizeof(gnss_sv_status.gnss_sv_list[0]);
    gnss_sv_status.gnss_sv_list[0].svid = 1;
    gnss_sv_status.gnss_sv_list[0].c_n0_dbhz = 0.0;
    gnss_sv_status.gnss_sv_list[0].elevation = 30.0;
    gnss_sv_status.gnss_sv_list[0].azimuth = 30.0;
    gnss_sv_status.gnss_sv_list[0].flags = GNSS_SV_FLAGS_USED_IN_FIX;

    nmea_reader_init(reader);

    int rrr = epoll_register(epoll_fd, control_fd);
	if (rrr < 0) {
		D("epoll_register(control_fd) unexpected error: %s", strerror(errno));
		return;
	}
    rrr = epoll_register(epoll_fd, gps_fd);
	if (rrr < 0) {
		D("epoll_register(gps_fd) unexpected error: %s", strerror(errno));
		return;
	}

    D("gps thread running");
    bool first_enter = true;
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        int timeout = -1;
        if (gps_status.status == GPS_STATUS_SESSION_BEGIN) {
            timeout = 10 * 1000;
        }

		if (first_enter) {
			if (!WriteFully(gps_fd, "H", 2)) {
                D("%s: Could not connect to service: %s", __FUNCTION__, strerror(errno));
            } else {
                first_enter = false;
			}
		}

        nevents = epoll_wait(epoll_fd, events, 2, timeout);
        if (state->callbacks.gnss_sv_status_cb) {
            state->callbacks.gnss_sv_status_cb(&gnss_sv_status);
        }
        // update satilite info
        if (nevents < 0) {
            if (errno != EINTR) {
                D("epoll_wait() unexpected error: %s", strerror(errno));
            }
            continue;
        }

        D("gps thread received %d events", nevents);
		if ((nevents == 0) && (gps_status.status == GPS_STATUS_SESSION_BEGIN)) {
			int fd;
			if (client(&fd)) {
			    if (!WriteFully(fd, "send", sizeof("send"))) {
                    D("%s: WriteFully fail: %s", __FUNCTION__, strerror(errno));
                }
			} else {
                Close(fd);
			}
		}
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                D("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                return;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd) {
                    char  cmd = 0xFF;
                    int   ret;
                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        return;
                    } else if (cmd == CMD_START) {
                        if (!started) {
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            nmea_reader_set_callback( reader, state->callbacks.location_cb );
                            gps_status.status = GPS_STATUS_SESSION_BEGIN;
                            if (state->callbacks.status_cb) {
                                state->callbacks.status_cb(&gps_status);
                            }
                        }
                    } else if (cmd == CMD_STOP) {
                        if (started) {
                            D("gps thread stopping");
                            started = 0;
                            nmea_reader_set_callback( reader, NULL );
                            gps_status.status = GPS_STATUS_SESSION_END;
                            if (state->callbacks.status_cb) {
                                state->callbacks.status_cb(&gps_status);
                            }
                        }
                    }
                } else if (fd == gps_fd) {
                    char  buff[32];
                    D("gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read(fd, buff, sizeof(buff));
                        if (ret < 0) {
                            if (errno == EINTR) {
                                continue;
                            }
                            if (errno != EWOULDBLOCK) {
                                D("error while reading from gps daemon socket: %s:", strerror(errno));
                            }
                            break;
                        }
                        D("received %d bytes: %.*s", ret, ret, buff);
                        for (nn = 0; nn < ret; nn++) {
                            nmea_reader_addc( reader, buff[nn] );
                        }
                    }
                    D("gps fd event end");
                } else {
                    D("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
}

void send_last_data(int fd)
{
    char buffer[128] = {0};
	property_get("persist.fde.gnss", buffer, "$GPGGA,,28.228304,N,112.938882,E,,,,58,M");
	D("strlen(buffer): %zu", strlen(buffer));
	Writen(fd, (void *)buffer, strlen(buffer));
}

void * server(void * __unused arg)
{
	int client_hal_fd;
	int 				listenfd, connfd;
	socklen_t			clilen;
	struct sockaddr_un	cliaddr, servaddr;

	listenfd = Socket(AF_LOCAL, SOCK_STREAM, 0);

	unlink(unix_str_path.c_str());
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sun_family = AF_LOCAL;
	strcpy(servaddr.sun_path, unix_str_path.c_str());

	Bind(listenfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
	int res = chmod(unix_str_path.c_str(),0666);
	if(res) {
		D("chmod fail!!(%s)", strerror(errno));
	}

	Listen(listenfd, 1024);

	for (;;) {
		clilen = sizeof(cliaddr);
		if ((connfd = accept(listenfd, (struct sockaddr *) &cliaddr, &clilen)) < 0) {
			if (errno == EINTR) {
				continue;
			} else {
				D("accept error");
            }
		}
	    ssize_t		n;
	    char		buf[4096];
		if (((n = read(connfd, buf, 4096)) > 0) && (strncmp("H", buf, 1) == 0)) {
            client_hal_fd = connfd;
			usleep(5000);
		    send_last_data(client_hal_fd);
		} else if ((n > 0) && (strncmp("send", buf, 4) == 0)) {
			send_last_data(client_hal_fd);
		} else if ((n > 0) && (strncmp("$GPGGA,,", buf, strlen("$GPGGA,,")) == 0)) {
			D("get buf: %s\n", buf);
			Writen(client_hal_fd, (void *)buf, strlen(buf));
			property_set("persist.fde.gnss", buf);
			
		} else {
			D("unknown buf: %s\n", buf);
		}
		if (client_hal_fd != connfd) {
		    Close(connfd);
		}
	}
}

static void gps_state_init(GpsState* state, GpsCallbacks* callbacks)
{
    state->init       = 1;
    state->control[0] = -1;
    state->control[1] = -1;
    pthread_t tid;
	int wait_count = 10000;
    int err = pthread_create(&tid, NULL, server, (void *)state);
    if (err) {
	    D("pthread_create fail(%s)", strerror(errno));
		goto Fail;
    }
	while (!client(&state->fd) && wait_count) {
        wait_count--;
		Close(state->fd);
		usleep(100);
	}

    if (wait_count == 0) {
        D("no gps emulation server(%s)", strerror(errno));
        goto Fail;
    }

    if (socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0) {
        D("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    state->thread = callbacks->create_thread_cb( "gps_state_thread", gps_state_thread, state );

    if (!state->thread) {
        D("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }

    state->callbacks = *callbacks;

    // Explicitly initialize capabilities
    state->callbacks.set_capabilities_cb(0);

    // Setup system info, we are pre 2016 hardware.
    GnssSystemInfo sysinfo;
    sysinfo.size = sizeof(GnssSystemInfo);
    sysinfo.year_of_hw = 2015;
    state->callbacks.set_system_info_cb(&sysinfo);

    D("gps state initialized");
    return;

Fail:
	D("FUNC: %s, FILE: %s, LINE: %d,\n", __FUNCTION__, __FILE__, __LINE__);
    gps_state_done(state);
}

static int qemu_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        gps_state_init(s, callbacks);
    }

    if (s->fd < 0) {
        return -1;
    }

    return 0;
}

static void qemu_gps_cleanup(void)
{
	D("FUNC: %s, FILE: %s, LINE: %d,\n", __FUNCTION__, __FILE__, __LINE__);
#if 0
    GpsState*  s = _gps_state;
    if (s->init) {
        gps_state_done(s);
    }
#endif
}

static int qemu_gps_start()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}

static int qemu_gps_stop()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}

static int qemu_gps_inject_time(GpsUtcTime __unused time,
                     int64_t __unused timeReference,
                     int __unused uncertainty)
{
    return 0;
}

static int qemu_gps_inject_location(double __unused latitude,
                         double __unused longitude,
                         float __unused accuracy)
{
    return 0;
}

static void qemu_gps_delete_aiding_data(GpsAidingData __unused flags)
{
}

static int qemu_gps_set_position_mode(GpsPositionMode __unused mode,
                                      GpsPositionRecurrence __unused recurrence,
                                      uint32_t __unused min_interval,
                                      uint32_t __unused preferred_accuracy,
                                      uint32_t __unused preferred_time)
{
    // FIXME - support fix_frequency
    return 0;
}

static const void* qemu_gps_get_extension(const char* __unused name)
{
    return NULL;
}

static const GpsInterface  qemuGpsInterface = {
    sizeof(GpsInterface),
    qemu_gps_init,
    qemu_gps_start,
    qemu_gps_stop,
    qemu_gps_cleanup,
    qemu_gps_inject_time,
    qemu_gps_inject_location,
    qemu_gps_delete_aiding_data,
    qemu_gps_set_position_mode,
    qemu_gps_get_extension,
};

const GpsInterface* gps__get_gps_interface(struct gps_device_t* __unused dev)
{
    return &qemuGpsInterface;
}

static int open_gps(const struct hw_module_t* module,
                    char const* __unused name,
                    struct hw_device_t** device)
{
    struct gps_device_t *dev = (struct gps_device_t *)malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}

static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "OpenFDE GPS Module",
    .author = "OpenFDE",
    .methods = &gps_module_methods,
};
