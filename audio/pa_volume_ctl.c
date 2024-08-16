#define LOG_TAG "audio_pa_volume"
#define LOG_NDEBUG 0

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>

#include <log/log.h>
#include <pulse/pulseaudio.h>
#include <pulsecore/macro.h>
#include <pulsecore/core-util.h>

#include "pa_volume_ctl.h"

struct pa_callback_args {
	pa_mainloop_api *mainloop_api;
	pa_context *context;
	enum action m_action;
	pa_cvolume m_volume;
	bool muted;
	pthread_mutex_t lock;
	int quit_count;
	int ret;
	char *set_sink_name;
	char *set_source_name;
	char *get_sink_name;
	char *set_sink_name_mute;
	char *set_source_name_mute;
	char *get_sink_name_mute;
};

static void quit(int ret, void *userdata)
{
    pa_assert(((struct pa_callback_args *)userdata)->mainloop_api);
    ((struct pa_callback_args *)userdata)->mainloop_api->quit(((struct pa_callback_args *)userdata)->mainloop_api, ret);
}

static void context_drain_complete(pa_context *c, void *userdata)
{
    pa_context_disconnect(c);
}

static void drain(void *userdata)
{
    pa_operation *o;
    if (!(o = pa_context_drain(((struct pa_callback_args *)userdata)->context, context_drain_complete, NULL))) {
        pa_context_disconnect(((struct pa_callback_args *)userdata)->context);
    } else {
        pa_operation_unref(o);
    }
}

static void fill_volume(pa_cvolume *cv, pa_cvolume *m_volume, unsigned supported)
{
    pa_cvolume_set(m_volume, supported, m_volume->values[0]);
    *cv = *m_volume;
}

static void simple_callback(pa_context *c, int success, void *userdata)
{
    if (!success) {
        ALOGE("Failure: %s", pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }
	pthread_mutex_lock(&((struct pa_callback_args *)userdata)->lock);
	int quit_count = --((struct pa_callback_args *)userdata)->quit_count;
	pthread_mutex_unlock(&((struct pa_callback_args *)userdata)->lock);
	if (quit_count == 0) {
		drain(userdata);
	}
}

static void get_sink_info_callback(pa_context *c, const pa_sink_info *i, int is_last, void *userdata)
{
    pa_cvolume cv;
    if (is_last < 0) {
        ALOGE("(%d)Failed to get sink information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }

    if (is_last) {
        return;
    }

    pa_assert(i);

    cv = i->volume;
    fill_volume(&cv, &((struct pa_callback_args *)userdata)->m_volume, i->channel_map.channels);

    pa_operation_unref(pa_context_set_sink_volume_by_name(c, ((struct pa_callback_args *)userdata)->set_sink_name, 
		&cv, simple_callback, userdata));
}

static void get_source_volume_callback(pa_context *c, const pa_source_info *i, int is_last, void *userdata)
{
    pa_cvolume cv;
    if (is_last < 0) {
        ALOGE("(%d)Failed to get source information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }

    if (is_last) {
        return;
    }

    pa_assert(i);

    cv = i->volume;
    fill_volume(&cv, &((struct pa_callback_args *)userdata)->m_volume, i->channel_map.channels);

    pa_operation_unref(pa_context_set_source_volume_by_name(c, ((struct pa_callback_args *)userdata)->set_source_name, &cv, 
		simple_callback, userdata));
}

static void set_master_volume_callback(pa_context *c, const pa_server_info *i, void *userdata)
{
	if (!i) {
		ALOGE("(%d)Failed to get server information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }

	ALOGV(("Default Sink: %s\nDefault Source: %s\n"), i->default_sink_name, i->default_source_name);

	((struct pa_callback_args *)userdata)->set_sink_name = pa_xstrdup(i->default_sink_name);
	((struct pa_callback_args *)userdata)->set_source_name = pa_xstrdup(i->default_source_name);

	pa_operation_unref(pa_context_get_sink_info_by_name(c, ((struct pa_callback_args *)userdata)->set_sink_name, 
        get_sink_info_callback, userdata));
	pa_operation_unref(pa_context_get_source_info_by_name(c, ((struct pa_callback_args *)userdata)->set_source_name, 
        get_source_volume_callback, userdata));
}

static void get_mute_callback(pa_context *c, const pa_sink_info *i, int is_last, void *userdata)
{
    if (is_last < 0) {
        ALOGE("(%d)Failed to get sink information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }

    if (is_last) {
        return;
    }

    pa_assert(i);

	ALOGV("Mute: %s, Mute: %d", pa_yes_no_localised(i->mute), i->mute);
	((struct pa_callback_args *)userdata)->ret = i->mute;
	drain(userdata);
}

static void get_sink_volume_callback(pa_context *c, const pa_sink_info *i, int is_last, void *userdata)
{
    if (is_last < 0) {
        ALOGE("(%d)Failed to get sink information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }

    if (is_last) {
        return;
    }

    pa_assert(i);
	pa_volume_t volume = i->volume.values[0];

	ALOGV("Mute: %s ,Volume: %d",
		pa_yes_no_localised(i->mute),
		(unsigned)(((uint64_t)volume * 100 + (uint64_t)PA_VOLUME_NORM / 2) / (uint64_t)PA_VOLUME_NORM));
	((struct pa_callback_args *)userdata)->ret = (int)(((uint64_t)volume * 100 + (uint64_t)PA_VOLUME_NORM / 2) / (uint64_t)PA_VOLUME_NORM);
	drain(userdata);
}

static void get_master_volume_callback(pa_context *c, const pa_server_info *i, void *userdata)
{
	if (!i) {
        ALOGE("(%d)Failed to get server information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }
    ALOGV("Default Sink: %s", i->default_sink_name);
	((struct pa_callback_args *)userdata)->get_sink_name = pa_xstrdup(i->default_sink_name);

	pa_operation_unref(pa_context_get_sink_info_by_name(c, ((struct pa_callback_args *)userdata)->get_sink_name, 
		get_sink_volume_callback, userdata));
}

static void set_master_mute_callback(pa_context *c, const pa_server_info *i, void *userdata)
{
	if (!i) {
        ALOGE("(%d)Failed to get server information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }
    ALOGV("Default Sink: %s ,Default Source: %s", i->default_sink_name, i->default_source_name);
	((struct pa_callback_args *)userdata)->set_sink_name_mute = pa_xstrdup(i->default_sink_name);
	((struct pa_callback_args *)userdata)->set_source_name_mute = pa_xstrdup(i->default_source_name);

	pa_operation_unref(pa_context_set_sink_mute_by_name(c, ((struct pa_callback_args *)userdata)->set_sink_name_mute, 
		((struct pa_callback_args *)userdata)->muted, simple_callback, userdata));
	pa_operation_unref(pa_context_set_source_mute_by_name(c, ((struct pa_callback_args *)userdata)->set_source_name_mute, 
		((struct pa_callback_args *)userdata)->muted, simple_callback, userdata));
}

static void get_master_mute_callback(pa_context *c, const pa_server_info *i, void *userdata)
{
	if (!i) {
        ALOGE("(%d)Failed to get server information: %s", __LINE__, pa_strerror(pa_context_errno(c)));
        quit(1, userdata);
        return;
    }
    ALOGV("Default Sink: %s", i->default_sink_name);
	((struct pa_callback_args *)userdata)->get_sink_name_mute = pa_xstrdup(i->default_sink_name);

	pa_operation_unref(pa_context_get_sink_info_by_name(c, ((struct pa_callback_args *)userdata)->get_sink_name_mute, 
		get_mute_callback, userdata));
}

static void context_state_callback(pa_context *c, void *userdata)
{
    pa_operation *o = NULL;
	enum action m_action = ((struct pa_callback_args *)userdata)->m_action;

    pa_assert(c);
	ALOGV("pa_context_get_state: %d, action: %d", pa_context_get_state(c), m_action);

    switch (pa_context_get_state(c)) {
        case PA_CONTEXT_CONNECTING:
        case PA_CONTEXT_AUTHORIZING:
        case PA_CONTEXT_SETTING_NAME:
            break;
        case PA_CONTEXT_READY:
            switch (m_action) {
				case SET_MASTER_VOLUME:
					((struct pa_callback_args *)userdata)->quit_count = 2;
					o = pa_context_get_server_info(c, set_master_volume_callback, userdata);
                    break;
				case GET_MASTER_VOLUME:
					o = pa_context_get_server_info(c, get_master_volume_callback, userdata);
                    break;
				case SET_MASTER_MUTE:
					((struct pa_callback_args *)userdata)->quit_count = 2;
                    o = pa_context_get_server_info(c, set_master_mute_callback, userdata);
                    break;
				case GET_MASTER_MUTE:
                    o = pa_context_get_server_info(c, get_master_mute_callback, userdata);
                    break;
                default:
                    pa_assert_not_reached();
            }

            if (o) {
                pa_operation_unref(o);
            }
            break;

        case PA_CONTEXT_TERMINATED:
            quit(0, userdata);
            break;

        case PA_CONTEXT_FAILED:
        default:
		    ALOGE("Connection failure: %s", pa_strerror(pa_context_errno(c)));
            quit(1, userdata);
    }
}

static pa_process(float volume, bool mute, enum action action)
{
	pa_mainloop *m = NULL;
	int ret = -1;
	char *server = NULL;
	struct pa_callback_args m_pa_callback_args;
	m_pa_callback_args.muted = mute;
	m_pa_callback_args.m_volume.channels = 1;
	m_pa_callback_args.m_volume.values[0] = (pa_volume_t)((int)(volume * 100) * (double) PA_VOLUME_NORM / 100);

	if (!(m = pa_mainloop_new())) {
		ALOGE("pa_mainloop_new() failed.");
		goto quit;
	}

	m_pa_callback_args.mainloop_api = pa_mainloop_get_api(m);

	if (!(m_pa_callback_args.context = pa_context_new_with_proplist(m_pa_callback_args.mainloop_api, NULL, NULL))) {
		ALOGE("pa_context_new() failed.");
		goto quit;
	}

	m_pa_callback_args.m_action = action; 
	pthread_mutex_init (&m_pa_callback_args.lock, (const pthread_mutexattr_t *)NULL);

	pa_context_set_state_callback(m_pa_callback_args.context, context_state_callback, &m_pa_callback_args);

	if (pa_context_connect(m_pa_callback_args.context, server, 0, NULL) < 0) {
		ALOGE("pa_context_connect() failed: %s", pa_strerror(pa_context_errno(m_pa_callback_args.context)));
		goto quit;
	}

	if (pa_mainloop_run(m, &ret) < 0) {
		ALOGE("pa_mainloop_run() failed.");
		goto quit;
	}
	ret = m_pa_callback_args.ret;

quit:

	if (m_pa_callback_args.context) {
		pa_context_unref(m_pa_callback_args.context);
    }

	if (m) {
		pa_mainloop_free(m);
	}

	pa_xfree(server);
	pa_xfree(m_pa_callback_args.set_sink_name);
	pa_xfree(m_pa_callback_args.set_source_name);
	pa_xfree(m_pa_callback_args.get_sink_name);
	pa_xfree(m_pa_callback_args.set_sink_name_mute);
	pa_xfree(m_pa_callback_args.get_sink_name_mute);
	pa_xfree(m_pa_callback_args.set_source_name_mute);
	pthread_mutex_destroy(&m_pa_callback_args.lock);

	return ret;
}

int pa_set_master_volume(float volume)
{
    return pa_process(volume, false, SET_MASTER_VOLUME);
}

int pa_get_master_volume(float *volume)
{
    int ret = pa_process(0, false, GET_MASTER_VOLUME);
	if (ret < 0) {
        return ret;
	}
	
    *volume = ret / 100.0;
	ALOGV("volume: %f", *volume);
    return 0;
}

int pa_set_master_mute(bool muted)
{
    return pa_process(0, muted, SET_MASTER_MUTE);
}

int pa_get_master_mute(bool *muted)
{
	int ret = pa_process(0, false, GET_MASTER_MUTE);
	if (ret < 0) {
		return ret;
	}
	
	*muted = ret;
	ALOGV("muted: %d", ret);
    return 0;
}
