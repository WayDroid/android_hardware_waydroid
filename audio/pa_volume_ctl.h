#ifndef PA_VOLUME_CTL_H
#define PA_VOLUME_CTL_H
enum action {
    SET_MASTER_VOLUME,
    GET_MASTER_VOLUME,
    SET_MASTER_MUTE,
    GET_MASTER_MUTE
};

int pa_set_master_volume(float volume);
int pa_get_master_volume(float *volume);
int pa_set_master_mute(bool muted);
int pa_get_master_mute(bool *muted);
#endif
