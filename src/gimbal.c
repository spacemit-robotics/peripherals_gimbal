/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gimbal_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static struct gimbal_driver_info *g_driver_list = NULL;

static float clampf_local(float value, float min_value, float max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

void gimbal_driver_register(struct gimbal_driver_info *info) {
    if (!info)
        return;

    info->next = g_driver_list;
    g_driver_list = info;
    printf("[GIMBAL] Registered driver: %s\n", info->name);
}

static struct gimbal_driver_info *find_driver(const char *name) {
    struct gimbal_driver_info *curr = g_driver_list;

    while (curr) {
        if (curr->name && name && strcmp(curr->name, name) == 0)
            return curr;
        curr = curr->next;
    }

    printf("[GIMBAL] Driver not found: %s\n", name ? name : "(null)");
    return NULL;
}

struct gimbal_dev *gimbal_dev_alloc(const char *name, size_t priv_size) {
    struct gimbal_dev *dev = calloc(1, sizeof(*dev));
    void *priv = NULL;
    char *name_copy = NULL;

    if (!dev)
        return NULL;

    if (priv_size > 0U) {
        priv = calloc(1, priv_size);
        if (!priv) {
            free(dev);
            return NULL;
        }
        dev->priv_data = priv;
    }

    if (name) {
        size_t n = strlen(name);
        name_copy = calloc(1, n + 1U);
        if (!name_copy) {
            free(priv);
            free(dev);
            return NULL;
        }
        memcpy(name_copy, name, n);
        dev->name = name_copy;
    }

    pthread_mutex_init(&dev->state_lock, NULL);
    pthread_mutex_init(&dev->cmd_lock, NULL);
    dev->mode = GIMBAL_MODE_OFF;

    return dev;
}

void gimbal_dev_free_default(struct gimbal_dev *dev) {
    if (!dev)
        return;

    pthread_mutex_destroy(&dev->state_lock);
    pthread_mutex_destroy(&dev->cmd_lock);

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free(dev->name);
    free(dev);
}

double gimbal_now_monotonic_s(void) {
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

float gimbal_normalize_deg(float angle_deg) {
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    while (angle_deg < -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

float gimbal_delta_deg(float target_deg, float current_deg) {
    return gimbal_normalize_deg(target_deg - current_deg);
}

static void apply_limits_locked(struct gimbal_dev *dev, gimbal_mode_t mode,
                                gimbal_euler_t *target) {
    if (!dev->limits_valid || !target)
        return;

    if (mode == GIMBAL_MODE_SPEED) {
        target->pitch = clampf_local(
            target->pitch,
            -fabsf(dev->limits.max_speed.pitch),
            fabsf(dev->limits.max_speed.pitch));
        target->yaw = clampf_local(
            target->yaw,
            -fabsf(dev->limits.max_speed.yaw),
            fabsf(dev->limits.max_speed.yaw));
        target->roll = clampf_local(
            target->roll,
            -fabsf(dev->limits.max_speed.roll),
            fabsf(dev->limits.max_speed.roll));
        return;
    }

    target->pitch = clampf_local(target->pitch,
        dev->limits.min_angle.pitch,
        dev->limits.max_angle.pitch);
    target->yaw = clampf_local(target->yaw,
        dev->limits.min_angle.yaw,
        dev->limits.max_angle.yaw);
    target->roll = clampf_local(target->roll,
        dev->limits.min_angle.roll,
        dev->limits.max_angle.roll);
}

struct gimbal_dev *gimbal_alloc_udp(const char *driver_name, void *args) {
    struct gimbal_driver_info *drv;

    if (!driver_name)
        return NULL;

    drv = find_driver(driver_name);
    if (!drv || !drv->factory) {
        printf("[GIMBAL] No driver found: %s\n", driver_name);
        return NULL;
    }

    return drv->factory(driver_name, args);
}

int gimbal_set_mode(struct gimbal_dev *dev, gimbal_mode_t mode) {
    int ret = GIMBAL_OK;

    if (!dev)
        return GIMBAL_ERR_PARAM;

    if (dev->ops && dev->ops->set_mode)
        ret = dev->ops->set_mode(dev, mode);

    if (ret == GIMBAL_OK) {
        pthread_mutex_lock(&dev->cmd_lock);
        dev->mode = mode;
        if (mode == GIMBAL_MODE_SPEED) {
            memset(&dev->target, 0, sizeof(dev->target));
            dev->has_target = false;
        } else if (mode != GIMBAL_MODE_ANGLE_ABS
            && mode != GIMBAL_MODE_ANGLE_REL) {
            dev->has_target = false;
        }
        pthread_mutex_unlock(&dev->cmd_lock);
    }

    return ret;
}

int gimbal_set_target(struct gimbal_dev *dev, const gimbal_euler_t *target) {
    gimbal_mode_t mode;
    gimbal_euler_t effective;

    if (!dev || !target)
        return GIMBAL_ERR_PARAM;

    pthread_mutex_lock(&dev->cmd_lock);
    mode = dev->mode;
    effective = *target;

    if (mode == GIMBAL_MODE_ANGLE_REL) {
        pthread_mutex_lock(&dev->state_lock);
        effective.pitch = dev->cur_angle.pitch + target->pitch;
        effective.yaw = dev->cur_angle.yaw + target->yaw;
        effective.roll = dev->cur_angle.roll + target->roll;
        pthread_mutex_unlock(&dev->state_lock);
    }

    apply_limits_locked(dev, mode, &effective);

    pthread_mutex_unlock(&dev->cmd_lock);

    if (dev->ops && dev->ops->set_target) {
        int ret = dev->ops->set_target(dev, &effective);
        if (ret == GIMBAL_OK) {
            pthread_mutex_lock(&dev->cmd_lock);
            dev->target = effective;
            dev->has_target = true;
            pthread_mutex_unlock(&dev->cmd_lock);
        }
        return ret;
    }

    return GIMBAL_ERR_NOSYS;
}

int gimbal_set_limits(struct gimbal_dev *dev, const gimbal_limits_t *limits) {
    if (!dev || !limits)
        return GIMBAL_ERR_PARAM;

    pthread_mutex_lock(&dev->cmd_lock);
    dev->limits = *limits;
    dev->limits_valid = true;
    pthread_mutex_unlock(&dev->cmd_lock);

    if (dev->ops && dev->ops->set_limits)
        return dev->ops->set_limits(dev, limits);

    return GIMBAL_OK;
}

int gimbal_set_zoom(struct gimbal_dev *dev, gimbal_zoom_dir_t dir,
    uint8_t speed_level) {
    if (!dev)
        return GIMBAL_ERR_PARAM;

    if (dir < GIMBAL_ZOOM_STOP || dir > GIMBAL_ZOOM_OUT)
        return GIMBAL_ERR_PARAM;

    if (dev->ops && dev->ops->set_zoom)
        return dev->ops->set_zoom(dev, dir, speed_level);

    return GIMBAL_ERR_NOSYS;
}

int gimbal_get_state(struct gimbal_dev *dev, gimbal_euler_t *out_angle,
    gimbal_euler_t *out_speed) {
    int ret;
    double age_s;

    if (!dev)
        return GIMBAL_ERR_PARAM;

    pthread_mutex_lock(&dev->state_lock);
    if (out_angle)
        *out_angle = dev->cur_angle;
    if (out_speed)
        *out_speed = dev->cur_speed;
    age_s = dev->has_feedback
        ? (gimbal_now_monotonic_s() - dev->last_feedback_time_s)
        : 1e9;
    ret = (dev->has_feedback && age_s <= 0.5) ? GIMBAL_OK : GIMBAL_ERR_TIMEOUT;
    pthread_mutex_unlock(&dev->state_lock);

    return ret;
}

bool gimbal_is_stable(struct gimbal_dev *dev, float threshold_deg) {
    gimbal_mode_t mode;
    gimbal_euler_t cur_angle;
    gimbal_euler_t cur_speed;
    gimbal_euler_t target;
    bool has_feedback;
    bool has_target;
    double age_s;

    if (!dev)
        return false;

    pthread_mutex_lock(&dev->state_lock);
    cur_angle = dev->cur_angle;
    cur_speed = dev->cur_speed;
    has_feedback = dev->has_feedback;
    age_s = has_feedback
        ? (gimbal_now_monotonic_s() - dev->last_feedback_time_s)
        : 1e9;
    pthread_mutex_unlock(&dev->state_lock);

    pthread_mutex_lock(&dev->cmd_lock);
    mode = dev->mode;
    target = dev->target;
    has_target = dev->has_target;
    pthread_mutex_unlock(&dev->cmd_lock);

    if (!has_feedback || age_s > 0.5)
        return false;

    if (fabsf(cur_speed.pitch) > 1.0f || fabsf(cur_speed.yaw) > 1.0f ||
        fabsf(cur_speed.roll) > 1.0f)
        return false;

    if (mode == GIMBAL_MODE_SPEED) {
        return fabsf(target.pitch) <= 0.1f &&
            fabsf(target.yaw) <= 0.1f &&
            fabsf(target.roll) <= 0.1f;
    }

    if (!has_target)
        return false;

    return fabsf(gimbal_delta_deg(target.pitch, cur_angle.pitch)) <= threshold_deg
        && fabsf(gimbal_delta_deg(target.yaw, cur_angle.yaw)) <= threshold_deg
        && fabsf(gimbal_delta_deg(target.roll, cur_angle.roll)) <= threshold_deg;
}

void gimbal_tick(struct gimbal_dev *dev, float dt_s) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->tick)
        dev->ops->tick(dev, dt_s);
}

void gimbal_free(struct gimbal_dev *dev) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->free) {
        dev->ops->free(dev);
        return;
    }

    gimbal_dev_free_default(dev);
}
