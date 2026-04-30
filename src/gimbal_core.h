/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GIMBAL_CORE_H
#define GIMBAL_CORE_H

#include "../include/gimbal.h"
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

struct gimbal_ops {
    int (*set_mode)(struct gimbal_dev *dev, gimbal_mode_t mode);
    int (*set_target)(struct gimbal_dev *dev, const gimbal_euler_t *target);
    int (*set_limits)(struct gimbal_dev *dev, const gimbal_limits_t *limits);
    int (*set_zoom)(struct gimbal_dev *dev, gimbal_zoom_dir_t dir,
                    uint8_t speed_level);
    void (*tick)(struct gimbal_dev *dev, float dt_s);
    void (*free)(struct gimbal_dev *dev);
};

struct gimbal_dev {
    char *name;

    const struct gimbal_ops *ops;
    void *priv_data;

    pthread_mutex_t state_lock;
    pthread_mutex_t cmd_lock;

    gimbal_mode_t mode;
    gimbal_euler_t cur_angle;
    gimbal_euler_t cur_speed;
    gimbal_euler_t target;

    gimbal_limits_t limits;
    bool limits_valid;
    bool has_target;
    bool has_feedback;

    double last_feedback_time_s;
    uint8_t last_cmd_ack;
    uint8_t guide_status;
    uint8_t servo_status;
};

typedef struct gimbal_dev *(*gimbal_factory_t)(const char *name, void *args);

struct gimbal_driver_info {
    const char *name;
    gimbal_factory_t factory;
    struct gimbal_driver_info *next;
};

void gimbal_driver_register(struct gimbal_driver_info *info);

#define REGISTER_GIMBAL_DRIVER(_name, _factory)                                \
    static struct gimbal_driver_info __drv_info_##_factory = {                 \
        .name = _name, .factory = _factory, .next = NULL};                     \
    __attribute__((constructor)) static void __auto_reg_##_factory(void) {     \
        gimbal_driver_register(&__drv_info_##_factory);                        \
    }

struct gimbal_dev *gimbal_dev_alloc(const char *name, size_t priv_size);
void gimbal_dev_free_default(struct gimbal_dev *dev);

double gimbal_now_monotonic_s(void);
float gimbal_normalize_deg(float angle_deg);
float gimbal_delta_deg(float target_deg, float current_deg);

#endif /* GIMBAL_CORE_H */
