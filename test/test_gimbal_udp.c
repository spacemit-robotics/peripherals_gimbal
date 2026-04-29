/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "gimbal.h"

static volatile bool g_running = true;

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

static const char *mode_name(gimbal_mode_t mode) {
    switch (mode) {
    case GIMBAL_MODE_OFF:
        return "OFF";
    case GIMBAL_MODE_ANGLE_ABS:
        return "ANGLE_ABS";
    case GIMBAL_MODE_ANGLE_REL:
        return "ANGLE_REL";
    case GIMBAL_MODE_SPEED:
        return "SPEED";
    case GIMBAL_MODE_FOLLOW:
        return "FOLLOW";
    case GIMBAL_MODE_FPV:
        return "FPV";
    case GIMBAL_MODE_LOCK:
        return "LOCK";
    case GIMBAL_MODE_CALIBRATE:
        return "CALIBRATE";
    default:
        return "UNKNOWN";
    }
}

static const char *zoom_name(gimbal_zoom_dir_t dir) {
    switch (dir) {
    case GIMBAL_ZOOM_STOP:
        return "ZOOM_STOP";
    case GIMBAL_ZOOM_IN:
        return "ZOOM_IN";
    case GIMBAL_ZOOM_OUT:
        return "ZOOM_OUT";
    default:
        return "ZOOM_UNKNOWN";
    }
}

static void usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("\n");
    printf("Options:\n");
    printf("  --ip <addr>         Gimbal IP (default: 192.168.44.160)\n");
    printf("  --port <num>        Gimbal port (default: 10000)\n");
    printf("  --bind-ip <addr>    Local bind IP (default: 0.0.0.0)\n");
    printf("  --bind-port <num>   Local bind port (default: 10000)\n");
    printf("  -h, --help          Show this help message\n");
    printf("\n");
    printf("Demo sequence:\n");
    printf("  1. Center\n");
    printf("  2. Absolute guide\n");
    printf("  3. Relative guide\n");
    printf("  4. Speed mode yaw=12 deg/s for 2s\n");
    printf("  5. Visible zoom in / stop / zoom out / stop\n");
    printf("  6. Switch to heading lock\n");
    printf("\n");
    printf("Press Ctrl+C to stop early.\n");
}

static int send_mode(struct gimbal_dev *dev, gimbal_mode_t mode) {
    int ret;

    printf("[TX-API] set_mode(%s)\n", mode_name(mode));
    ret = gimbal_set_mode(dev, mode);
    printf("[TX-API] set_mode(%s) -> %d\n", mode_name(mode), ret);
    return ret;
}

static int send_target(struct gimbal_dev *dev, const char *label,
        const gimbal_euler_t *target) {
    int ret;

    printf("[TX-API] %s target: pitch=%.2f yaw=%.2f roll=%.2f\n",
        label, target->pitch, target->yaw, target->roll);
    ret = gimbal_set_target(dev, target);
    printf("[TX-API] %s -> %d\n", label, ret);
    return ret;
}

static int send_zoom(struct gimbal_dev *dev, gimbal_zoom_dir_t dir,
        uint8_t speed_level) {
    int ret;

    printf("[TX-API] set_zoom(%s, speed=%u)\n", zoom_name(dir), speed_level);
    ret = gimbal_set_zoom(dev, dir, speed_level);
    printf("[TX-API] set_zoom(%s, speed=%u) -> %d\n",
        zoom_name(dir), speed_level, ret);
    return ret;
}

static void tick_for(struct gimbal_dev *dev, float duration_s,
    const char *label) {
    int elapsed_ms = 0;
    float total_ms = duration_s * 1000.0f;

    printf("\n[%s] %.1f s\n", label, duration_s);
    while (g_running && elapsed_ms < total_ms) {
        gimbal_tick(dev, 0.02f);
        usleep(20000);
        elapsed_ms += 20;

        if (elapsed_ms % 200 == 0) {
            gimbal_euler_t angle = {0.0f, 0.0f, 0.0f};
            gimbal_euler_t speed = {0.0f, 0.0f, 0.0f};
            int ret = gimbal_get_state(dev, &angle, &speed);
            if (ret == GIMBAL_OK) {
                printf(
                    "[RX-STATE] pitch=%7.2f yaw=%7.2f roll=%7.2f | "
                    "speed: pitch=%6.2f yaw=%6.2f roll=%6.2f | stable=%s\n",
                    angle.pitch,
                    angle.yaw,
                    angle.roll,
                    speed.pitch,
                    speed.yaw,
                    speed.roll,
                    gimbal_is_stable(dev, 1.5f) ? "yes" : "no");
            } else {
                printf("[RX-STATE] waiting feedback...\n");
            }
        }
    }
}

int main(int argc, char *argv[]) {
    const char *device_ip = "192.168.44.160";
    uint16_t device_port = 4900;
    const char *bind_ip = "0.0.0.0";
    uint16_t bind_port = 4900;
    struct gimbal_dev *dev;
    gimbal_euler_t target;

    struct {
        const char *bind_ip;
        uint16_t bind_port;
        const char *device_ip;
        uint16_t device_port;
        float resend_period_s;
    } cfg;

    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "--ip") == 0) && i + 1 < argc) {
            device_ip = argv[++i];
        } else if ((strcmp(argv[i], "--port") == 0) && i + 1 < argc) {
            device_port = (uint16_t)atoi(argv[++i]);
        } else if ((strcmp(argv[i], "--bind-ip") == 0) && i + 1 < argc) {
            bind_ip = argv[++i];
        } else if ((strcmp(argv[i], "--bind-port") == 0) && i + 1 < argc) {
            bind_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0
                || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cfg.bind_ip = bind_ip;
    cfg.bind_port = bind_port;
    cfg.device_ip = device_ip;
    cfg.device_port = device_port;
    cfg.resend_period_s = 0.02f;

    printf("=== Gimbal UDP Demo ===\n");
    printf("Remote: %s:%u\n", device_ip, device_port);
    printf("Local : %s:%u\n", bind_ip, bind_port);

    dev = gimbal_alloc_udp("drv_udp_TZ0xxx", &cfg);
    if (!dev) {
        fprintf(stderr, "Failed to create gimbal device\n");
        return 1;
    }

    tick_for(dev, 1.0f, "Warmup / receive feedback");

    if (g_running) {
        printf("\n[1] Center\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "center", &target) != GIMBAL_OK)
            fprintf(stderr, "  center command failed\n");
        tick_for(dev, 3.0f, "Centering");
    }

    if (g_running) {
        printf("\n[2] Absolute guide\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = -70.0f;
        target.yaw = 58.0f;
        target.roll = 0.0f;
        if (send_target(dev, "absolute", &target) != GIMBAL_OK)
            fprintf(stderr, "  absolute target failed\n");
        tick_for(dev, 4.0f, "Absolute target");
    }

    if (g_running) {
        printf("\n[3] Relative guide\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_REL) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_REL mode failed\n");
        target.pitch = 80.0f;
        target.yaw = -20.0f;
        target.roll = 0.0f;
        if (send_target(dev, "relative", &target) != GIMBAL_OK)
            fprintf(stderr, "  relative target failed\n");
        tick_for(dev, 4.0f, "Relative target");
    }

    if (g_running) {
        printf("\n[4] Speed mode yaw +12 deg/s\n");
        if (send_mode(dev, GIMBAL_MODE_SPEED) != GIMBAL_OK)
            fprintf(stderr, "  set SPEED mode failed\n");
        target.pitch = 0.0f;
        target.yaw = 12.0f;
        target.roll = 0.0f;
        if (send_target(dev, "speed", &target) != GIMBAL_OK)
            fprintf(stderr, "  speed target failed\n");
        tick_for(dev, 2.0f, "Speed command");

        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "speed-stop", &target) != GIMBAL_OK)
            fprintf(stderr, "  stop speed command failed\n");
        tick_for(dev, 1.5f, "Speed stop");
    }

    if (g_running) {
        printf("\n[1] Center\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "center", &target) != GIMBAL_OK)
            fprintf(stderr, "  center command failed\n");
        tick_for(dev, 3.0f, "Centering");
    }

    if (g_running) {
        printf("\n[5] Visible zoom test\n");
        if (send_zoom(dev, GIMBAL_ZOOM_IN, 4) != GIMBAL_OK)
            fprintf(stderr, "  zoom in failed\n");
        tick_for(dev, 1.5f, "Zoom in");

        if (send_zoom(dev, GIMBAL_ZOOM_STOP, 0) != GIMBAL_OK)
            fprintf(stderr, "  zoom stop failed\n");
        tick_for(dev, 1.0f, "Zoom stop");

        if (send_zoom(dev, GIMBAL_ZOOM_OUT, 4) != GIMBAL_OK)
            fprintf(stderr, "  zoom out failed\n");
        tick_for(dev, 1.5f, "Zoom out");

        if (send_zoom(dev, GIMBAL_ZOOM_STOP, 0) != GIMBAL_OK)
            fprintf(stderr, "  final zoom stop failed\n");
        tick_for(dev, 1.0f, "Final zoom stop");
    }

    if (g_running) {
        printf("\n[6] Heading lock\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        tick_for(dev, 2.0f, "Heading lock");
    }

    printf("\nCleaning up...\n");
    gimbal_free(dev);
    return 0;
}
