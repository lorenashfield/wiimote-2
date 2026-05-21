/*
 * The runtime loop: a single FreeRTOS task that drives the remote.
 *
 * While connected it ticks at a fixed cadence, each tick sampling the input
 * sources and emitting the current mode's data report (continuously when the
 * host asked for it, otherwise only on change). This is the one place a new
 * sensor (accelerometer, IR, GPIO buttons) or actuator hooks in.
 */
#pragma once

/* Create the runtime task. Call once at startup; it idles until connected. */
void wiimote_runtime_start(void);
