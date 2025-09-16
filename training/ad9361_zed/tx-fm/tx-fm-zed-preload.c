
#define _GNU_SOURCE
#include <iio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define DEFAULT_BUFFER_TIME 0.1
#define DEFAULT_ATTENUATION -10

static struct iio_context *ctx = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_channel *lo_chan = NULL;
static struct iio_channel *phy_chan = NULL;
static struct iio_buffer *txbuf = NULL;

static volatile bool stop = false;

static int16_t *samples = NULL;
static size_t total_samples = 0;
static long long center_freq = 96500000;
static long long sample_rate = 2304000;
static const char *input_filename = NULL;

static double deviation_scale = 1.0;
static double time_per_sample = 1.0;
static double phase_accum = 0.0;

static void handle_sig(int sig) {
    stop = true;
}

static void wait_until(struct timespec *target) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, target, NULL);
}

static void time_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec += 1;
    }
}

static void modulate_sample(int16_t deviation, int16_t *i_sample, int16_t *q_sample) {
    double dev_hz = deviation * deviation_scale;
    double dphi = 2 * M_PI * dev_hz * time_per_sample;
    phase_accum = fmod(phase_accum + dphi, 2 * M_PI);
    *i_sample = (int16_t)(cos(phase_accum) * 0x7FFF);
    *q_sample = (int16_t)(sin(phase_accum) * 0x7FFF);
}

int main(int argc, char **argv) {
    int opt;
    while ((opt = getopt(argc, argv, "f:s:i:")) != -1) {
        switch (opt) {
            case 'f': center_freq = atoll(optarg); break;
            case 's': sample_rate = atoll(optarg); break;
            case 'i': input_filename = optarg; break;
            default:
                fprintf(stderr, "Usage: %s -f freq -s samplerate -i input.raw\n", argv[0]);
                return 1;
        }
    }

    if (!input_filename) {
        fprintf(stderr, "Input file is required.\n");
        return 1;
    }

    signal(SIGINT, handle_sig);

    FILE *fp = fopen(input_filename, "rb");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    rewind(fp);

    samples = malloc(fsize);
    if (!samples) {
        perror("malloc");
        fclose(fp);
        return 1;
    }

    total_samples = fread(samples, sizeof(int16_t), fsize / 2, fp);
    fclose(fp);

    time_per_sample = 1.0 / sample_rate;
    deviation_scale = 7500.0 / 32767.0;

    ctx = iio_create_default_context();
    if (!ctx) {
        fprintf(stderr, "Could not create IIO context\n");
        return 1;
    }

    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_device *tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!phy || !tx) {
        fprintf(stderr, "Could not find IIO devices\n");
        return 1;
    }

    phy_chan = iio_device_find_channel(phy, "voltage0", true);
    lo_chan = iio_device_find_channel(phy, "altvoltage1", true);
    tx0_i = iio_device_find_channel(tx, "voltage0", true);
    tx0_q = iio_device_find_channel(tx, "voltage1", true);

    iio_channel_attr_write_longlong(lo_chan, "frequency", center_freq);
    iio_channel_attr_write_longlong(lo_chan, "powerdown", 0);
    iio_channel_attr_write_double(phy_chan, "hardwaregain", DEFAULT_ATTENUATION);
    iio_channel_attr_write_longlong(phy_chan, "sampling_frequency", sample_rate);

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    size_t buffer_size = (size_t)(DEFAULT_BUFFER_TIME * sample_rate);
    txbuf = iio_device_create_buffer(tx, buffer_size, false);
    if (!txbuf) {
        fprintf(stderr, "Failed to create buffer\n");
        return 1;
    }

    while (!stop) {
        phase_accum = 0.0;
        size_t index = 0;
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);

        while (!stop && index < total_samples) {
            ptrdiff_t p_inc = iio_buffer_step(txbuf);
            char *p_end = iio_buffer_end(txbuf);
            char *p = iio_buffer_first(txbuf, tx0_i);

            for (; p < p_end && index < total_samples; p += p_inc) {
                int16_t i, q;
                modulate_sample(samples[index++], &i, &q);
                ((int16_t*)p)[0] = i;
                ((int16_t*)p)[1] = q;
            }

            iio_buffer_push(txbuf);
            time_add_ns(&t, DEFAULT_BUFFER_TIME * 1e9);
            wait_until(&t);
        }
    }

    iio_channel_attr_write_longlong(lo_chan, "powerdown", 1);
    iio_buffer_destroy(txbuf);
    iio_channel_disable(tx0_i);
    iio_channel_disable(tx0_q);
    iio_context_destroy(ctx);
    free(samples);
    return 0;
}

