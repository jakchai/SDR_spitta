// tx-fm-zed.c : FM transmitter for ZedBoard + FMCOMMS2 (no Pluto dependency)
// Build: gcc -o tx-fm-zed tx-fm-zed.c -liio -lm

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <iio.h>

#define MAX_SAMPLE_VALUE 0x7FFF
#define DEFAULT_BANDWIDTH 200000  // 200 kHz
#define DEFAULT_ATTENUATION -10   // -10 dB TX gain
#define DEFAULT_BUFFER_TIME 0.04  // 40ms buffer

static volatile bool stop = false;

void signal_handler(int signum) {
    stop = true;
}

int16_t get_next_sample(void) {
    int16_t value;
    if (read(STDIN_FILENO, &value, 2) != 2) {
        stop = true;
        return 0;
    }
    return value;
}

void modulate_sample(int16_t deviation, int16_t* i_sample, int16_t* q_sample,
                     double* signal, double deviation_scale, double time_per_sample) {
    double freq_hz = deviation * deviation_scale;
    double phase_increment = 2 * M_PI * freq_hz * time_per_sample;
    *signal = fmod(*signal + phase_increment, 2 * M_PI);
    *i_sample = (int16_t)(cos(*signal) * MAX_SAMPLE_VALUE);
    *q_sample = (int16_t)(sin(*signal) * MAX_SAMPLE_VALUE);
}

int main(int argc, char** argv) {
    long long center_freq = -1, sample_rate = -1;
    double deviation_hz = 10000;

    // Parse arguments
    int opt;
    while ((opt = getopt(argc, argv, "f:s:d:")) != -1) {
        switch (opt) {
            case 'f': center_freq = atoll(optarg); break;
            case 's': sample_rate = atoll(optarg); break;
            case 'd': deviation_hz = atof(optarg); break;
            default:
                fprintf(stderr, "Usage: %s -f freq -s samplerate [-d deviation]\n", argv[0]);
                return 1;
        }
    }

    if (center_freq < 70000000 || center_freq > 6000000000) {
        fprintf(stderr, "Invalid frequency. Must be between 70 MHz and 6 GHz.\n");
        return 1;
    }
    if (sample_rate < 1000000 || sample_rate > 61440000) {
        fprintf(stderr, "Invalid sample rate. Must be between 1 MHz and 61.44 MHz.\n");
        return 1;
    }

    struct iio_context* ctx = iio_create_default_context();
    if (!ctx) {
        fprintf(stderr, "Failed to create IIO context.\n");
        return 1;
    }

    struct iio_device* tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    struct iio_device* phy_dev = iio_context_find_device(ctx, "ad9361-phy");
    if (!tx_dev || !phy_dev) {
        fprintf(stderr, "Missing required IIO devices.\n");
        return 1;
    }

    struct iio_channel *tx0_i = iio_device_find_channel(tx_dev, "voltage0", true);
    struct iio_channel *tx0_q = iio_device_find_channel(tx_dev, "voltage1", true);
    struct iio_channel *lo_chan = iio_device_find_channel(phy_dev, "altvoltage1", true);
    struct iio_channel *phy_chan = iio_device_find_channel(phy_dev, "voltage0", true);

    if (!tx0_i || !tx0_q || !lo_chan || !phy_chan) {
        fprintf(stderr, "Missing TX channels or LO channel.\n");
        return 1;
    }

    // Set frequency, gain, bandwidth
    iio_channel_attr_write_longlong(lo_chan, "frequency", center_freq);
    iio_channel_attr_write_longlong(lo_chan, "powerdown", 0);
    iio_channel_attr_write_double(phy_chan, "hardwaregain", DEFAULT_ATTENUATION);
    iio_channel_attr_write_longlong(phy_chan, "sampling_frequency", sample_rate);
    iio_channel_attr_write_longlong(phy_chan, "rf_bandwidth", DEFAULT_BANDWIDTH);

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    size_t buffer_len = (size_t)(DEFAULT_BUFFER_TIME * sample_rate);
    struct iio_buffer* txbuf = iio_device_create_buffer(tx_dev, buffer_len, false);
    if (!txbuf) {
        fprintf(stderr, "Could not create TX buffer\n");
        return 1;
    }

    double deviation_scale = deviation_hz / MAX_SAMPLE_VALUE;
    double time_per_sample = 1.0 / sample_rate;
    double signal_phase = 0.0;

    signal(SIGINT, signal_handler);
    fprintf(stderr, "Starting transmission at %.1f MHz\n", center_freq / 1e6);

    while (!stop) {
        ptrdiff_t p_inc = iio_buffer_step(txbuf);
        char* p_end = iio_buffer_end(txbuf);
        char* p_dat;

        for (p_dat = iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
            int16_t sample = get_next_sample();
            int16_t i, q;
            modulate_sample(sample, &i, &q, &signal_phase, deviation_scale, time_per_sample);
            ((int16_t*)p_dat)[0] = i;
            ((int16_t*)p_dat)[1] = q;
        }

        ssize_t nbytes = iio_buffer_push(txbuf);
        if (nbytes < 0) {
            fprintf(stderr, "Error pushing buffer\n");
            break;
        }
    }

    fprintf(stderr, "Stopping transmission\n");

    iio_channel_attr_write_longlong(lo_chan, "powerdown", 1); // 👈 เพิ่มบรรทัดนี้

    iio_buffer_destroy(txbuf);
    iio_channel_disable(tx0_i);
    iio_channel_disable(tx0_q);
    iio_context_destroy(ctx);
    return 0;
}

