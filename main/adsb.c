#include "adsb.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#define ADSB_PREAMBLE_SAMPLES     16
#define ADSB_LONG_MESSAGE_BITS    112
#define ADSB_LONG_MESSAGE_BYTES   14
#define ADSB_LONG_MESSAGE_SAMPLES (ADSB_PREAMBLE_SAMPLES + ADSB_LONG_MESSAGE_BITS * 2)
#define ADSB_TAIL_SAMPLES         512
#define ADSB_CPR_MAX_AGE_MS       10000
#define ADSB_STALE_MS             120000
#define ADSB_MODE_S_GENERATOR     0x1FFF409U
#define ADSB_CPR_SCALE            131072.0
#define ADSB_PI                   3.14159265358979323846

typedef struct {
    bool valid;
    int lat;
    int lon;
    int64_t time_ms;
} cpr_frame_t;

typedef struct {
    adsb_aircraft_t aircraft;
    cpr_frame_t even;
    cpr_frame_t odd;
} aircraft_state_t;

typedef struct {
    char icao[7];
    char callsign[9];
    bool has_altitude;
    int altitude_feet;
    bool has_cpr;
    bool cpr_odd;
    int cpr_latitude;
    int cpr_longitude;
    bool has_speed;
    float speed_knots;
    bool has_heading;
    float heading_degrees;
} decoded_message_t;

struct adsb_receiver {
    uint32_t sample_rate_hz;
    adsb_aircraft_cb_t aircraft_cb;
    void *aircraft_cb_ctx;
    uint32_t *magnitudes;
    size_t magnitude_capacity;
    uint32_t tail[ADSB_TAIL_SAMPLES];
    size_t tail_count;
    aircraft_state_t aircraft[ADSB_MAX_AIRCRAFT];
    uint32_t aircraft_count;
    uint64_t candidates;
    uint64_t valid_messages;
};

static const char *TAG = "adsb";

static uint32_t iq_magnitude(uint8_t i, uint8_t q)
{
    int centered_i = (int)i - 127;
    int centered_q = (int)q - 127;
    return (uint32_t)(centered_i * centered_i + centered_q * centered_q);
}

static bool ensure_magnitude_capacity(adsb_receiver_t *receiver, size_t needed)
{
    if (needed <= receiver->magnitude_capacity) {
        return true;
    }
    uint32_t *next = (uint32_t *)realloc(receiver->magnitudes, needed * sizeof(uint32_t));
    if (!next) {
        ESP_LOGW(TAG, "no memory for %u magnitudes", (unsigned)needed);
        return false;
    }
    receiver->magnitudes = next;
    receiver->magnitude_capacity = needed;
    return true;
}

static bool has_mode_s_preamble(const uint32_t *mags, int offset)
{
    uint32_t p0 = mags[offset + 0];
    uint32_t p1 = mags[offset + 2];
    uint32_t p2 = mags[offset + 7];
    uint32_t p3 = mags[offset + 9];
    uint32_t signal = (p0 + p1 + p2 + p3) / 4U;
    uint32_t noise = (mags[offset + 1] + mags[offset + 3] + mags[offset + 4] + mags[offset + 5] +
                      mags[offset + 6] + mags[offset + 8] + mags[offset + 10] + mags[offset + 11]) / 8U;

    if (signal < noise + 600U || signal * 2U < noise * 3U) {
        return false;
    }

    return p0 > mags[offset + 1] &&
           p1 > mags[offset + 1] &&
           p1 > mags[offset + 3] &&
           p2 > mags[offset + 6] &&
           p2 > mags[offset + 8] &&
           p3 > mags[offset + 8] &&
           p3 > mags[offset + 10];
}

static bool checksum_is_valid(const uint8_t *message)
{
    bool bits[ADSB_LONG_MESSAGE_BITS] = {0};
    for (int bit = 0; bit < ADSB_LONG_MESSAGE_BITS; bit++) {
        bits[bit] = ((message[bit / 8] >> (7 - (bit % 8))) & 0x01U) != 0;
    }

    for (int bit = 0; bit < ADSB_LONG_MESSAGE_BITS - 24; bit++) {
        if (!bits[bit]) {
            continue;
        }
        for (int poly_bit = 0; poly_bit < 25; poly_bit++) {
            bool poly = ((ADSB_MODE_S_GENERATOR >> (24 - poly_bit)) & 0x01U) != 0;
            bits[bit + poly_bit] = bits[bit + poly_bit] != poly;
        }
    }

    for (int bit = ADSB_LONG_MESSAGE_BITS - 24; bit < ADSB_LONG_MESSAGE_BITS; bit++) {
        if (bits[bit]) {
            return false;
        }
    }
    return true;
}

static int get_bits(const uint8_t *message, int first_bit, int bit_count)
{
    int value = 0;
    for (int bit = 0; bit < bit_count; bit++) {
        int source_bit = first_bit + bit;
        value = (value << 1) | ((message[source_bit / 8] >> (7 - (source_bit % 8))) & 0x01U);
    }
    return value;
}

static void decode_callsign(const uint8_t *message, char *out, size_t out_len)
{
    char text[9] = {0};
    for (int index = 0; index < 8; index++) {
        int code = get_bits(message, 40 + index * 6, 6);
        char ch = ' ';
        if (code >= 1 && code <= 26) {
            ch = (char)('A' + code - 1);
        } else if (code >= 48 && code <= 57) {
            ch = (char)('0' + code - 48);
        }
        text[index] = ch;
    }

    int end = 8;
    while (end > 0 && text[end - 1] == ' ') {
        end--;
    }
    size_t copy = MIN((size_t)end, out_len - 1);
    memcpy(out, text, copy);
    out[copy] = '\0';
}

static bool decode_altitude(int altitude_code, int *altitude_feet)
{
    if ((altitude_code & 0x10) == 0) {
        return false;
    }
    int n = ((altitude_code & 0x0FE0) >> 1) | (altitude_code & 0x000F);
    *altitude_feet = n * 25 - 1000;
    return true;
}

static int cpr_mod(int value, int modulo)
{
    int result = value % modulo;
    if (result < 0) {
        result += modulo;
    }
    return result;
}

static int cpr_nl(double latitude)
{
    double abs_lat = fabs(latitude);
    if (abs_lat < 1.0e-9) {
        return 59;
    }
    if (abs_lat >= 87.0) {
        return 1;
    }

    double numerator = 1.0 - cos(ADSB_PI / 30.0);
    double denominator = cos(abs_lat * ADSB_PI / 180.0);
    double value = 1.0 - numerator / (denominator * denominator);
    if (value < -1.0) {
        value = -1.0;
    } else if (value > 1.0) {
        value = 1.0;
    }
    int nl = (int)floor(2.0 * ADSB_PI / acos(value));
    return MAX(1, MIN(59, nl));
}

static bool decode_global_cpr(bool newest_odd,
                              int even_lat,
                              int even_lon,
                              int odd_lat,
                              int odd_lon,
                              double *latitude,
                              double *longitude)
{
    double even_lat_cpr = even_lat / ADSB_CPR_SCALE;
    double odd_lat_cpr = odd_lat / ADSB_CPR_SCALE;
    double even_lon_cpr = even_lon / ADSB_CPR_SCALE;
    double odd_lon_cpr = odd_lon / ADSB_CPR_SCALE;

    int j = (int)floor(59.0 * even_lat_cpr - 60.0 * odd_lat_cpr + 0.5);
    double rlat_even = 6.0 * (cpr_mod(j, 60) + even_lat_cpr);
    double rlat_odd = (360.0 / 59.0) * (cpr_mod(j, 59) + odd_lat_cpr);
    if (rlat_even >= 270.0) {
        rlat_even -= 360.0;
    }
    if (rlat_odd >= 270.0) {
        rlat_odd -= 360.0;
    }
    if (cpr_nl(rlat_even) != cpr_nl(rlat_odd)) {
        return false;
    }

    double selected_lat = newest_odd ? rlat_odd : rlat_even;
    int nl = cpr_nl(selected_lat);
    int ni = newest_odd ? MAX(nl - 1, 1) : MAX(nl, 1);
    int m = (int)floor(even_lon_cpr * (nl - 1) - odd_lon_cpr * nl + 0.5);
    double selected_lon = (360.0 / ni) * (cpr_mod(m, ni) + (newest_odd ? odd_lon_cpr : even_lon_cpr));
    if (selected_lon > 180.0) {
        selected_lon -= 360.0;
    }

    *latitude = selected_lat;
    *longitude = selected_lon;
    return true;
}

static bool decode_message(const uint8_t *message, decoded_message_t *decoded)
{
    int downlink_format = message[0] >> 3;
    if (downlink_format != 17 && downlink_format != 18) {
        return false;
    }

    memset(decoded, 0, sizeof(*decoded));
    snprintf(decoded->icao,
             sizeof(decoded->icao),
             "%02X%02X%02X",
             (unsigned)message[1],
             (unsigned)message[2],
             (unsigned)message[3]);

    int type_code = get_bits(message, 32, 5);
    if (type_code >= 1 && type_code <= 4) {
        decode_callsign(message, decoded->callsign, sizeof(decoded->callsign));
    } else if (type_code >= 9 && type_code <= 18) {
        decoded->has_altitude = decode_altitude(get_bits(message, 40, 12), &decoded->altitude_feet);
        decoded->has_cpr = true;
        decoded->cpr_odd = get_bits(message, 53, 1) != 0;
        decoded->cpr_latitude = get_bits(message, 54, 17);
        decoded->cpr_longitude = get_bits(message, 71, 17);
    } else if (type_code == 19) {
        int subtype = get_bits(message, 37, 3);
        if (subtype == 1 || subtype == 2) {
            int east_west = get_bits(message, 46, 10);
            int north_south = get_bits(message, 57, 10);
            if (east_west > 0 && north_south > 0) {
                east_west--;
                north_south--;
                if (get_bits(message, 45, 1) != 0) {
                    east_west = -east_west;
                }
                if (get_bits(message, 56, 1) != 0) {
                    north_south = -north_south;
                }
                decoded->speed_knots = sqrtf((float)(east_west * east_west + north_south * north_south));
                if (subtype == 2) {
                    decoded->speed_knots *= 4.0f;
                }
                decoded->heading_degrees = atan2f((float)east_west, (float)north_south) * 180.0f / (float)ADSB_PI;
                if (decoded->heading_degrees < 0.0f) {
                    decoded->heading_degrees += 360.0f;
                }
                decoded->has_speed = true;
                decoded->has_heading = true;
            }
        } else if (subtype == 3 || subtype == 4) {
            if (get_bits(message, 45, 1) != 0) {
                decoded->heading_degrees = (float)get_bits(message, 46, 10) * 360.0f / 1024.0f;
                decoded->has_heading = true;
            }
            int airspeed = get_bits(message, 57, 10);
            if (airspeed > 0) {
                airspeed--;
                decoded->speed_knots = subtype == 4 ? (float)airspeed * 4.0f : (float)airspeed;
                decoded->has_speed = true;
            }
        }
    }

    return true;
}

static aircraft_state_t *find_or_create_aircraft(adsb_receiver_t *receiver, const char *icao)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    int stale_index = -1;
    int oldest_index = 0;
    int64_t oldest_ms = INT64_MAX;

    for (uint32_t i = 0; i < receiver->aircraft_count; i++) {
        aircraft_state_t *state = &receiver->aircraft[i];
        if (strcmp(state->aircraft.icao, icao) == 0) {
            return state;
        }
        if (state->aircraft.last_seen_ms < oldest_ms) {
            oldest_ms = state->aircraft.last_seen_ms;
            oldest_index = (int)i;
        }
        if (stale_index < 0 && now_ms - state->aircraft.last_seen_ms > ADSB_STALE_MS) {
            stale_index = (int)i;
        }
    }

    int index;
    if (receiver->aircraft_count < ADSB_MAX_AIRCRAFT) {
        index = (int)receiver->aircraft_count++;
    } else {
        index = stale_index >= 0 ? stale_index : oldest_index;
    }

    memset(&receiver->aircraft[index], 0, sizeof(receiver->aircraft[index]));
    snprintf(receiver->aircraft[index].aircraft.icao, sizeof(receiver->aircraft[index].aircraft.icao), "%s", icao);
    return &receiver->aircraft[index];
}

static void handle_message(adsb_receiver_t *receiver, const uint8_t *message)
{
    decoded_message_t decoded;
    if (!decode_message(message, &decoded)) {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    aircraft_state_t *state = find_or_create_aircraft(receiver, decoded.icao);
    adsb_aircraft_t *aircraft = &state->aircraft;
    snprintf(aircraft->icao, sizeof(aircraft->icao), "%s", decoded.icao);
    aircraft->messages++;
    aircraft->last_seen_ms = now_ms;

    if (decoded.callsign[0]) {
        snprintf(aircraft->callsign, sizeof(aircraft->callsign), "%s", decoded.callsign);
    }
    if (decoded.has_altitude) {
        aircraft->has_altitude = true;
        aircraft->altitude_feet = decoded.altitude_feet;
    }
    if (decoded.has_speed) {
        aircraft->has_speed = true;
        aircraft->speed_knots = decoded.speed_knots;
    }
    if (decoded.has_heading) {
        aircraft->has_heading = true;
        aircraft->heading_degrees = decoded.heading_degrees;
    }
    if (decoded.has_cpr) {
        cpr_frame_t *frame = decoded.cpr_odd ? &state->odd : &state->even;
        frame->valid = true;
        frame->lat = decoded.cpr_latitude;
        frame->lon = decoded.cpr_longitude;
        frame->time_ms = now_ms;

        if (state->even.valid && state->odd.valid &&
            llabs(state->even.time_ms - state->odd.time_ms) <= ADSB_CPR_MAX_AGE_MS) {
            double lat = 0.0;
            double lon = 0.0;
            bool newest_odd = state->odd.time_ms > state->even.time_ms;
            if (decode_global_cpr(newest_odd,
                                  state->even.lat,
                                  state->even.lon,
                                  state->odd.lat,
                                  state->odd.lon,
                                  &lat,
                                  &lon)) {
                aircraft->has_position = true;
                aircraft->latitude = lat;
                aircraft->longitude = lon;
            }
        }
    }

    if (receiver->aircraft_cb) {
        receiver->aircraft_cb(aircraft, receiver->aircraft_cb_ctx);
    }
}

esp_err_t adsb_receiver_create(const adsb_receiver_config_t *config, adsb_receiver_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    adsb_receiver_t *receiver = (adsb_receiver_t *)calloc(1, sizeof(*receiver));
    ESP_RETURN_ON_FALSE(receiver, ESP_ERR_NO_MEM, TAG, "no memory");
    receiver->sample_rate_hz = config->sample_rate_hz ? config->sample_rate_hz : ADSB_DEFAULT_SAMPLE_RATE_HZ;
    receiver->aircraft_cb = config->aircraft_cb;
    receiver->aircraft_cb_ctx = config->aircraft_cb_ctx;
    *out = receiver;
    return ESP_OK;
}

void adsb_receiver_destroy(adsb_receiver_t *receiver)
{
    if (!receiver) {
        return;
    }
    free(receiver->magnitudes);
    free(receiver);
}

void adsb_receiver_reset(adsb_receiver_t *receiver)
{
    if (!receiver) {
        return;
    }
    receiver->tail_count = 0;
    receiver->aircraft_count = 0;
    receiver->candidates = 0;
    receiver->valid_messages = 0;
    memset(receiver->aircraft, 0, sizeof(receiver->aircraft));
}

void adsb_receiver_push_iq(adsb_receiver_t *receiver, const uint8_t *iq, size_t len)
{
    if (!receiver || !iq || len < 2) {
        return;
    }

    size_t new_samples = len / 2;
    size_t total_samples = receiver->tail_count + new_samples;
    if (total_samples < ADSB_LONG_MESSAGE_SAMPLES ||
        !ensure_magnitude_capacity(receiver, total_samples)) {
        return;
    }

    memcpy(receiver->magnitudes, receiver->tail, receiver->tail_count * sizeof(receiver->tail[0]));
    size_t mag_index = receiver->tail_count;
    for (size_t offset = 0; offset + 1 < len; offset += 2) {
        receiver->magnitudes[mag_index++] = iq_magnitude(iq[offset], iq[offset + 1]);
    }

    int end = (int)mag_index - ADSB_LONG_MESSAGE_SAMPLES;
    int scan_start = MAX(0, (int)receiver->tail_count - 240);
    for (int offset = scan_start; offset <= end; offset++) {
        if (!has_mode_s_preamble(receiver->magnitudes, offset)) {
            continue;
        }
        receiver->candidates++;

        uint8_t message[ADSB_LONG_MESSAGE_BYTES] = {0};
        for (int bit = 0; bit < ADSB_LONG_MESSAGE_BITS; bit++) {
            int sample = offset + ADSB_PREAMBLE_SAMPLES + bit * 2;
            bool one = receiver->magnitudes[sample] > receiver->magnitudes[sample + 1];
            if (one) {
                message[bit / 8] |= (uint8_t)(1U << (7 - (bit % 8)));
            }
        }

        int downlink_format = message[0] >> 3;
        if ((downlink_format == 17 || downlink_format == 18) && checksum_is_valid(message)) {
            receiver->valid_messages++;
            handle_message(receiver, message);
            offset += ADSB_LONG_MESSAGE_SAMPLES - 1;
        }
    }

    size_t keep = MIN((size_t)ADSB_TAIL_SAMPLES, mag_index);
    memcpy(receiver->tail, receiver->magnitudes + mag_index - keep, keep * sizeof(receiver->tail[0]));
    receiver->tail_count = keep;
}

void adsb_receiver_get_status(adsb_receiver_t *receiver, adsb_status_t *out)
{
    if (!receiver || !out) {
        return;
    }
    out->candidates = receiver->candidates;
    out->valid_messages = receiver->valid_messages;
    out->aircraft_count = receiver->aircraft_count;
}
