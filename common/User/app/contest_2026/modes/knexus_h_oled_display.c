#include "knexus_h_hooks.h"
#include "knexus_config.h"
#include "OLED.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if KNEXUS_H_TASK_ENABLE && KNEXUS_H_OLED_ENABLE

volatile uint32_t knexus_h_oled_displayed_ms;
volatile uint8_t knexus_h_oled_display_state;

static bool s_init_attempted;
static bool s_was_running;
static uint32_t s_last_bucket = UINT32_MAX;
static uint32_t s_limit_ms = KNEXUS_H_SCORE_TIME_LIMIT_MS;
static char s_cache[4][17];

enum {
    H_OLED_READY = 0,
    H_OLED_RUNNING = 1,
    H_OLED_PASS = 2,
    H_OLED_OVER = 3,
    H_OLED_STOPPED = 4,
};

void knexus_h_display_invalidate(void)
{
    memset(s_cache, 0, sizeof(s_cache));
    s_last_bucket = UINT32_MAX;
    s_was_running = false;
    knexus_h_oled_display_state = 0xFFU;
    if (OLED_IsReady()) OLED_Clear();
}

static void make_line(char out[17], const char *text)
{
    uint8_t i = 0U;
    while (i < 16U && text != NULL && text[i] != '\0') {
        out[i] = text[i];
        i++;
    }
    while (i < 16U) out[i++] = ' ';
    out[16] = '\0';
}

static void show_line(uint8_t line, const char *text)
{
    char next[17];
    make_line(next, text);
    for (uint8_t i = 0U; i < 16U; ++i) {
        if (s_cache[line - 1U][i] != next[i]) {
            OLED_ShowChar(line, (uint8_t)(i + 1U), next[i]);
            s_cache[line - 1U][i] = next[i];
        }
    }
}

static void format_time(char out[17], uint32_t elapsed_ms)
{
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t millis = elapsed_ms % 1000U;
    if (seconds > 999U) seconds = 999U;
    make_line(out, "TIME 000.000s");
    out[5] = (char)('0' + (seconds / 100U) % 10U);
    out[6] = (char)('0' + (seconds / 10U) % 10U);
    out[7] = (char)('0' + seconds % 10U);
    out[9] = (char)('0' + (millis / 100U) % 10U);
    out[10] = (char)('0' + (millis / 10U) % 10U);
    out[11] = (char)('0' + millis % 10U);
}

static void format_limit(char out[17], uint32_t limit_ms)
{
    uint32_t seconds = limit_ms / 1000U;
    uint32_t millis = limit_ms % 1000U;
    if (seconds > 999U) seconds = 999U;
    make_line(out, "LIMIT 000.000s");
    out[6] = (char)('0' + (seconds / 100U) % 10U);
    out[7] = (char)('0' + (seconds / 10U) % 10U);
    out[8] = (char)('0' + seconds % 10U);
    out[10] = (char)('0' + (millis / 100U) % 10U);
    out[11] = (char)('0' + (millis / 10U) % 10U);
    out[12] = (char)('0' + millis % 10U);
}

void knexus_h_display_set_limit_ms(uint32_t limit_ms)
{
    s_limit_ms = limit_ms;
    s_last_bucket = UINT32_MAX;
    if (OLED_IsReady()) {
        char line[17];
        format_limit(line, s_limit_ms);
        show_line(4U, line);
    }
}

void knexus_h_display_time_ms(uint32_t elapsed_ms,
                              bool running,
                              bool completed)
{
    if (!s_init_attempted) {
        s_init_attempted = true;
        memset(s_cache, 0, sizeof(s_cache));
        if (!OLED_Init()) return;
        show_line(1U, "KNEXUS H TASK");
        char limit_line[17];
        format_limit(limit_line, s_limit_ms);
        show_line(4U, limit_line);
    }
    if (!OLED_IsReady()) return;

    uint8_t state;
    if (completed) {
        state = (elapsed_ms <= s_limit_ms)
                    ? H_OLED_PASS : H_OLED_OVER;
    } else if (running) {
        state = H_OLED_RUNNING;
    } else if (s_was_running) {
        state = H_OLED_STOPPED;
    } else {
        state = H_OLED_READY;
    }

    uint32_t bucket = elapsed_ms / KNEXUS_H_OLED_UPDATE_MS;
    bool state_changed = state != knexus_h_oled_display_state;
    if (!state_changed && bucket == s_last_bucket) return;

    switch (state) {
        case H_OLED_RUNNING: show_line(2U, "STATE RUN"); break;
        case H_OLED_PASS:    show_line(2U, "STATE PASS"); break;
        case H_OLED_OVER:    show_line(2U, "STATE OVER"); break;
        case H_OLED_STOPPED: show_line(2U, "STATE STOP"); break;
        default:             show_line(2U, "STATE READY"); break;
    }

    char time_line[17];
    format_time(time_line, elapsed_ms);
    show_line(3U, time_line);

    s_last_bucket = bucket;
    s_was_running = running || s_was_running;
    knexus_h_oled_displayed_ms = elapsed_ms;
    knexus_h_oled_display_state = state;
}

#endif
