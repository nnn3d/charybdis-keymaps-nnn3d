RGB_MATRIX_EFFECT(DEFAULT_ANIM)
RGB_MATRIX_EFFECT(D_ANIM)
RGB_MATRIX_EFFECT(DM_ANIM)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS
#ifdef RGB_MATRIX_KEYPRESSES

bool run_default_effect(effect_params_t* params, bool red_mode, bool mouse_mode) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    uint8_t count = g_last_hit_tracker.count;
    uint8_t time;
    if (red_mode) {
        time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_config.speed, 1));
    } else {
        time = scale16by8(g_rgb_timer, qadd8(rgb_matrix_config.speed / 4, 1));
    }

    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();
        HSV hsv = rgb_matrix_config.hsv;
        hsv.v   = 0;
        for (uint8_t j = 0; j < count; j++) {
                int16_t  dx   = g_led_config.point[i].x - g_last_hit_tracker.x[j];
                int16_t  dy   = g_led_config.point[i].y - g_last_hit_tracker.y[j];
                uint8_t  dist = sqrt16(dx * dx + dy * dy);
                uint16_t tick = scale16by8(g_last_hit_tracker.tick[j], qadd8(rgb_matrix_config.speed, 1));

                uint16_t effect = tick + dist * 5;
                if (effect > 255) effect = 255;

                hsv.v = qadd8(hsv.v, 255 - effect);
        }

        hsv.h = g_led_config.point[i].y - time;
        if (red_mode) {
            // led_point_t k_rgb_matrix_center = {192, 12};
            int16_t  dx   = g_led_config.point[i].x - 192;
            int16_t  dy   = g_led_config.point[i].y - 12;
            uint8_t  dist = sqrt16(dx * dx + dy * dy);

            uint8_t mod = time == 0 ? 1 : time;
            mod = mul8(abs(mod - 128), 2);
            mod = scale8(mod, pow(scale8(dist, 7), 2));
            uint8_t red = 5 + mod + scale8(hsv.v, rgb_matrix_config.hsv.v - 5 - mod);
            uint8_t bg = 0;
            if (mouse_mode && g_led_config.point[i].x >= 112 && g_led_config.point[i].y >= 36) {
                bg = 1;
            }

            rgb_matrix_set_color(i, red, bg, bg);
        } else {
            hsv.s = add8(100, scale8(hsv.v, 50));
            hsv.v = add8(40, scale8(hsv.v, sub8(rgb_matrix_config.hsv.v, 40)));

            RGB rgb = rgb_matrix_hsv_to_rgb(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
        }
    }

    return rgb_matrix_check_finished_leds(led_max);
}


bool DEFAULT_ANIM(effect_params_t* params) {
    return run_default_effect(params, false, false);
}

bool D_ANIM(effect_params_t* params) {
    return run_default_effect(params, true, false);
}

bool DM_ANIM(effect_params_t* params) {
    return run_default_effect(params, true, true);
}


#endif // RGB_MATRIX_KEYPRESSES
#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
