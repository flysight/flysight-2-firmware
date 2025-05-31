// tests/test_time.c
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "mock_stm32.h"

// Include your time functions - you may need to copy/paste or adjust paths
#include "../FlySight/time.c"

// Your normalize function (copy from your implementation)
// Helper function to normalize nanoseconds and adjust time accordingly
void normalizeGNSSTime(const FS_GNSS_Data_t *data,
                       uint16_t *norm_year, uint8_t *norm_month, uint8_t *norm_day,
                       uint8_t *norm_hour, uint8_t *norm_min, uint8_t *norm_sec,
                       int32_t *norm_millis)
{
    // Convert original time to epoch seconds
    uint32_t epoch_seconds = mk_gmtime(data->year, data->month, data->day,
                                       data->hour, data->min, data->sec);

    // Convert nanoseconds to milliseconds (rounded)
    if (data->nano >= 0) {
    	*norm_millis = (data->nano + 500000) / 1000000;
    } else {
    	*norm_millis = (data->nano - 499999) / 1000000;
    }

    // Handle negative milliseconds by borrowing from seconds
    if (*norm_millis >= 1000) {
        *norm_millis -= 1000;
        ++epoch_seconds;
    }
    else if (*norm_millis < 0) {
        *norm_millis += 1000;
        --epoch_seconds;
    }

    // Convert back to date/time components
    gmtime_r(epoch_seconds, norm_year, norm_month, norm_day,
             norm_hour, norm_min, norm_sec);
}

// Simple test framework
#define TEST(name) \
    printf("Testing: %s\n", #name); \
    test_##name(); \
    printf("✓ %s passed\n\n", #name);

#define ASSERT_EQ(expected, actual) \
    if ((expected) != (actual)) { \
        printf("✗ FAIL: Expected %d, got %d\n", (int)(expected), (int)(actual)); \
        exit(1); \
    }

// Individual test functions
void test_positive_nano() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = 123456789
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);
    ASSERT_EQ(30, day);
    ASSERT_EQ(12, hour);
    ASSERT_EQ(30, min);
    ASSERT_EQ(45, sec);
    ASSERT_EQ(123, millis);
}

void test_nano_minus_one() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = -1
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    // -1 nanosecond rounds to 0 milliseconds, no borrowing needed
    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);
    ASSERT_EQ(30, day);
    ASSERT_EQ(12, hour);
    ASSERT_EQ(30, min);
    ASSERT_EQ(45, sec);  // Should stay at 45 seconds
    ASSERT_EQ(0, millis); // Should round to 0 ms
}

void test_nano_needs_borrowing() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = -500001
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);
    ASSERT_EQ(30, day);
    ASSERT_EQ(12, hour);
    ASSERT_EQ(30, min);
    ASSERT_EQ(44, sec);   // Should borrow 1 second
    ASSERT_EQ(999, millis); // Should be 500ms after borrowing
}

void test_large_negative_nano() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = -500000000  // -0.5 seconds
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);
    ASSERT_EQ(30, day);
    ASSERT_EQ(12, hour);
    ASSERT_EQ(30, min);
    ASSERT_EQ(44, sec);    // Should borrow 1 second
    ASSERT_EQ(500, millis); // Should be 500ms
}

void test_seconds_underflow() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 0,
        .nano = -500001
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);
    ASSERT_EQ(30, day);
    ASSERT_EQ(12, hour);
    ASSERT_EQ(29, min);    // Should underflow to previous minute
    ASSERT_EQ(59, sec);    // Should be 59 seconds
    ASSERT_EQ(999, millis);
}

void test_day_underflow() {
    FS_GNSS_Data_t data = {
        .year = 2024, .month = 6, .day = 1,
        .hour = 0, .min = 0, .sec = 0,
        .nano = -500001
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data, &year, &month, &day, &hour, &min, &sec, &millis);

    ASSERT_EQ(2024, year);
    ASSERT_EQ(5, month);   // Should underflow to May
    ASSERT_EQ(31, day);    // May has 31 days
    ASSERT_EQ(23, hour);
    ASSERT_EQ(59, min);
    ASSERT_EQ(59, sec);
    ASSERT_EQ(999, millis);
}

void test_rounding() {
    // Test rounding down
    FS_GNSS_Data_t data1 = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = 499999  // Should round down to 0
    };

    uint16_t year; uint8_t month, day, hour, min, sec; int32_t millis;
    normalizeGNSSTime(&data1, &year, &month, &day, &hour, &min, &sec, &millis);
    ASSERT_EQ(0, millis);

    // Test rounding up
    FS_GNSS_Data_t data2 = {
        .year = 2024, .month = 5, .day = 30,
        .hour = 12, .min = 30, .sec = 45,
        .nano = 500000  // Should round up to 1
    };

    normalizeGNSSTime(&data2, &year, &month, &day, &hour, &min, &sec, &millis);
    ASSERT_EQ(1, millis);
}

int main() {
    printf("Starting time normalization tests...\n\n");

    TEST(positive_nano);
    TEST(nano_minus_one);
    TEST(large_negative_nano);
    TEST(nano_needs_borrowing);
    TEST(seconds_underflow);
    TEST(day_underflow);
    TEST(rounding);

    printf("All tests passed! ✓\n");
    return 0;
}
