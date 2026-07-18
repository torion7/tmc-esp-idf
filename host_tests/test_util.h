// Mini-Testgeruest fuer die Host-Tests: CHECK-Makros + Zaehler, kein
// Framework noetig (laeuft mit jedem gcc/clang).
#pragma once

#include <stdio.h>
#include <string.h>

static int test_fehler = 0;
static int test_checks = 0;

#define CHECK(cond) do { \
    test_checks++; \
    if (!(cond)) { \
        test_fehler++; \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ_U32(ist, soll) do { \
    test_checks++; \
    unsigned long i_ = (unsigned long)(ist), s_ = (unsigned long)(soll); \
    if (i_ != s_) { \
        test_fehler++; \
        fprintf(stderr, "FAIL %s:%d: %s = %lu, erwartet %lu\n", \
                __FILE__, __LINE__, #ist, i_, s_); \
    } \
} while (0)

#define CHECK_EQ_I32(ist, soll) do { \
    test_checks++; \
    long i_ = (long)(ist), s_ = (long)(soll); \
    if (i_ != s_) { \
        test_fehler++; \
        fprintf(stderr, "FAIL %s:%d: %s = %ld, erwartet %ld\n", \
                __FILE__, __LINE__, #ist, i_, s_); \
    } \
} while (0)

#define CHECK_NEAR(ist, soll, eps) do { \
    test_checks++; \
    double i_ = (double)(ist), s_ = (double)(soll); \
    double d_ = i_ > s_ ? i_ - s_ : s_ - i_; \
    if (d_ > (eps)) { \
        test_fehler++; \
        fprintf(stderr, "FAIL %s:%d: %s = %g, erwartet %g (+/-%g)\n", \
                __FILE__, __LINE__, #ist, i_, s_, (double)(eps)); \
    } \
} while (0)

static inline int test_report(const char *name)
{
    if (test_fehler) {
        fprintf(stderr, "%s: %d/%d Checks FEHLGESCHLAGEN\n",
                name, test_fehler, test_checks);
        return 1;
    }
    printf("%s: %d Checks ok\n", name, test_checks);
    return 0;
}
