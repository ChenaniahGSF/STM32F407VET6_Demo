/*
 * getopt_long.h
 *
 *  Created on: Oct 13, 2025
 *      Author: Diamo
 */

#ifndef UTILITY_INC_GETOPT_LONG_H_
#define UTILITY_INC_GETOPT_LONG_H_

#define no_argument		0
#define required_argument	1
#define optional_argument	2

/* Long option structure structure definition */
struct option {
    const char *name;    /* Long option name */
    int         has_arg; /* 0=no argument, 1=required argument, 2=optional argument */
    int        *flag;    /* If non-NULL, store val here and return 0 */
    int         val;     /* Short option value or custom return value */
};

/* Now declare getopt_long using the global struct option */
extern int optind;
extern char *optarg;
extern int opterr;
extern int optopt;

void getopt_reset(void);
int getopt_long(int argc, char *const argv[], const char *optstring, const struct option *longopts, int *longindex);

#endif /* UTILITY_INC_GETOPT_LONG_H_ */
