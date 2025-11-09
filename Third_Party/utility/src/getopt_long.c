/*
 * getopt_long.c
 *
 *  Created on: Oct 13, 2025
 *      Author: Diamo
 */

#include <string.h>
#include <stdio.h>
#include "getopt_long.h"
#include "logger.h"

/* Global variables, compatible with standard getopt */
int optind = 1;          /* Index of the next argument to process */
char *optarg = NULL;     /* Pointer to option argument argument */
int opterr = 1;          /* Whether to output error messages (1=output) */
int optopt = '?';        /* Unrecognized option */

/* Internal state variables */
static int optpos = 0;   /* Current position in short option processing */
static int longindex = 0;/* Index of matched long option */

/* Reset parsing state (for multiple parsing passes) */
void getopt_reset(void) {
    optind = 1;
    optarg = NULL;
    optpos = 0;
    longindex = 0;
}

/* Find matching long option */
static int find_long_option(int argc, char *const argv[],
                           const struct option *longopts, int *idx) {
    int i, match = -1;
    size_t len;

    if (argv[optind][2] == '\0') return -1; /* Only "--" marks end of options */

    len = strlen(&argv[optind][2]); /* Length of long option name */

    /* Search through long options list for matches */
    for (i = 0; longopts[i].name; i++) {
        if (strncmp(longopts[i].name, &argv[optind][2], len) != 0)
            continue;

        if (strlen(longopts[i].name) == len) {
            /* Exact match */
            *idx = i;
            return i;
        }
        if (match == -1) {
            /* First partial match */
            match = i;
        } else {
            /* Ambiguous match (multiple partial matches) */
            return -1;
        }
    }

    if (match != -1) {
        /* Unique partial match */
        *idx = match;
        return match;
    }

    return -1; /* No match found */
}

/* Main getopt_long implementation */
int getopt_long(int argc, char *const argv[], const char *optstring, const struct option *longopts, int *longindex) {
    int c;
    const char *cp;
    int option_index = 0;

    /* Reset state (first call or all arguments processed) */
    if (optind == 0) {
        optind = 1;
        optpos = 0;
    }

    /* No more arguments to process */
    if (optind >= argc) {
        optpos = 0;
        return -1;
    }

    /* Process non-option arguments or option terminator "--" */
    if (argv[optind][0] != '-' || argv[optind][1] == '\0') {
        return -1; /* Non-option argument, end of parsing */
    }

    /* Process long options (--xxx) */
    if (argv[optind][1] == '-') {
        optpos = 0;
        if (argv[optind][2] == '\0') {
            /* Arguments after "--" are treated as non-options */
            optind++;
            return -1;
        }

        /* Find matching long option */
        int idx = find_long_option(argc, argv, longopts, &option_index);
        if (idx == -1) {
            /* No matching long option found */
            if (opterr) {
                logger_error("%s: unrecognized option '--%s'\n", argv[0], &argv[optind][2]);
            }
            optind++;
            optopt = 0; /* Mark long option error */
            return '?';
        }

        /* Record long option index if needed */
        if (longindex != NULL) {
            *longindex = option_index;
        }

        const struct option *opt = &longopts[option_index];
        optind++;

        /* Process long options requiring arguments */
        if (opt->has_arg == 1) {
            if (optind >= argc) {
                /* Missing required argument */
                if (opterr) {
                	logger_error("%s: option '--%s' requires an argument\n", argv[0], opt->name);
                }
                optopt = opt->val;
                return '?';
            }
            optarg = argv[optind++];
        }

        /* Determine return value based on flag */
        if (opt->flag) {
            *opt->flag = opt->val;
            return 0;
        } else {
            return opt->val;
        }
    }

    /* Process short options (-x) */
    if (optpos == 0) {
        optpos = 1; /* Start parsing from first character */
    }

    c = argv[optind][optpos++];
    cp = strchr(optstring, c);

    /* Unrecognized short option */
    if (cp == NULL || c == ':') {
        if (opterr) {
        	logger_error("%s: invalid option -- '%c'\n", argv[0], c);
        }
        optopt = c;
        /* Check if this is the last character of the option */
        if (argv[optind][optpos] == '\0') {
            optind++;
            optpos = 0;
        }
        return '?';
    }

    /* Process short options requiring arguments */
    if (cp[1] == ':') {
        /* Option argument may be immediately after option or as next argument */
        if (argv[optind][optpos] != '\0') {
            optarg = &argv[optind][optpos];
            optind++;
            optpos = 0;
        } else {
            if (optind + 1 >= argc) {
                /* Missing required argument */
                if (opterr) {
                	logger_error("%s: option requires an argument -- '%c'\n", argv[0], c);
                }
                optopt = c;
                optind++;
                optpos = 0;
                return (optstring[0] == ':') ? ':' : '?';
            }
            optarg = argv[optind + 1];
            optind += 2;
            optpos = 0;
        }
        return c;
    }

    /* Short options without arguments */
    if (argv[optind][optpos] == '\0') {
        optind++;
        optpos = 0;
    }
    return c;
}
