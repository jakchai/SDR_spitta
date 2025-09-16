#!/bin/sh
$CC -I$SDKTARGETSYSROOT/usr/include -L$SDKTARGETSYSROOT/usr/lib -Wall -Wextra -pedantic -Wstrict-prototypes -o $1 $1.c -liio -lm 
