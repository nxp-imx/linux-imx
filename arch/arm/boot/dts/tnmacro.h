
#ifndef _tnmacro_h
#define _tnmacro_h

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Dirty Macro hacks for splitting gpio definitions to bank and number
 *
 * Copyright 2016 Technexion Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

 Usage:
   Provides macros that can split a gpio definition to bank and number.

   Define gpios with bank and number:

   #define EDM_GPIO gpio2 11

   then this can be defined as gpio with (for example with)

   gpio=<&EDM_GPIO 0>;

   and as interrupt as

   interrupt-parent = < &GPIO_BANK(EDM_GPIO) >;
   interrupts = < GPIO_NO(EDM_GPIO) 0 >;

*/

#define TN_LPAREN (
#define TN_RPAREN )
#define TN_COMMA ,

#define TN_CAT(L, R) TN_CAT_(L, R)
#define TN_CAT_(L, R) L ## R
#define TN_EXPAND(...) __VA_ARGS__

#define TN_SPLIT(D) TN_EXPAND(TN_CAT(TN_SPLIT_, D) TN_RPAREN)

#define TN_SPLIT_gpio0 TN_LPAREN gpio0 TN_COMMA
#define TN_SPLIT_gpio1 TN_LPAREN gpio1 TN_COMMA
#define TN_SPLIT_gpio2 TN_LPAREN gpio2 TN_COMMA
#define TN_SPLIT_gpio3 TN_LPAREN gpio3 TN_COMMA
#define TN_SPLIT_gpio4 TN_LPAREN gpio4 TN_COMMA
#define TN_SPLIT_gpio5 TN_LPAREN gpio5 TN_COMMA
#define TN_SPLIT_gpio6 TN_LPAREN gpio6 TN_COMMA
#define TN_SPLIT_gpio7 TN_LPAREN gpio7 TN_COMMA
#define TN_SPLIT_pca9554 TN_LPAREN pca9554 TN_COMMA

#define TN_FIRST(a, b) a
#define TN_SECOND(a, b) b

#define GPIO_NO(x) TN_EXPAND(TN_SECOND TN_SPLIT(x))
#define GPIO_BANK(x) TN_EXPAND(TN_FIRST TN_SPLIT(x))

#endif
