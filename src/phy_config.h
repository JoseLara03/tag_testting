#include "deca_device_api.h"

#ifndef PHY_CONFIG_H_
#define PHY_CONFIG_H

#define CONFIG_OPTION CONFIG_OPTION_07

/* Factory-reference antenna delay (device time units), split equally between
 * TX and RX. This is ONLY the seed/fallback used by the calibration routine on
 * an uncalibrated unit — the active value is now per-unit and stored in NVS by
 * src/cal.c (see the `cal <mm>` BLE command). Each unit must be calibrated
 * before ranging starts (CONFIG_OPTION-specific). */
#define TX_ANT_DLY 16371
#define RX_ANT_DLY 16371

/* Configuration option 01.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_01 1

/* Configuration option 02.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_02 2

/* Configuration option 03.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_03 3

/* Configuration option 04.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_04 4

/* Configuration option 05.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_05 5

/* Configuration option 06.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_06 6

/* Configuration option 07.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_07 7  

/* Configuration option 08.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_08 8

/* Configuration option 09.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_09 9

/* Configuration option 10.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_10 10

/* Configuration option 11.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_11 11

/* Configuration option 12.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_12 12

/* Configuration option 13.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_13 13

/* Configuration option 14.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_14 14

/* Configuration option 15.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_15 15

/* Configuration option 16.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_16 16

/* Configuration option 17.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_17 17

/* Configuration option 18.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_18 18

/* Configuration option 19.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_19 19

/* Configuration option 20.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_20 20

/* Configuration option 21.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
// #define CONFIG_OPTION_21 21

/* Configuration option 22.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_22 22

/* Configuration option 23.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_23 23

/* Configuration option 24.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_24 24

/* Configuration option 25.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_25 25

/* Configuration option 26.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_26 26

/* Configuration option 27.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_27 27

/* Configuration option 28.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_28 28

/* Configuration option 29.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_29 29

/* Configuration option 30.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_30 30

/* Configuration option 31.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_31 31

/* Configuration option 32.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_32 32

/* Configuration option 33.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
// #define CONFIG_OPTION_33 33

/* Configuration option 34.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
// #define CONFIG_OPTION_34 34

/* Configuration option 35.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_35 35

/* Configuration option 36.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_36 36

/* Configuration option 37.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 256
 */
// #define CONFIG_OPTION_37 37

/* Configuration option 38.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_38 38

/* Configuration option 39.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
// #define CONFIG_OPTION_39 39

/* Configuration option 40.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 32, Preamble code 9, Data Rate 850K, STS Length 64
 */
// #define CONFIG_OPTION_40 40

/* Configuration option 41.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
// #define CONFIG_OPTION_41 41

#endif