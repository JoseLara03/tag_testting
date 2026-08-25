#include "phy_config.h"

/*
 * TX Power Configuration Settings
 */
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
dwt_txconfig_t txconfig_options = {
    0x34,       /* PG delay. */
    0xffffffff,/* TX power. */
    0x0         /*PG count*/
};

dwt_txconfig_t txconfig_options_ch9 = {
    0x34,       /* PG delay. */
    0xfefefefe, /* TX power. */
    0x0         /*PG count*/
};

/*
 * Configuration options for the following parameters:
 * Channel: 5, 9
 * PRF: 64
 * Preamble Length: 64, 128, 512, 1024
 * Preamble Code: 3/4 for 16MHz PRf, 9/10/11/12 for 64MHz PRF
 * Data Rate: 0.85, 6.8
 * STS: Length 64
 */

#if CONFIG_OPTION == CONFIG_OPTION_01
/* Configuration option 01.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length (ignored when STS off) */
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_02
/* Configuration option 02.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_03
/* Configuration option 03.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_04
/* Configuration option 04.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 32, Preamble code 9, Data Rate 850k, STS Length 64
 *
 * Comment corrected 2026-08-25: it claimed PLEN 128 / PAC 8, but the struct
 * below has always been PLEN_1024 / PAC32 with a matching SFD timeout of
 * (1024 + 1 + 8 - 32). The CODE was right and internally consistent -- this is
 * the channel-9 twin of option 07 -- only the comment was stale. Nothing here
 * was changed; if you need a real PLEN-128 channel-9 option, add one rather
 * than editing this.
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC32,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (1024 + 1 + 8 - 32), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_05
/* Configuration option 05.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_06
/* Configuration option 06.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_07
/* Configuration option 07.  THE ACTIVE CONFIGURATION -- see CONFIG_OPTION.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 32, Preamble code 9, Data Rate 850k, STS Length 64
 *
 * PAC is 32, not the 8 this comment claimed until 2026-08-25. The SFD timeout
 * below must be derived from the PAC actually in the struct: it was
 * (1024 + 1 + 8 - 8) = 1025, i.e. computed for PAC8 while the struct ran
 * PAC32, which left the RX window open ~24 symbols longer than needed after a
 * false preamble detect. Harmless enough to go unnoticed, but the anchor's
 * src/uwb_phy.h -- the fixed PHY contract both sides must match EXACTLY --
 * carries (1025 + 8 - 32) = 1001, so the two repos disagreed on a value the
 * contract says is identical everywhere. 1001 is the correct one.
 * See ANCLA_ESP32S3/docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md
 * section 2.8. Do not "simplify" this back to - 8 without changing DWT_PAC32.
 */
dwt_config_t config_options = {
    5,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC32,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,        /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 32), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_08
/* Configuration option 08.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,        /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_09
/* Configuration option 09.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_10
/* Configuration option 10.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_11
/* Configuration option 11.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_12
/* Configuration option 12.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_13
/* Configuration option 13.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_14
/* Configuration option 14.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_15
/* Configuration option 15.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    10,                 /* TX preamble code. Used in TX only. */
    10,                 /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,        /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_16
/* Configuration option 16.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    9,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    10,                 /* TX preamble code. Used in TX only. */
    10,                 /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,        /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_17
/* Configuration option 17.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_18
/* Configuration option 18.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_19
/* Configuration option 19.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_20
/* Configuration option 20.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_21
/* Configuration option 21.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_22
/* Configuration option 22.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_23
/* Configuration option 23.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_24
/* Configuration option 24.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_25
/* Configuration option 25.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION ==  CONFIG_OPTION_26
/* Configuration option 26.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    3,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (64 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,   /* Mode 1 STS enabled */
    DWT_STS_LEN_64,   /* STS length*/
    DWT_PDOA_M0       /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_27
/* Configuration option 27.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_28
/* Configuration option 28.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_29
/* Configuration option 29.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_30
/* Configuration option 30.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_512,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    10,                /* TX preamble code. Used in TX only. */
    10,                /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (512 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_64,    /* STS length*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_31
/* Configuration option 31.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    10,                 /* TX preamble code. Used in TX only. */
    10,                 /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_32
/* Configuration option 32.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                  /* Channel number. */
    DWT_PLEN_1024,      /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    10,                 /* TX preamble code. Used in TX only. */
    10,                 /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
    DWT_STS_LEN_64,     /* STS length*/
    DWT_PDOA_M0         /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_33
/* Configuration option 33.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_128,   /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};

/* Configuration option SP3.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128, STS Mode 3
 */
dwt_config_t config_option_sp3 = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_ND,   /* Mode 3 STS (no data) enabled */
    DWT_STS_LEN_128,   /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};

/* Configuration option SP0.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, No STS
 */
dwt_config_t config_option_sp0 = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,  /* STS Off */
    DWT_STS_LEN_128,   /* Ignore value when STS is disabled */
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_34
/* Configuration option 34.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1,    /* Mode 1 STS enabled */
    DWT_STS_LEN_128,   /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_35
/* Configuration option 35.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,  /* No STS mode enabled (STS Mode 0) */
    DWT_STS_LEN_64,    /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_36
/* Configuration option 36.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    9,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,  /* No STS mode enabled (STS Mode 0) */
    DWT_STS_LEN_64,    /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_37
/* Configuration option 37.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 256
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    (DWT_STS_MODE_1 | DWT_STS_MODE_SDC), /* STS enabled */
    DWT_STS_LEN_256,                     /* Cipher length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M3                          /* PDOA mode 3 */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_38
/* Configuration option 38.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    3,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    (DWT_STS_MODE_1 | DWT_STS_MODE_SDC), /* STS enabled */
    DWT_STS_LEN_64,                      /* Cipher length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0                          /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_39
/* Configuration option 39.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,  /* No STS mode enabled (STS Mode 0) */
    DWT_STS_LEN_128,   /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_40
/* Configuration option 40.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 32, Preamble code 9, Data Rate 850K, STS Length 64
 */
dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_1024,     /* Preamble length. Used in TX only. */
    DWT_PAC32,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (1025 + 8 - 32),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,  /* No STS mode enabled (STS Mode 0) */
    DWT_STS_LEN_64,    /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0        /* PDOA mode off */
};
#endif

#if CONFIG_OPTION == CONFIG_OPTION_41
/* Configuration option 41.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
dwt_config_t config_options = {
    5,                /* Channel number. */
    DWT_PLEN_64,      /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (65 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1 | DWT_STS_MODE_SDC,  /* STS enabled */
    DWT_STS_LEN_64,                     /* (STS length  in blocks of 8) - 1*/
    DWT_PDOA_M0                         /* PDOA mode off */
};
#endif