#ifndef UWB_H_
#define UWB_H_

#include <stdint.h>

int      uwb_init(int max_retries);
uint32_t uwb_get_dev_id(void);

#endif /* UWB_H_ */
