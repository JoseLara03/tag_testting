#ifndef TAG_CMD_H_
#define TAG_CMD_H_

/* Registers the single NUS RX handler. Dispatches `pwr ...` locally and
 * forwards all other commands to the cal parser. Call once after cal_init(). */
void tag_cmd_init(void);

#endif /* TAG_CMD_H_ */
