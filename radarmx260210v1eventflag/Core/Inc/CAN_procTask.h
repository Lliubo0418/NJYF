#ifndef __CAN_PROC_TASK_H
#define __CAN_PROC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void StartCAN_procTask(void *argument);
void heartCallback(void *argument);

#ifdef __cplusplus
}
#endif

#endif
