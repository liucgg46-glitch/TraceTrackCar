#ifndef __PROJECT_STATUS_H
#define __PROJECT_STATUS_H

/* 与具体芯片和BSP无关的通用接口状态。 */
typedef enum {
    PROJECT_OK = 0,
    PROJECT_ERROR,
    PROJECT_BUSY,
    PROJECT_TIMEOUT,
    PROJECT_PARAM
} Project_Status_t;

#endif /* __PROJECT_STATUS_H */
