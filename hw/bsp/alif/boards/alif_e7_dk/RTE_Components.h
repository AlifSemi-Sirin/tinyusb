#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

#define RTE_Drivers_GPIO 1

/*
 * Define the Device Header File: 
 */
#if defined(CORE_M55_HE)
#define CMSIS_device_header "M55_HE.h"
#elif defined(CORE_M55_HP)
#define CMSIS_device_header "M55_HP.h"
#else
#error "Unsupported core!"
#endif



#endif /* RTE_COMPONENTS_H */