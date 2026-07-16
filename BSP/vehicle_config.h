#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

/*
 * Vehicle drive hardware profile.
 *
 * 2WD: FL + FR motors/encoders are enabled. RL + RR resources are not
 *      initialized, and PA5/PA6/PA7 are available for SPI1.
 * 4WD: FL + FR + RL + RR motors/encoders are enabled. Encoder CH3 occupies
 *      PA6/PA7, so SPI1 cannot be enabled on its default pins.
 *
 * Change only VEHICLE_DRIVE_MODE when switching chassis hardware.
 */
#define VEHICLE_DRIVE_MODE_2WD             2U
#define VEHICLE_DRIVE_MODE_4WD             4U

#ifndef VEHICLE_DRIVE_MODE
#define VEHICLE_DRIVE_MODE                 VEHICLE_DRIVE_MODE_2WD
#endif

#if (VEHICLE_DRIVE_MODE == VEHICLE_DRIVE_MODE_2WD)
#define VEHICLE_REAR_DRIVE_ENABLE          0U
#define VEHICLE_SPI1_PINS_AVAILABLE        1U
#define VEHICLE_DRIVE_MODE_NAME            "2WD"
#elif (VEHICLE_DRIVE_MODE == VEHICLE_DRIVE_MODE_4WD)
#define VEHICLE_REAR_DRIVE_ENABLE          1U
#define VEHICLE_SPI1_PINS_AVAILABLE        0U
#define VEHICLE_DRIVE_MODE_NAME            "4WD"
#else
#error "VEHICLE_DRIVE_MODE must be VEHICLE_DRIVE_MODE_2WD or VEHICLE_DRIVE_MODE_4WD"
#endif

#endif /* __VEHICLE_CONFIG_H */
