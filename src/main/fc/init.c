/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/debug_pin.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf_serial.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"

#include "cli/cli.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_spi.h"
#include "drivers/buttons.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/flash.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/mco.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/pin_pull_up_down.h"
#include "drivers/pwm_output.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/sbus_output.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/sdcard.h"
#include "drivers/sdio.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/freq.h"
#include "drivers/usb_io.h"
#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif
#include "drivers/vtx_common.h"
#include "drivers/vtx_rtc6705.h"
#include "drivers/vtx_table.h"

#include "fc/board_info.h"
#include "fc/dispatch.h"
#include "fc/init.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"
#include "fc/tasks.h"

#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "flight/governor.h"
#include "flight/rpm_filter.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/displayport_frsky_osd.h"
#include "io/displayport_max7456.h"
#include "io/displayport_msp.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/piniobox.h"
#include "io/rcdevice_cam.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_tramp.h"

#include "msc/emfat_file.h"
#ifdef USE_PERSISTENT_MSC_RTC
#include "msc/usbd_storage.h"
#endif

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/bus_quadspi.h"
#include "pg/flash.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/pin_pull_up_down.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"
#include "pg/vtx_io.h"
#include "pg/freq.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

uint8_t systemState = SYSTEM_STATE_INITIALISING;

#ifdef BUS_SWITCH_PIN
void busSwitchInit(void)
{
    IO_t busSwitchResetPin = IO_NONE;

    busSwitchResetPin = IOGetByTag(IO_TAG(BUS_SWITCH_PIN));
    IOInit(busSwitchResetPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(busSwitchResetPin, IOCFG_OUT_PP);

    // ENABLE
    IOLo(busSwitchResetPin);
}
#endif


static void configureSPIAndQuadSPI(void)
{
#ifdef USE_SPI
    spiPinConfigure(spiPinConfig(0));
#endif

    sensorsPreInit();

#ifdef USE_SPI
    spiPreinit();

#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1);
#endif
#ifdef USE_SPI_DEVICE_2
    spiInit(SPIDEV_2);
#endif
#ifdef USE_SPI_DEVICE_3
    spiInit(SPIDEV_3);
#endif
#ifdef USE_SPI_DEVICE_4
    spiInit(SPIDEV_4);
#endif
#ifdef USE_SPI_DEVICE_5
    spiInit(SPIDEV_5);
#endif
#ifdef USE_SPI_DEVICE_6
    spiInit(SPIDEV_6);
#endif
#endif // USE_SPI

#ifdef USE_QUADSPI
    quadSpiPinConfigure(quadSpiConfig(0));

#ifdef USE_QUADSPI_DEVICE_1
    quadSpiInit(QUADSPIDEV_1);
#endif
#endif // USE_QUAD_SPI
}

#ifdef USE_SDCARD
static void sdCardAndFSInit()
{
    sdcard_init(sdcardConfig());
    afatfs_init();
}
#endif

static void swdPinsInit(void)
{
    IO_t io = IOGetByTag(DEFIO_TAG_E(PA13)); // SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PA14)); // SWCLK
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PB3)); // SWO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
}

void init(void)
{
#ifdef SERIAL_PORT_COUNT
    printfSerialInit();
#endif

    systemInit();

    // Initialize task data as soon as possible. Has to be done before tasksInit(),
    // and any init code that may try to modify task behaviour before tasksInit().
    tasksInitData();

    // initialize IO (needed for all IO operations)
    IOInitGlobal();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

#if defined(USE_TARGET_CONFIG)
    // Call once before the config is loaded for any target specific configuration required to support loading the config
    targetConfiguration();
#endif

    enum {
        FLASH_INIT_ATTEMPTED            = (1 << 0),
        SD_INIT_ATTEMPTED               = (1 << 1),
        SPI_AND_QSPI_INIT_ATTEMPTED      = (1 << 2),
    };
    uint8_t initFlags = 0;

#ifdef CONFIG_IN_SDCARD

    //
    // Config in sdcard presents an issue with pin configuration since the pin and sdcard configs for the
    // sdcard are in the config which is on the sdcard which we can't read yet!
    //
    // FIXME We need to add configuration somewhere, e.g. bootloader image or reserved flash area, that can be read by the firmware.
    // it's currently possible for the firmware resource allocation to be wrong after the config is loaded if the user changes the settings.
    // This would cause undefined behaviour once the config is loaded.  so for now, users must NOT change sdio/spi configs needed for
    // the system to boot and/or to save the config.
    //
    // note that target specific SDCARD/SDIO/SPI/QUADSPI configs are
    // also not supported in USE_TARGET_CONFIG/targetConfigure() when using CONFIG_IN_SDCARD.
    //

    //
    // IMPORTANT: all default flash and pin configurations must be valid for the target after pgResetAll() is called.
    // Target designers must ensure other devices connected the same SPI/QUADSPI interface as the flash chip do not
    // cause communication issues with the flash chip.  e.g. use external pullups on SPI/QUADSPI CS lines.
    //

#ifdef TARGET_BUS_INIT
#error "CONFIG_IN_SDCARD and TARGET_BUS_INIT are mutually exclusive"
#endif

    pgResetAll();

#ifdef USE_SDCARD_SPI
    configureSPIAndQuadSPI();
    initFlags |= SPI_AND_QSPI_INIT_ATTEMPTED;
#endif

    sdCardAndFSInit();
    initFlags |= SD_INIT_ATTEMPTED;

    if (!sdcard_isInserted()) {
        failureMode(FAILURE_SDCARD_REQUIRED);
    }

    while (afatfs_getFilesystemState() != AFATFS_FILESYSTEM_STATE_READY) {
        afatfs_poll();

        if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL) {
            failureMode(FAILURE_SDCARD_INITIALISATION_FAILED);
        }
    }

#endif // CONFIG_IN_SDCARD

#ifdef CONFIG_IN_EXTERNAL_FLASH
    //
    // Config on external flash presents an issue with pin configuration since the pin and flash configs for the
    // external flash are in the config which is on a chip which we can't read yet!
    //
    // FIXME We need to add configuration somewhere, e.g. bootloader image or reserved flash area, that can be read by the firmware.
    // it's currently possible for the firmware resource allocation to be wrong after the config is loaded if the user changes the settings.
    // This would cause undefined behaviour once the config is loaded.  so for now, users must NOT change flash/pin configs needed for
    // the system to boot and/or to save the config.
    //
    // note that target specific FLASH/SPI/QUADSPI configs are
    // also not supported in USE_TARGET_CONFIG/targetConfigure() when using CONFIG_IN_EXTERNAL_FLASH.
    //

    //
    // IMPORTANT: all default flash and pin configurations must be valid for the target after pgResetAll() is called.
    // Target designers must ensure other devices connected the same SPI/QUADSPI interface as the flash chip do not
    // cause communication issues with the flash chip.  e.g. use external pullups on SPI/QUADSPI CS lines.
    //
    pgResetAll();

#ifdef TARGET_BUS_INIT
#error "CONFIG_IN_EXTERNAL_FLASH and TARGET_BUS_INIT are mutually exclusive"
#endif

    configureSPIAndQuadSPI();
    initFlags |= SPI_AND_QSPI_INIT_ATTEMPTED;


#ifndef USE_FLASH_CHIP
#error "CONFIG_IN_EXTERNAL_FLASH requires USE_FLASH_CHIP to be defined."
#endif

    bool haveFlash = flashInit(flashConfig());

    if (!haveFlash) {
        failureMode(FAILURE_EXTERNAL_FLASH_INIT_FAILED);
    }
    initFlags |= FLASH_INIT_ATTEMPTED;

#endif // CONFIG_IN_EXTERNAL_FLASH

    initEEPROM();

    if (!isEEPROMVersionValid() || !isEEPROMStructureValid() || !readEEPROM()) {
#if defined(USE_CUSTOM_DEFAULTS)
        if (hasCustomDefaults())
            resetConfigToCustomDefaults();
        else
#endif
            resetConfig();
        writeUnmodifiedConfigToEEPROM();
        systemResetHard();
    }

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

#if defined(USE_BOARD_INFO)
    initBoardInformation();
#endif

#ifdef USE_DEBUG_PIN
    dbgPinInit();
#endif

    debugMode = systemConfig()->debug_mode;
    debugAxis = systemConfig()->debug_axis;

#ifdef TARGET_PREINIT
    targetPreInit();
#endif

#if !defined(USE_FAKE_LED)
    ledInit(statusLedConfig());
#endif
    LED2_ON;

#if !defined(SIMULATOR_BUILD)
    EXTIInit();
#endif

#if defined(USE_BUTTONS)

    buttonsInit();

    delayMicroseconds(10);  // allow configuration to settle // XXX Could be removed, too?

    // Allow EEPROM reset with two-button-press without power cycling in DEBUG build
#ifdef DEBUG_BUILD
#define EEPROM_RESET_PRECONDITION true
#else
#define EEPROM_RESET_PRECONDITION (!isMPUSoftReset())
#endif

    if (EEPROM_RESET_PRECONDITION) {
#if defined(BUTTON_A_PIN) && defined(BUTTON_B_PIN)
        // two buttons required
        uint8_t secondsRemaining = 5;
        bool bothButtonsHeld;
        do {
            bothButtonsHeld = buttonAPressed() && buttonBPressed();
            if (bothButtonsHeld) {
                if (--secondsRemaining == 0) {
                    resetEEPROM(false);
#ifdef USE_PERSISTENT_OBJECTS
                    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
#endif
                    systemResetHard();
                }
                delay(1000);
                LED0_TOGGLE;
            }
        } while (bothButtonsHeld);
#endif
    }

#undef EEPROM_RESET_PRECONDITION

#endif // USE_BUTTONS

    // Note that spektrumBind checks if a call is immediately after
    // hard reset (including power cycle), so it should be called before
    // systemClockSetHSEValue and OverclockRebootIfNecessary, as these
    // may cause soft reset which will prevent spektrumBind not to execute
    // the bind procedure.

#if defined(USE_SPEKTRUM_BIND)
    if (featureIsEnabled(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
        case SERIALRX_SRXL:
            // Spektrum satellite binding if enabled on startup.
            // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
            // The rest of Spektrum initialization will happen later - via spektrumInit()
            spektrumBind(rxConfigMutable());
            break;
        }
    }
#endif

#if defined(STM32F4) || defined(STM32G4)
    // F4 has non-8MHz boards
    // G4 for Betaflight allow 24 or 27MHz oscillator
    systemClockSetHSEValue(systemConfig()->hseMhz * 1000000U);
#endif

#ifdef USE_OVERCLOCK
    OverclockRebootIfNecessary(systemConfig()->cpu_overclock);
#endif

    // Configure MCO output after config is stable
#ifdef USE_MCO
    // Note that mcoConfigure must be augmented with an additional argument to
    // indicate which device instance to configure when MCO and MCO2 are both supported

#if defined(STM32F4) || defined(STM32F7)
    // F4 and F7 support MCO on PA8 and MCO2 on PC9, but only MCO2 is supported for now
    mcoConfigure(MCODEV_2, mcoConfig(MCODEV_2));
#elif defined(STM32G4)
    // G4 only supports one MCO on PA8
    mcoConfigure(MCODEV_1, mcoConfig(MCODEV_1));
#else
#error Unsupported MCU
#endif
#endif // USE_MCO

#ifdef USE_TIMER
    timerInit();  // timer must be initialized before any channel is allocated
#endif

#ifdef BUS_SWITCH_PIN
    busSwitchInit();
#endif

#if defined(USE_UART) && !defined(SIMULATOR_BUILD)
    uartPinConfigure(serialPinConfig());
#endif

#if defined(AVOID_UART1_FOR_PWM_PPM)
    serialInit(featureIsEnabled(FEATURE_SOFTSERIAL),
            featureIsEnabled(FEATURE_RX_PPM) || featureIsEnabled(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART1 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(featureIsEnabled(FEATURE_SOFTSERIAL),
            featureIsEnabled(FEATURE_RX_PPM) || featureIsEnabled(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(featureIsEnabled(FEATURE_SOFTSERIAL),
            featureIsEnabled(FEATURE_RX_PPM) || featureIsEnabled(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(featureIsEnabled(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    mixerInit();

#ifdef USE_MOTOR
    motorInit();
    systemState |= SYSTEM_STATE_MOTORS_READY;
#endif

    if (0) {}
#if defined(USE_PPM)
    else if (featureIsEnabled(FEATURE_RX_PPM)) {
        ppmRxInit(ppmConfig());
    }
#endif
#if defined(USE_PWM)
    else if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
        pwmRxInit(pwmConfig());
    }
#endif

#ifdef USE_BEEPER
    beeperInit(beeperDevConfig());
#endif
/* temp until PGs are implemented. */
#if defined(USE_INVERTER) && !defined(SIMULATOR_BUILD)
    initInverters(serialPinConfig());
#endif


#ifdef TARGET_BUS_INIT
    targetBusInit();

#else

    // Depending on compilation options SPI/QSPI initialisation may already be done.
    if (!(initFlags & SPI_AND_QSPI_INIT_ATTEMPTED)) {
        configureSPIAndQuadSPI();
        initFlags |= SPI_AND_QSPI_INIT_ATTEMPTED;
    }

#if defined(USE_SDCARD_SDIO) && !defined(CONFIG_IN_SDCARD) && defined(STM32H7)
    sdioPinConfigure();
    SDIO_GPIO_Init();
#endif

#ifdef USE_USB_MSC
/* MSC mode will start after init, but will not allow scheduler to run,
 *  so there is no bottleneck in reading and writing data */
    mscInit();
    if (mscCheckBootAndReset() || mscCheckButton()) {
        ledInit(statusLedConfig());

#ifdef USE_SDCARD
        if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
            if (sdcardConfig()->mode) {
                if (!(initFlags & SD_INIT_ATTEMPTED)) {
                    sdCardAndFSInit();
                    initFlags |= SD_INIT_ATTEMPTED;
                }
            }
        }
#endif

#if defined(USE_FLASHFS)
        // If the blackbox device is onboard flash, then initialize and scan
        // it to identify the log files *before* starting the USB device to
        // prevent timeouts of the mass storage device.
        if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
            emfat_init_files();
        }
#endif
        // There's no more initialisation to be done, so enable DMA where possible for SPI
#ifdef USE_SPI
        spiInitBusDMA();
#endif
        if (mscStart() == 0) {
             mscWaitForButton();
        } else {
            systemResetFromMsc();
        }
    }
#endif

#ifdef USE_PERSISTENT_MSC_RTC
    // if we didn't enter MSC mode then clear the persistent RTC value
    persistentObjectWrite(PERSISTENT_OBJECT_RTC_HIGH, 0);
    persistentObjectWrite(PERSISTENT_OBJECT_RTC_LOW, 0);
#endif

#ifdef USE_I2C
    i2cHardwareConfigure(i2cConfig(0));

    // Note: Unlike UARTs which are configured when client is present,
    // I2C buses are initialized unconditionally if they are configured.

#ifdef USE_I2C_DEVICE_1
    i2cInit(I2CDEV_1);
#endif
#ifdef USE_I2C_DEVICE_2
    i2cInit(I2CDEV_2);
#endif
#ifdef USE_I2C_DEVICE_3
    i2cInit(I2CDEV_3);
#endif
#ifdef USE_I2C_DEVICE_4
    i2cInit(I2CDEV_4);
#endif
#endif // USE_I2C

#endif // TARGET_BUS_INIT

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#ifdef USE_VTX_RTC6705
    bool useRTC6705 = rtc6705IOInit(vtxIOConfig());
#endif

#ifdef USE_CAMERA_CONTROL
    cameraControlInit();
#endif

#ifdef USE_ADC
    adcInit(adcConfig());
#endif

    initBoardAlignment(boardAlignment());

    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, notify and don't arm.
        if (true
#if defined(USE_UNIFIED_TARGET)
            && isSystemConfigured()
#endif
            ) {
            indicateFailure(FAILURE_MISSING_ACC, 2);
        }
        setArmingDisabled(ARMING_DISABLED_NO_GYRO);
    }

    systemState |= SYSTEM_STATE_SENSORS_READY;

    // Set the targetLooptime based on the detected gyro sampleRateHz and pid_process_denom
    gyroSetLooptime(pidConfig()->pid_process_denom, pidConfig()->filter_process_denom);

    // Validate and correct the gyro config or PID loop time if needed
    validateAndFixGyroConfig();

    // Now reset the targetLooptime as it's possible for the validation to change the pid_process_denom
    gyroSetLooptime(pidConfig()->pid_process_denom, pidConfig()->filter_process_denom);

    // Finally initialize the gyro filtering
    gyroInitFilters();

    // Initialize PID control
    pidInit(currentPidProfile);

#ifdef USE_SERVOS
    servoInit();
#endif

#ifdef USE_SBUS_OUTPUT
    sbusOutInit();
#endif

#ifdef USE_PINIO
    pinioInit(pinioConfig());
#endif

#ifdef USE_PIN_PULL_UP_DOWN
    pinPullupPulldownInit();
#endif

#ifdef USE_PINIOBOX
    pinioBoxInit(pinioBoxConfig());
#endif

    LED1_ON;
    LED0_OFF;
    LED2_OFF;

    for (int i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
#if defined(USE_BEEPER)
        delay(25);
        if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT))) {
            BEEP_ON;
        }
        delay(25);
        BEEP_OFF;
#else
        delay(50);
#endif
    }
    LED0_OFF;
    LED1_OFF;

    imuInit();

    setpointInit();

    failsafeInit();

    rxInit();

    positionInit();

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
        gpsInit();
    }
#endif

#ifdef USE_LED_STRIP
    ledStripInit();

    if (featureIsEnabled(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        escSensorInit();
    }
#endif

#ifdef USE_FREQ_SENSOR
    if (featureIsEnabled(FEATURE_FREQ_SENSOR)) {
        freqInit(freqConfig());
    }
#endif

    rpmSourceInit();

#ifdef USE_RPM_FILTER
    if (featureIsEnabled(FEATURE_RPM_FILTER)) {
        rpmFilterInit();
    }
#endif

#ifdef USE_DYN_NOTCH_FILTER
    if (featureIsEnabled(FEATURE_DYN_NOTCH)) {
        dynNotchInit(dynNotchConfig());
    }
#endif

    if (featureIsEnabled(FEATURE_GOVERNOR)) {
        governorInit(currentPidProfile);
    }

#ifdef USE_USB_DETECT
    usbCableDetectInit();
#endif

#ifdef USE_FLASH_CHIP
    if (!(initFlags & FLASH_INIT_ATTEMPTED)) {
        flashInit(flashConfig());
        initFlags |= FLASH_INIT_ATTEMPTED;
    }
#endif
#ifdef USE_FLASHFS
    flashfsInit();
#endif

#ifdef USE_BLACKBOX
#ifdef USE_SDCARD
    if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
        if (sdcardConfig()->mode) {
            if (!(initFlags & SD_INIT_ATTEMPTED)) {
                sdCardAndFSInit();
                initFlags |= SD_INIT_ATTEMPTED;
            }
        }
    }
#endif
    blackboxInit();
#endif

    gyroStartCalibration(false);
#ifdef USE_BARO
    baroStartCalibration();
#endif

#if defined(USE_VTX_COMMON) || defined(USE_VTX_CONTROL)
    vtxTableInit();
#endif

#ifdef USE_VTX_CONTROL
    vtxControlInit();

#if defined(USE_VTX_COMMON)
    vtxCommonInit();
#endif

#ifdef USE_VTX_SMARTAUDIO
    vtxSmartAudioInit();
#endif

#ifdef USE_VTX_TRAMP
    vtxTrampInit();
#endif

#ifdef USE_VTX_RTC6705
    if (!vtxCommonDevice() && useRTC6705) { // external VTX takes precedence when configured.
        vtxRTC6705Init();
    }
#endif

#endif // VTX_CONTROL

#ifdef USE_TIMER
    // start all timers
    // TODO - not implemented yet
    timerStart();
#endif

    batteryInit(); // always needs doing, regardless of features.

#ifdef USE_RCDEVICE
    rcdeviceInit();
#endif // USE_RCDEVICE

#ifdef USE_PERSISTENT_STATS
    statsInit();
#endif

    // Initialize MSP
    mspInit();
    mspSerialInit();

/*
 * CMS, display devices and OSD
 */
#ifdef USE_CMS
    cmsInit();
#endif

#if defined(USE_OSD)
    displayPort_t *osdDisplayPort = NULL;
    osdDisplayPortDevice_e osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_NONE;
#endif

#if defined(USE_OSD)
    //The OSD need to be initialised after GYRO to avoid GYRO initialisation failure on some targets

    if (featureIsEnabled(FEATURE_OSD)) {
        osdDisplayPortDevice_e device = osdConfig()->displayPortDevice;

        switch(device) {

        case OSD_DISPLAYPORT_DEVICE_AUTO:
            FALLTHROUGH;

#if defined(USE_FRSKYOSD)
        // Test OSD_DISPLAYPORT_DEVICE_FRSKYOSD first, since an FC could
        // have a builtin MAX7456 but also an FRSKYOSD connected to an
        // uart.
        case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
            osdDisplayPort = frskyOsdDisplayPortInit(vcdProfile()->video_system);
            if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_FRSKYOSD) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_FRSKYOSD;
                break;
            }
            FALLTHROUGH;
#endif

#if defined(USE_MAX7456)
        case OSD_DISPLAYPORT_DEVICE_MAX7456:
            // If there is a max7456 chip for the OSD configured and detected then use it.
            if (max7456DisplayPortInit(vcdProfile(), &osdDisplayPort) || device == OSD_DISPLAYPORT_DEVICE_MAX7456) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;
                break;
            }
            FALLTHROUGH;
#endif

#if defined(USE_CMS) && defined(USE_MSP_DISPLAYPORT) && defined(USE_OSD_OVER_MSP_DISPLAYPORT)
        case OSD_DISPLAYPORT_DEVICE_MSP:
            osdDisplayPort = displayPortMspInit();
            if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_MSP) {
                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;
                break;
            }
            FALLTHROUGH;
#endif

        // Other device cases can be added here

        case OSD_DISPLAYPORT_DEVICE_NONE:
        default:
            break;
        }

        // osdInit will register with CMS by itself.
        osdInit(osdDisplayPort, osdDisplayPortDevice);

        if (osdDisplayPortDevice == OSD_DISPLAYPORT_DEVICE_NONE) {
            featureDisableImmediate(FEATURE_OSD);
        }
    }
#endif // USE_OSD

#if defined(USE_CMS) && defined(USE_MSP_DISPLAYPORT)
    // If BFOSD is not active, then register MSP_DISPLAYPORT as a CMS device.
#if defined(USE_OSD)
    if (!osdDisplayPort)
#endif
    {
        cmsDisplayPortRegister(displayPortMspInit());
    }
#endif

#ifdef USE_DASHBOARD
    // Dashbord will register with CMS by itself.
    if (featureIsEnabled(FEATURE_DASHBOARD)) {
        dashboardInit();
#ifdef USE_OLED_GPS_DEBUG_PAGE_ONLY
        dashboardShowFixedPage(PAGE_GPS);
#else
        dashboardResetPageCycling();
        dashboardEnablePageCycling();
#endif
    }
#endif

#ifdef USE_TELEMETRY
    // Telemetry will initialise displayport and register with CMS by itself.
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        telemetryInit();
    }
#endif

    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    // On F4/F7 allocate SPI DMA streams before motor timers
#if defined(STM32F4) || defined(STM32F7)
#ifdef USE_SPI
    // Attempt to enable DMA on all SPI busses
    spiInitBusDMA();
#endif
#endif

#ifdef USE_MOTOR
    motorPostInit();
    motorEnable();
#endif

    // On H7/G4 allocate SPI DMA streams after motor timers as SPI DMA allocate will always be possible
#if defined(STM32H7) || defined(STM32G4)
#ifdef USE_SPI
    // Attempt to enable DMA on all SPI busses
    spiInitBusDMA();
#endif
#endif

    swdPinsInit();

    unusedPinsInit();

    tasksInit();

    systemState |= SYSTEM_STATE_READY;
}
