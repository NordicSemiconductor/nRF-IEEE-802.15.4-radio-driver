# nRF IEEE 802.15.4 radio driver

The nRF IEEE 802.15.4 radio driver implements the IEEE 802.15.4 PHY layer on the following Nordic Semiconductor SoCs:
    - nRF52811
    - nRF52833
    - nRF52840
    - nRF5340

The architecture of the nRF IEEE 802.15.4 radio driver is independent of the OS and IEEE 802.15.4 MAC layer.
It allows to use the driver in any IEEE 802.15.4 based stacks that implement protocols such as Thread, ZigBee, or RF4CE.

In addition, it was designed to work with multiprotocol applications.
The driver allows to share the RADIO peripheral with other PHY protocol drivers, for example, Bluetooth LE.

For more information and a detailed description of the driver, see the [Wiki](https://infocenter.nordicsemi.com/topic/struct_drivers/struct/radio_driver_latest.html).

Note that the *nRF-IEEE-802.15.4-radio-driver.packsc* file is a project building description used for internal testing in Nordic Semiconductor.
This file is NOT needed to build the driver with any other tool.
