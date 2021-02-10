# nRF IEEE 802.15.4 Delayed Echo Client

The delayed echo client example sends a frame every two seconds and listens for a response from the echo server in a 10 millisecond reception window scheduled one second after a successful transmission.
This example application is designed to cooperate with the IEEE 802.15.4 delayed echo server located in `<repo-root>/examples/nrf_802154_delayed_echo_server`.

## Building

1. Generate the build scripts using the `cmake` build system.
    ```bash
    mkdir build
    cd build
    cmake ..
    ```
    The above command will generate build files for `make`.

2. Invoke build tool
    ```bash
    make
    ```

## Flashing

Flash the created `.hex` file by building the `flash` target.
```bash
make flash
```

If there are multiple development kits connected to the computer J-Link will display a GUI prompt requesting to select one of them.

## Testing

1. Build and flash this example to nRF52840 development kit
2. Build and flash the delayed echo server example to a second nRF52840 development kit
3. Observe flashing LEDs on both kits resulting from echo exchanges

Device UI:
* __LED1__: toggles when a successful frame transmission is completed
* __LED2__: toggles when a frame is received

__Note__: Transmitted frames are configured to require an acknowledgement.

The data traffic can be observed with a sniffer. The example transmits frames containing string `Nordic Semiconductor` as payload on channel 11.
