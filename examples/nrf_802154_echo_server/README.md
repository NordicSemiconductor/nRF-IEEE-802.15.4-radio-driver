# nRF IEEE 802.15.4 Echo Server

nrf_802154_echo_server listens for frames and responds to the sender with the same frame it
received.
This example application is designed to cooperate with nrf_802154_echo_client.

The echo server example listens for frames and responds to the sender with a frame with the same payload it received.
This example application is designed to cooperate with the IEEE 802.15.4 echo client located in `<repo-root>/examples/nrf_802154_echo_client`.

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

LEDs: LED3 toggles when a frame is received.

Using sniffer: channel 11 is set, payload of every transmitted frame is the same as payload
of a received frame, source and destination addresses are swapped.


## Testing

1. Build and flash this example to nRF52840 development kit
2. Build and flash the echo client example to a second nRF52840 development kit
3. Observe flashing LEDs on both kits resulting from echo exchanges

Device UI:
* __LED3__: toggles when a frame is received

__Note__: Transmitted frames are configured to require an acknowledgement.

The data traffic can be observed with a sniffer. The example listens for frames on channel 11.
