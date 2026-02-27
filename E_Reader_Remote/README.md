# Bluetooth remote for e-reader (Kobo)

This is FW for a BT e-reader remote that runs on NodeMCU32s. It features two mechanical buttons for turning pages.

## Flashing

Build and flash code using ESD-IDF. Run commands from `OSuRV_2025\E_Reader_Remote\e_reader_remote`.

```
idf.py build
idf.py -p COM$ flash
```

## Usage

After turning the device on, it will be available for bluetooth pairing.