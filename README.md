# WakeBed

A bed-presence alarm. At the alarm time it rings only if you're in bed. It keeps ringing while you're in bed, goes quiet when you get up, and stops for good once the bed has been empty for a few minutes. Getting back in bed starts it ringing again.

- `firmware/WakeBed/WakeBed.ino`: Arduino sketch for the M5Stack AtomS3 Lite, the M5Stack Weight Unit (HX711) on the Atom's Grove port, a passive buzzer on PortABC Port B, and (optional) the M5Stack RTC Unit (HYM8563) on PortABC Port A.
- `docs/index.html`: control page using Web Bluetooth. It needs Chrome on Android or desktop, and is served by GitHub Pages from `/docs`.

## Clock

With the RTC Unit plugged in, the clock survives power cuts and reboots: the Atom reads the time from the RTC at boot, and every phone connection writes the phone's time back to it. Its pins are auto-detected at boot (`RTC found on G../G..` in the Serial Monitor). Without the RTC, the time is lost on power-off until the page connects again.

## Flashing

1. Arduino IDE 2, board package **M5Stack** (or **esp32** by Espressif), board **M5AtomS3**, *USB CDC On Boot: Enabled*.
2. Library Manager: **HX711 Arduino Library** (Bogdan Necula) and **NimBLE-Arduino** (h2zero, 2.x).
3. Upload. If upload can't connect: hold reset ~2 s until green, upload, then tap reset.

## Serial / BLE commands

Commands can be sent over BLE or typed in the Serial Monitor at 115200 baud. Separate several with `;`.

| Command | Effect |
|---|---|
| `status` | print state, signal, settings, RTC state and RTC-vs-system drift |
| `tare` | record the current reading as "bed empty" |
| `test=3` | ring for 3 s |
| `sim=15` | simulate the alarm in 15 s |
| `stop` | end a ringing session |
| `time=<unix s>` | set the clock (the page does this on connect) |
| `alarm=07:30` | set the alarm time |
| `en`, `ah`, `am`, `days`, `on`, `off`, `clr`, `cap`, `vol`, `freq`, `ramp`, `btn` `=<n>` | settings, saved to flash |

`days` is a bitmask: bit 0 = Monday … bit 6 = Sunday (127 = every day).

## LED

| Colour | Meaning |
|---|---|
| Purple blink | Clock not set: connect the page once after each power-up |
| Orange blink | Load sensor not responding |
| Blue | Phone connected |
| Green | Alarm armed; dim green means the alarm is off |
| Red flash | Ringing session |
