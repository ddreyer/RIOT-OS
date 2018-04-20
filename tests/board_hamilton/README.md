Background
==========
The Hamilton board has SAMR21 SoC (Cortex M0+ CPU and AT86RF233 radio), HDC1080 (humidity and temperature sensors), TMP006 (object temperature), FXOS8700 (accleration and magnetic field), Button sensor, LED, APDS9007 (light sensor), and EKMB (PIR motion sensor).

This test application tests each module of the hamilton board.

Expected result
===============
When running this application, you should see sensing result of each sensor and an LED toggle, every SAMPLE_INTERVAL. The current implementation supports HDC1080, TMP006, Button sensor-based pulse counter, and LED toggle. 

To Do
===============
1) The other sensors will be added after corresponding sensor drivers are merged.
2) Clock configuration will be updated to provide low idle current.
3) After doing 1) and 2), you should see the idle current ~6 uA between each sampling.
