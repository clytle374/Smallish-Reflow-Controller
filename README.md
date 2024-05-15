# Smallish-Reflow-Controller


This code started as Tiny Reflow Controller and adapted and modified https://github.com/rocketscream/TinyReflowController  While some of the original code still exists like much of the display, it was more of a teaching tool for me at this point.

This runs on a ATmega644PA processor on a MightyCore board for development. So the MightyCore board library needs loaded to build with the Arduino IDE. It uses a 128x64 OLED display with the SSD1306 driver and MAX6675 thermocouple chip plus outputs for SSR(I also added a 4N35 opto), servo and buzzer.

Here is the basic setup of the reflow profiles.  There are 2 profiles one for lead and another for lead free, the parameters in this chart have separate entries for each profile.

![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/423ee643de7cbcbdcfa11f0c58c11b50bf000d47/images/reflowplot.png?raw=true "Parameters and useage")

This is a overview of the settings, more later on the oven and how the code works.  Preheat is set at 2C/s. If your oven cannot hold a 2C/s ramp, I am not sure this code will work out very well on your oven. The ramp will continue until the SOAK_TEMP_HOLDOFF is reached.  Its value is the number of degrees before the SOAK_TEMP. So if SOAK_TEMP is 150C and SOAK_TEMP_HOLDOFF is 25C then the elements will cut out at 125C. Temperature overshoot will come close to reaching the SOAK_TEMP. The SOAK_TIME will start when SOAK_TIMER_TEMP is reached. If the delta temp becomes too low during this the time the program will start adding small pulses of temperature increase, the display will show “RECOVER” at the bottom. There is an option for adding in a SOAK_RAMP to increase the temp linearly over the soak cycle, setting it to 0 disables it.

At the end of the soak cycle a 2C/s ramp is started.  Just like the SOAK_TEMP_HOLDOFF the heaters will cutoff at the value of REFLOW_TEMP_HOLDOFF before REFLOW_TEMP temp is reached. Once REFLOW_TIMER_TEMP is reached the timer will start and hold temperature until the set REFLOW_TIME is reached.


Why use those holdoff values?  I could not tune the PID of the system to not overshoot severely and hold a steady temperature. In researching this issue I found this quote “The unfortunate relationship between servo systems and oscillators is very apparent in thermal control systems.” by  Jim Williams

Results
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/e3206c26fff62b9263a2f295be6cd4586ccee949/images/goodWminorRecovery.png?raw=true "Good Run")
Blue= Setpoint       Orange= Temperature        Red= Output %
You can see the recovery right before the oven hits reflow temp. Below is an example of the TEMP_HOLDOFF values being too large making the reflow cycle too long.
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/e3206c26fff62b9263a2f295be6cd4586ccee949/images/recover.png?raw=true "Too much Recovery")

The oven is based on a $20 toaster oven from Walmart (MS54100112163), 2 of them actually as the whole oven is cheaper than one heating element. Each oven has a 470 watt and 580 watt quartz elements.  The oven was fitted with an insert made from the second oven and aluminum flashing as the top half of the chamber for less heat mass. It was insulated with Kaowool, and cheap aluminum coated fiberglass adhesive on the inside. The quarts tubes were cut shorter with a diamond wheel in a dreamil tool and all 4 were put in the bottom, giving about 2KW.  In hindsight I would have tried using aluminum flashing with koawool across the bottom/back/top before I cut the elements. 4 holes were drilled though the oven sides and into the chamber.  Sheet metal screws were then used as stay bolts to hold it in place.  One of the element shields was used at the bottom front to protect the glass due to their close proximity.  It might not be needed, but it greatly stiffens the chamber.

![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/3f3e0b47c759dd6e11878eca607b123d0159e093/images/PXL_20240429_055558839.jpg?raw=true "Display")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/91d92f42080de914bb2c817a75af77370e52f0ce/images/PXL_20240418_001903719.jpg?raw=true "Chamber Installed")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/3b54d600ae06a638f3762178752a18d4dce408ca/images/PXL_20240224_103518909.MP.jpg?raw=true "Making the chamber")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/91d92f42080de914bb2c817a75af77370e52f0ce/images/PXL_20240416_003407255.jpg?raw=true "Staybolts")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/40be4b0a984caed33c0f9d8965e814f7ed2b25fe/images/PXL_20240416_003355388.jpg?raw=true "Cut down elements")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/26c49e90dd1e2c1f2d230064f0cf6a58ef4e6dbc/images/PXL_20240426_064943152.jpg?raw=true "elements holder")
