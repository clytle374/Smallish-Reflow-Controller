# Smallish-Reflow-Controller

This is under development. incomplete documentation, not working, please don't use.

At this stage backups were getting to ugly and had to learn how to setup and upload to github.

This code was taken from Tiny Reflow Controller and adapted and modified, only
some parts and some of the structure remains, and its excellent display code. https://github.com/rocketscream/TinyReflowController

Thermocouple controller was changed to a MAX6657. The buttons were replaced with a encoder button to enable settings configuration at the panel.

A servo to open the door is being added.

The original code wouldn't run from on a 328P due to, I believe, running out of memory. So it was moved to a ATmega644 processor on a MightyCore board for development. So the MightyCore board needs loaded to build with the Arduino IDE.

Here is the basic setup of the reflow profiles.  There are 2 profiles(LEAD and ROHS), and the following parameters are separate for each one.




![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/423ee643de7cbcbdcfa11f0c58c11b50bf000d47/images/reflowplot.png?raw=true "Parameters and useage")



This is a overview of the settings, more later on the oven and how the code works.  Preheat is set at 2C/s. If your oven cannot hold a 2C/s ramp, I am not sure this code will work out very well on your oven. The ramp will continue until the SOAK_TEMP_HOLDOFF is reached.  Its value is the degrees before SOAK_TEMP. So if SOAK_TEMP is 150C and SOAK_TEMP_HOLDOFF is 25C then the elements will cut out at 125C. Temperature overshoot will reach the SOAK_TEMP and start the timer for SOAK_TIME.  There is an option for adding in a SOAK_RAMP to increase the temp linearly over the soak cycle, setting it to 0 disables it.

At the end of the soak cycle a 2C/s ramp is started.  Just like the SOAK_TEMP_HOLDOFF the heaters will cutoff the value of REFLOW_TEMP_HOLDOFF before REFLOW_TEMP temp is reached. Once REFLOW_TEMP is reached the timer will start and hold temperature until the set REFLOW_TIME is reached.


Why use those holdoff values?  I could not tune the PID of the system to not overshoot severely and hold a steady temperature. In researching this issue I found this quote

“According to Linear Technology’s Jim Williams, “The unfortunate relationship between servo systems and oscillators is very apparent in thermal control systems.” (Linear Applications Handbook, 1990).”

I devised a gimmick.  Here is a plot of setpoint vs real value.

![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/6c69fa6e21b77d11785d1b0f0b822d51ea9b8408/images/V35refloeResults.png?raw=true "Untuned Results")

BLUE = SETPOINT                                         RED = Actually temperature

Initially the setpoint is set above current temperature.  The program waits for the input value to reach the desired 2C/s, when that is reached the setpoint is set to the current value +4 and the setpoint ramp is started. The setpoint is dropped to below the current value when holdoff temperature is reached.  The program then waits for the overshoot to reach the soak temperature. During this time the C/s is monitored and if it falls below a set C/s threshold the system will bump the temp setpoint up to avoid the situation where the temperature isn’t reached.  The reflow ramp works the same way.

The oven is based on a $20 toaster oven from Walmart, 2 of them actually as the whole oven is cheaper than one heating element. Each oven has a 470 watt and 580 watt quartz elements.  The oven was fitted with an insert made from the second oven and aluminum flashing as the top half of the chamber for less heat mass. It was insulated with Kaowool, and cheap aluminum coated fiberglass adhesive on the inside. The quarts tubes were cut shorter with a diamond wheel in a dreamil tool and all 4 were put in the bottom, giving about 2KW.  In hindsight I would have tried using aluminum flashing with koawool across the bottom/back/top before I cut the elements. 4 holes were drilled though the oven sides and into the chamber.  Sheetmetal screws were then used as staybolts to hold it in place.  One of the elemet shields was used at the bottom front to protect the glass due to their close proximity.  It might not be needed, but it greatly stiffens the chamber.

![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/91d92f42080de914bb2c817a75af77370e52f0ce/images/PXL_20240418_001903719.jpg?raw=true "Chamber Installed")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/3b54d600ae06a638f3762178752a18d4dce408ca/images/PXL_20240224_103518909.MP.jpg?raw=true "Making the chamber")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/91d92f42080de914bb2c817a75af77370e52f0ce/images/PXL_20240416_003407255.jpg?raw=true "Staybolts")
![Alt text](https://github.com/clytle374/Smallish-Reflow-Controller/blob/40be4b0a984caed33c0f9d8965e814f7ed2b25fe/images/PXL_20240416_003355388.jpg?raw=true "Cut down elements")
