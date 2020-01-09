# Rythm_Game
- Peripherals used: DAC, GPIO, DMA, TIMER
- External interface types: Analog, GPIO

## Report
We developed a music-based game, something like guitar hero, with song select and a scoring system, where the goal 
is to press buttons in time with the rhythm of the music. We created six songs of varying difficulty using 8-bit 
audio and outputted on external speakers.  Six arcade buttons were implemented using GPIO. An adafruit 32x64 RGB LED matrix, driven 
also by the GPIO pins, was used to give visual cues for when buttons should be pressed. On adafruit.com, the 
six arcade buttons and a 32x64 led matrix product ID's were 2278, 3430, 3431, and 3432 respectively.
We also used an LM317 linear voltage regulator to power the LED matrix, and all this was driven using an __STM32F051R8T6__
microcontroller.

One major problem we faced was putting together two different sets of code. We had originally delegated tasks so that 
half of the group was working on the audio portion of the project and the other half on making the LED matrix function 
properly. When we finally came back together it took a lot of time make the two code sets work with each  other. 
We overcame this though by explaining each-others code and putting in a lot effort to iron out the bugs.  

One other challenge was implementing the timers. Changing the frequency of the timer would affect several parts of 
the code at once and made it hard to sync up the game-play and audio. We were eventually able to learn and understand 
how the different frequencies interacted with each other, and how to sync each part up properly.

If someone would liked to pursue this project, we would recommend finding someone to help explain how the LED matrix 
functions so that little to no time is spent searching documentations online, which are hard to find. We would also 
recommend close communication between team members so that when separate components of the project come together, the 
transition happens smoothly.

## Resources Used
- https://learn.adafruit.com/32x16-32x32-rgb-led-matrix/overview
- https://bikerglen.com/projects/lighting/led-panel-1up/

