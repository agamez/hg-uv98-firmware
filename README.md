# hg-uv98-firmware
LANCHONLH HG-UV98 APRS Dual Band Firmware

I requested this firmware to Wuxi Venus Information Technology Co., Ltd and they kindly sent it to me compressed in a .rar file with no license attached.

I asked for permission to modify and distribute it and was told:

> On Thu, 2024-11-14 at 16:54 +0800, Wuxi Venus Information Technology Co., Ltd wrote:
> Yes,you are allowed to modify.
> 
> regards,
> Dale

So, given that there's no license directly attached but we have permission to moodify and distribute the code, I think BSD-3c is appropriate, but please beware that we haven't been given an explicit license to use.


The project appears to be configured for the Keil µVision IDE, targeting the MCS-51 (8051) microcontroller family. Here are the steps to set up the project:

    Install Keil µVision IDE:
        Download and install Keil µVision from the Keil website.

    Open the Project:
        Launch Keil µVision.
        Open the project file UV98_868_220430_D4.uvproj located in the UV98_868_220430_D4 directory.

    Build the Project:
        Once the project is loaded, click on the "Build" button or use the menu option Project > Build Target to compile the project.

    Generate Hex File:
        Ensure the option to create a hex file is enabled (this is typically set in the project options).
        After building, the hex file will be generated in the specified output directory (.\OBJ\).

    Flash the Firmware:
        Use a compatible programmer to flash the generated hex file to the AT89C55 microcontroller.

Add these steps to your README file to guide users through the build process.

