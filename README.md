# Low Power Demo: Audio Capture

In this Audio Capture sample application, we demonstrate how to continuously capture audio data from a pair of external I2S MICs at low power. We reduced power consumption by making adjustments to the clocks and power configurations though SERVICES aiPM API calls: clock frequencies are reduced by running the MCU from RC rather than PLL and any power domains that are unused are disabled.

Make sure that you have setup and run the example projects from the [VSCode Getting Started Template](https://github.com/alifsemi/alif_vscode-template) before working on this project.