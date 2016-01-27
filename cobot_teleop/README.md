##Cobot_teleop package

This package contains nodes for controlling the movement
of Cobot by: gamepad, keyboard or by dead reckoning(reference).

(reference about what dead reckoning is)
Dead reckoning needs a TF between "odom" and "base_link". 


In the "info" directory are 3 configuration files:

gamepad.yaml: for specific gamepad information (which usb port to use , which axis etc.)

keyboard.yaml: for specific keyboard information (mapping of keys)

teleop.yaml: for general information (different scales)
