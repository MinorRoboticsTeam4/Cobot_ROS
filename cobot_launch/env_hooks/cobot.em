#Defauls for the CoBot launch environment

: ${COBOT_3D_SENSOR:=kinect}           #3D sensor to use
: ${COBOT_SERIAL_PORT:=/dev/usb0		   #/dev/ttyUSB0 etc.

: ${COBOT_BASE:=threemxl}				       #Which (base)board to use


export COBOT_3D_SENSOR
export COBOT_SERIAL_PORT
export COBOT_BASE
