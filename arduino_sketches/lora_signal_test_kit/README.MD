# Description
This kit is composed of :
* a mobile transceiver wich display on a screen LoRa signal quality
* a fixed station which acknowledged mobile device messages

# Algorithm
The 2 devices establish a session with the worst bandwith possible and discuss until no message are received.
If it's possible, and the signal strength is good, both device try to increase their bandwith (reduce the Spreading Factor).


# Implementation
For now the implementation is preaty bad. Only one sketch exsts to program both devices, you need to choose the end to program.
