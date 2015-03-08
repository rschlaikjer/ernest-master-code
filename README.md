# ArNest
Client implementation for a simple Arduino nest workalike

[Server Implementation](https://github.com/rschlaikjer/ernest-server)
[Master Node Schematics Implementation](https://github.com/rschlaikjer/ernest-master-eagle)

## What is this
This is the client code for a custom board that I'm using as a makeshift Nest
for my apartement. Most of the logic happens in the server, to avoid having to
reflash every time I change something.

This code manages reading the sensor data from slave nodes, posting it to the
controlling server and interpreting the response. There is also support for
an LCD readout displaying readouts from the master and each slave in turn.

Essentially, all this code does is:
- Grab a pair of temperature and pressure values
- Make a get request to the server with the temperature and pressure as args
- Parse the response for burn-y or burn-n
- Turn on/off the heater

![Finished Device](/master_v0.jpg?raw=true "Device")
