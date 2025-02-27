# ESPHome [![Discord Chat](https://img.shields.io/discord/429907082951524364.svg)](https://discord.gg/KhAMKrd) [![GitHub release](https://img.shields.io/github/release/esphome/esphome.svg)](https://GitHub.com/esphome/esphome/releases/)

<a href="https://esphome.io/">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://esphome.io/_static/logo-text-on-dark.svg", alt="ESPHome Logo">
    <img src="https://esphome.io/_static/logo-text-on-light.svg" alt="ESPHome Logo">
  </picture>
</a>

---

[Documentation](https://esphome.io) -- [Issues](https://github.com/esphome/issues/issues) -- [Feature requests](https://github.com/esphome/feature-requests/issues)

---

[![ESPHome - A project from the Open Home Foundation](https://www.openhomefoundation.org/badges/esphome.png)](https://www.openhomefoundation.org/)



Thanks pcr20, implementing from pcr20/esphome.

This may be different for my case of a MITM between two
devices (meter & inverter), plus listening to third device
on the bus (battery).
This requires modbus_controller with both a mute client with disable_send:True as the listener, and server on the same ttl-rs485 adapter & bus wires as the fake meter server, with another client on a second ttl-485 as the meter client, forming a MITM. 
The MITM functionality works fine with the existing head/dev branch
of esphome, nothing added.
pcr20's setup is a client listening alone, where the modbus byte
parsing identifies a request and response on the bus, which the 
ESP reads the response and parses data to sensors.

This branch contains modified versions of modbus and modbus_controller which includes an option to disable sending:
~~~
modbus_controller:
  - id: myid
    address: 0x0001
    modbus_id: modbus1
    update_interval: 10s
    setup_priority: -10
    command_throttle: 0ms
    disable_send: True
~~~
The components can be used by using the external_components feature in your yaml:
~~~
external_components:
  - source:
      type: git
      url: https://github.com/pcr20/esphome
      ref: dev
    components: [ modbus, modbus_controller ]
    refresh: 2minutes
~~~
The data is retrieved by creating a sensor - use a lambda to access the data:
~~~
sensor:
  - platform: modbus_controller
    modbus_controller_id: orno_we_504
    name: alldata
    id: alldata_id
    # 0x1 : modbus device address
    # 0x3 : modbus function code (read holding)
    # 0x00 : high byte of modbus register address
    # 0x00: low byte of modbus register address
    # 0x00: high byte of total number of registers requested
    # 0x11: low byte of total number of registers requested
    custom_command: [ 0x1, 0x3, 0x00, 0x00,0x00, 0x11]
    lambda: |-
      ESP_LOGD("Modbus Sensor Lambda","s: %d %02d %02d %02d %02d",data.size(),data[0],data[1],data[2],data[3],data[4]);
      return 0;
~~~