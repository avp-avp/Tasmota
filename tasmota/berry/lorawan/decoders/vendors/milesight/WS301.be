# LoRaWAN Decoder file for Milesight WS301
#
# References
#  WS301 User Manual:     https://resource.milesight.com/milesight/iot/document/ws301-user-guide-en.pdf
#  TTN Device Repository: https://github.com/TheThingsNetwork/lorawan-devices/blob/master/vendor/milesight-iot/ws301.js

import string

global.ws301Nodes = {}

class LwDecoWS301
  static def decodeUplink(Name, Node, RSSI, FPort, Bytes)
    var data = {"Device":"Milesight WS301"}

    var valid_values = false

    var rssi = RSSI
    var last_seen = 1451602800
    var battery = 0
    var battery_last_seen = 1451602800
    var door_open = false
    var door_open_last_seen = 1451602800
    var installed = false
    var installed_last_seen = 1451602800

    if global.ws301Nodes.find(Node)
      battery             = global.ws301Nodes.item(Node)[4]
      battery_last_seen   = global.ws301Nodes.item(Node)[5]
      door_open           = global.ws301Nodes.item(Node)[6]
      door_open_last_seen = global.ws301Nodes.item(Node)[7]
      installed           = global.ws301Nodes.item(Node)[8]
      installed_last_seen = global.ws301Nodes.item(Node)[9]
    end

    var i = 0
    while i < (Bytes.size()-1)
        last_seen = tasmota.rtc('local')

        var channel_id = Bytes[i]
        i += 1
        var channel_type = Bytes[i]
        i += 1

        if channel_id == 0x01 && channel_type == 0x75
            battery_last_seen = tasmota.rtc('local')
            battery = Bytes[i]
            i += 1
            valid_values = true

        elif channel_id == 0x03 && channel_type == 0x00
            door_open_last_seen = tasmota.rtc('local')
            door_open = Bytes[i] == 0 ? false : true
            data.insert("DoorOpen", ( door_open ) ? true : false)
            i += 1
            valid_values = true

        elif channel_id == 0x04 && channel_type == 0x00
            installed_last_seen = tasmota.rtc('local')
            installed = Bytes[i] == 0 ? true : false
            data.insert("Installed", ( installed ) ? true : false)
            i += 1
            valid_values = true

        else
          # Ignore other
          valid_values = false
          i = Bytes.size()
        end
    end

    if valid_values
      if global.ws301Nodes.find(Node)
        global.ws301Nodes.remove(Node)
      end
      #                         sensor[0]   [1]   [2]        [3]                [4]      [5]   [6]        [7]                  [8]        [9]
      global.ws301Nodes.insert(Node, [Name, Node, last_seen, battery_last_seen, battery, RSSI, door_open, door_open_last_seen, installed, installed_last_seen])
    end

    return data
  end #decodeUplink()

  static def add_web_sensor()
    var msg = ""
    for sensor: global.ws301Nodes
      var name = sensor[0]
      if string.find(name, "WS301") > -1                                    # If LoRaWanName contains WS301 use WS301-<node>
        name = string.format("WS301-%i", sensor[1])
      end
      var name_tooltip = "Milesight WS301"
      var last_seen = sensor[2]
      var battery_last_seen = sensor[3]
      var battery = sensor[4]
      var rssi = sensor[5]
      msg += lwdecode.header(name, name_tooltip, battery + 100000, battery_last_seen, rssi, last_seen)

      # Sensors
      var door_open = sensor[6]
      var door_open_last_seen = sensor[7]
      var installed = sensor[8]
      var installed_last_seen = sensor[9]

      msg += "<tr class='htr'><td colspan='4'>&#9478;"                      # |
      msg += string.format(" %s %s", (door_open == true) ? "&#x1F513" : "&#x1F512", # Open or Closed lock - Door
                                     lwdecode.dhm(door_open_last_seen))

      msg += string.format(" %s %s", (installed == true) ? "&#x2705" : "&#x274C",   # Installed
                                     lwdecode.dhm(installed_last_seen))

      msg += "{e}"                                                          # = </td></tr>
    end
    return msg
  end #add_web_sensor()
end #class

LwDeco = LwDecoWS301
